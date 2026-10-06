"""motor_driver — /cmd_vel(팬 angular.z, 틸트 angular.y)을 하드웨어에 전달하는 유일한 노드.

책임: 출력 허용 여부(output_enabled), 명령 감시(watchdog), 하드웨어 한계(속도·회전 범위),
OpenCR 시리얼 송수신, 현재 각도(/joint_states) 발행.
output_enabled=false(기본)이면 시리얼을 열지 않고 '보낼 값'만 로그로 남긴다.

OpenCR 시리얼: 한 줄 텍스트, 115200 baud (형식은 제어 담당 펌웨어와 맞춘다)
  보냄  "V <pan> <tilt>\\n"  팬·틸트 각속도 [deg/s] = /cmd_vel [rad/s] × 180/π × 부호(sign_pan·sign_tilt)
        명령을 받을 때마다(controller 20 Hz) 한 줄. 정지 = "V 0.00 0.00"을 계속 보낸다.
        명령이 cmd_timeout 동안 끊기거나 노드가 끝날 때도 0을 보낸다.
  받음  "P <pan> <tilt>\\n"  현재 각도 [deg], 원점(홈) 기준, OpenCR 부호 → × 부호로 /cmd_vel 기준(팬 + 왼쪽, 틸트 + 아래)
        → /joint_states 발행, 회전 범위 검사에 사용. 그 밖의 줄은 [OpenCR] 로그.

회전 범위(pan_limit_deg·tilt_limit_deg, /cmd_vel 기준 부호): 현재 각도가 범위 끝 이상이면 바깥 방향 속도를 0으로 한다.
현재 각도가 position_timeout 동안 없으면 범위를 확인할 수 없다 → require_position=true면 두 축 0, false면 경고만.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from tracking_common import interface as I

JOINTS = ['pan_joint', 'tilt_joint']


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        p = self.declare_parameter
        self.enabled = p('output_enabled', False).value
        self.watchdog = p('cmd_timeout', 0.2).value          # [s] 명령이 끊기면 정지
        self.hw_limit = p('hw_max_speed', 0.6).value         # [rad/s] 팬
        self.hw_limit_tilt = p('hw_max_speed_tilt', 0.4).value  # [rad/s] 틸트
        self.port = p('port', '/dev/ttyACM0').value
        self.baud = p('baud', 115200).value
        self.sign_pan = p('sign_pan', 1.0).value             # /cmd_vel 팬(+ 왼쪽) ↔ OpenCR 팬 부호. 실제 모터로 확인
        self.sign_tilt = p('sign_tilt', 1.0).value           # /cmd_vel 틸트(+ 아래) ↔ OpenCR 틸트 부호. 실제 모터로 확인
        self.limits = (list(p('pan_limit_deg', [-90.0, 90.0]).value),     # [deg] 원점 기준, /cmd_vel 부호
                       list(p('tilt_limit_deg', [-30.0, 30.0]).value))
        self.pos_timeout = p('position_timeout', 0.5).value  # [s] 이보다 오래된 각도는 쓰지 않는다
        self.require_pos = p('require_position', False).value  # true: 각도를 모르면 움직이지 않는다
        self.create_subscription(Twist, I.TOPICS['gimbal_cmd']['name'], self.on_cmd, I.CMD_QOS)
        self.joint_pub = self.create_publisher(JointState, I.TOPICS['joint_states']['name'], I.STATE_QOS)
        self.create_timer(0.05, self.check)
        self.last_rx = None
        self.out = None
        self.ser = None
        self.rx_buf = ''
        self.pos = None            # (팬, 틸트) [deg], /cmd_vel 부호
        self.pos_time = None
        if self.enabled:
            self.open_serial()
        self.get_logger().info(f'모터 출력 {"ON " + self.port if self.enabled else "OFF(로그만)"}, '
                               f'명령 감시 {self.watchdog}s, 한계 팬 ±{self.hw_limit} 틸트 ±{self.hw_limit_tilt} rad/s, '
                               f'범위 팬 {self.limits[0]} 틸트 {self.limits[1]} deg, '
                               f'부호 팬={self.sign_pan:+.0f} 틸트={self.sign_tilt:+.0f}, 각도 필수={self.require_pos}')

    def open_serial(self):
        try:
            import serial
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.05)
        except Exception as e:                       # 포트 없음·점유 중(시리얼 모니터·main.py) 등
            self.get_logger().error(f'시리얼 {self.port} 열기 실패: {e} → 출력하지 않는다.')
            self.enabled, self.ser = False, None

    def on_cmd(self, msg):
        self.last_rx = self.get_clock().now()
        clamp = lambda v, lim: max(-lim, min(lim, v))
        w = (clamp(msg.angular.z, self.hw_limit), clamp(msg.angular.y, self.hw_limit_tilt))
        w, why = self.limit_range(w)
        self.log_change(w, why)
        self.send_velocity(w)

    def limit_range(self, w):
        """현재 각도가 범위 끝이면 바깥 방향 속도를 0으로. 반환 (w, 로그 사유)."""
        if not self.position_fresh():
            if not self.enabled:
                return w, '명령'                             # 출력 OFF: 각도를 받을 수 없다(검사 생략)
            if self.require_pos:
                self.get_logger().warning('현재 각도 없음 → 범위 확인 불가, 정지', throttle_duration_sec=1.0)
                return (0.0, 0.0), '명령(각도 없음 → 정지)'
            self.get_logger().warning('현재 각도 없음 → 범위 확인 없이 전송', throttle_duration_sec=5.0)
            return w, '명령(범위 미확인)'
        out, hit = list(w), []
        for i, name in enumerate(('팬', '틸트')):
            lo, hi = self.limits[i]
            if (self.pos[i] >= hi and w[i] > 0) or (self.pos[i] <= lo and w[i] < 0):
                out[i] = 0.0
                hit.append(f'{name} {self.pos[i]:+.1f}°')
        return tuple(out), ('명령(범위 끝: ' + ', '.join(hit) + ')') if hit else '명령'

    def position_fresh(self):
        return (self.pos_time is not None and
                (self.get_clock().now() - self.pos_time).nanoseconds / 1e9 <= self.pos_timeout)

    def check(self):
        self.read_serial()
        if self.last_rx is None:
            return
        if (self.get_clock().now() - self.last_rx).nanoseconds / 1e9 > self.watchdog:
            self.log_change((0.0, 0.0), '명령 끊김(watchdog)')
            self.send_velocity((0.0, 0.0))                   # 끊긴 동안 0을 계속 보낸다

    def send_velocity(self, w):
        """w = (팬, 틸트) [rad/s] → "V <팬> <틸트>" [deg/s]."""
        pan = math.degrees(w[0]) * self.sign_pan + 0.0       # + 0.0: -0.00 대신 0.00
        tilt = math.degrees(w[1]) * self.sign_tilt + 0.0
        self.send(f'V {pan:.2f} {tilt:.2f}')

    def send(self, line):
        if not self.enabled:
            self.get_logger().info(f'[OFF] 보낼 값: {line}', throttle_duration_sec=1.0)   # 1초에 한 줄만
            return
        try:
            self.ser.write((line + '\n').encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'시리얼 쓰기 실패: {e}', throttle_duration_sec=1.0)

    def read_serial(self):
        """OpenCR 수신: "P <팬> <틸트>"는 현재 각도, 그 밖의 줄은 로그."""
        if self.ser is None:
            return
        try:
            self.rx_buf += self.ser.read(self.ser.in_waiting or 0).decode('utf-8', errors='replace')
        except Exception as e:
            self.get_logger().error(f'시리얼 읽기 실패: {e}', throttle_duration_sec=1.0)
            return
        *lines, self.rx_buf = self.rx_buf.split('\n')      # 마지막 조각은 다음에 이어 붙인다
        for line in (l.strip() for l in lines):
            if line.startswith('P '):
                self.on_position(line)
            elif line:
                self.get_logger().info(f'[OpenCR] {line}')

    def on_position(self, line):
        try:
            pan, tilt = (float(v) for v in line.split()[1:3])
        except ValueError:
            self.get_logger().warning(f'각도 줄 해석 실패: {line!r}', throttle_duration_sec=1.0)
            return
        self.pos = (pan * self.sign_pan, tilt * self.sign_tilt)   # OpenCR 부호 → /cmd_vel 부호
        self.pos_time = self.get_clock().now()
        msg = JointState()
        msg.header.stamp = self.pos_time.to_msg()
        msg.name = JOINTS
        msg.position = [math.radians(a) for a in self.pos]
        self.joint_pub.publish(msg)

    def log_change(self, w, why):
        """w = (팬 rad/s, 틸트 rad/s). 값이 바뀔 때만 로그."""
        if w == self.out:
            return
        self.out = w
        mode = 'HW' if self.enabled else 'OFF'
        self.get_logger().info(f'[{mode}] pan={w[0]:+.3f} tilt={w[1]:+.3f} rad/s ({why})')

    def close(self):
        if self.ser is not None:
            self.send_velocity((0.0, 0.0))
            self.ser.close()


def main():
    rclpy.init()
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise                  # 종료 신호로 컨텍스트가 닫힌 직후의 오류만 무시
    finally:
        try:
            node.close()           # 종료 시 속도 0 전송 후 포트 닫기
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
