"""tracking_controller — /target을 받아 팬·틸트 속도 명령과 추적 상태를 낸다. /search 액션 서버.

2축: 팬 = ex → angular.z, 틸트 = ey → angular.y. tilt_enabled=false면 팬 1축(틸트 0).

책임: 입력 신선도 판정, 상태 결정, 명령 계산(부호·포화·데드밴드), 정지 보장.
모터 하드웨어는 모른다(motor_driver의 책임).

단일 스레드 실행기를 쓴다. 다중 스레드 실행기는 30 Hz 입력을 받는 동안 20 Hz 명령 타이머가
최대 0.3 s 밀렸다(실측). /search는 블로킹 루프 대신 20 Hz tick이 진행·종료를 판정한다.
"""
import math

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

from tracking_interfaces.action import Search
from tracking_common import interface as I


class TrackingController(Node):
    def __init__(self):
        super().__init__('tracking_controller')
        p = self.declare_parameter
        self.kp = p('kp', 1.0).value                 # [rad/s] per 1.0 정규화 오차
        self.cmd_sign = p('cmd_sign', -1.0).value    # REP-103: ex>0(오른쪽) → angular.z<0
        self.deadband = p('deadband', 0.05).value    # |ex| 이내는 정지
        self.max_speed = p('max_speed', 0.6).value   # [rad/s] 포화
        self.rate_hz = p('rate_hz', 20.0).value      # 명령 주기
        self.timeout = p('timeout', 0.5).value       # 마지막 신선한 입력 이후 [s]
        self.max_age = p('max_input_age', 0.5).value # 촬영 시각이 이보다 오래되면 신선하지 않음 [s]
        # 틸트: 세로 화각이 좁고 중력이 걸리는 축이라 팬과 따로 둔다
        self.tilt_on = p('tilt_enabled', True).value
        self.kp_tilt = p('kp_tilt', 0.8).value             # [rad/s] per 1.0 정규화 오차
        self.cmd_sign_tilt = p('cmd_sign_tilt', 1.0).value # REP-103: ey>0(아래) → angular.y>0(아래로)
        self.deadband_tilt = p('deadband_tilt', 0.05).value
        self.max_speed_tilt = p('max_speed_tilt', 0.4).value

        t = I.TOPICS
        self.sub = self.create_subscription(PointStamped, t['target']['name'], self.on_target, I.TARGET_QOS)
        self.cmd_pub = self.create_publisher(Twist, t['gimbal_cmd']['name'], I.CMD_QOS)
        self.state_pub = self.create_publisher(String, t['tracking_status']['name'], I.STATE_QOS)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.action = ActionServer(self, Search, I.ACTIONS['search']['name'],
                                   execute_callback=self.execute_search,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel)

        self.last = None              # 마지막 신선한 입력 (x, y, z)
        self.last_stamp_ns = None     # 마지막 수용한 header.stamp
        self.last_rx_ns = None        # 마지막 신선한 입력 수신 시각(노드 시계)
        self.rejected = {'old_or_duplicate_stamp': 0, 'too_old': 0, 'invalid': 0}
        self.search = None            # 실행 중인 /search: dict(handle, goal, t0, done)
        self.prev_state = None
        self.get_logger().info(
            f'kp={self.kp} cmd_sign={self.cmd_sign} deadband={self.deadband} max={self.max_speed} rad/s '
            f'rate={self.rate_hz} Hz timeout={self.timeout} s max_input_age={self.max_age} s | '
            + (f'tilt kp={self.kp_tilt} sign={self.cmd_sign_tilt} deadband={self.deadband_tilt} '
               f'max={self.max_speed_tilt} rad/s' if self.tilt_on else 'tilt OFF(팬 1축)'))

    # ---------- 입력 ----------
    def on_target(self, msg):
        now = self.get_clock().now().nanoseconds
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        if not all(map(math.isfinite, (x, y, z))) or z < 0 or z > 1 or \
                (z > 0 and (abs(x) > 1.0 + 1e-6 or abs(y) > 1.0 + 1e-6)):
            return self.reject('invalid', msg)
        # 같은/이전 시각 = 같은 영상의 재발행 → 신선한 입력이 아니다(타임아웃을 연장하지 않음)
        if self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
            return self.reject('old_or_duplicate_stamp', msg)
        if (now - stamp) / 1e9 > self.max_age:
            return self.reject('too_old', msg)
        self.last = (x, y, z)
        self.last_stamp_ns = stamp
        self.last_rx_ns = now

    def reject(self, why, msg):
        self.rejected[why] += 1
        n = self.rejected[why]
        if n == 1 or n % 30 == 0:
            self.get_logger().warning(f'입력 거부({why}) 누적 {n}: stamp={msg.header.stamp.sec}.'
                                      f'{msg.header.stamp.nanosec:09d} x={msg.point.x:.3f} z={msg.point.z:.4f}')

    def fresh(self, now):
        return self.last_rx_ns is not None and (now - self.last_rx_ns) / 1e9 <= self.timeout

    # ---------- 상태·명령 ----------
    @staticmethod
    def axis(err, sign, kp, deadband, limit):
        """한 축의 P 명령. 데드밴드 안이면 0, 밖이면 clamp(sign × kp × err, ±limit)."""
        if abs(err) < deadband:
            return 0.0
        return max(-limit, min(limit, sign * kp * err))

    def decide(self, now):
        """반환 (상태, 팬 angular.z, 틸트 angular.y). 정지가 기본값이다."""
        if self.last_rx_ns is None:
            return I.WAITING, 0.0, 0.0
        if not self.fresh(now):
            return I.TIMEOUT, 0.0, 0.0
        x, y, z = self.last
        if z <= 0.0:
            return I.LOST, 0.0, 0.0     # x·y는 보지 않는다
        wz = self.axis(x, self.cmd_sign, self.kp, self.deadband, self.max_speed)
        wy = self.axis(y, self.cmd_sign_tilt, self.kp_tilt, self.deadband_tilt,
                       self.max_speed_tilt) if self.tilt_on else 0.0
        if wz == 0.0 and wy == 0.0:
            return I.CENTERED, 0.0, 0.0
        return I.TRACKING, wz, wy

    def tick(self):
        now = self.get_clock().now().nanoseconds
        state, wz, wy = self.decide(now)
        if self.search is not None:
            sw = self.step_search(now)
            if sw is not None:
                state, wz, wy = I.SEARCHING, sw, 0.0     # 탐색은 팬만
        cmd = Twist()
        cmd.angular.z, cmd.angular.y = float(wz), float(wy)
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))
        if state != self.prev_state:
            age = '-' if self.last_rx_ns is None else f'{(now - self.last_rx_ns) / 1e6:.0f} ms'
            self.get_logger().info(f'상태 {self.prev_state} → {state}  pan={wz:+.3f} tilt={wy:+.3f} rad/s  '
                                   f'마지막 입력={self.last}  입력 경과={age}  거부={self.rejected}')
            self.prev_state = state

    # ---------- /search 액션 ----------
    def on_goal(self, goal):
        if not (goal.speed > 0 and goal.timeout > 0 and goal.direction in (-1.0, 1.0)):
            self.get_logger().warning(f'/search 거부: direction={goal.direction} speed={goal.speed} '
                                      f'timeout={goal.timeout}')
            return GoalResponse.REJECT
        if self.search is not None:
            self.get_logger().warning('/search 거부: 이미 탐색 중')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, _):
        return CancelResponse.ACCEPT

    async def execute_search(self, handle):
        g = handle.request
        done = Future()
        self.search = dict(handle=handle, w=g.direction * min(g.speed, self.max_speed),
                           timeout=g.timeout, t0=self.get_clock().now().nanoseconds, done=done)
        self.get_logger().info(f'/search 시작 direction={g.direction:+.0f} '
                               f'speed={abs(self.search["w"])} timeout={g.timeout}')
        return await done            # step_search가 종료를 판정하면 결과가 들어온다

    def step_search(self, now):
        """tick마다 호출. 계속 탐색하면 angular.z, 끝났으면 None."""
        s = self.search
        h = s['handle']
        elapsed = (now - s['t0']) / 1e9
        if h.is_cancel_requested:
            reason = 'CANCELED'
        elif self.fresh(now) and self.last[2] > 0:
            reason = 'FOUND'
        elif not self.fresh(now) and elapsed >= self.timeout:
            reason = 'NO_INPUT'          # 입력이 끊기면 찾았는지 알 수 없다 → 실패
        elif elapsed >= s['timeout']:
            reason = 'TIMEOUT'
        else:
            h.publish_feedback(Search.Feedback(elapsed=elapsed, angular_z=s['w'], state=I.SEARCHING))
            return s['w']
        result = Search.Result(found=reason == 'FOUND', reason=reason, elapsed=elapsed)
        {'CANCELED': h.canceled, 'FOUND': h.succeed}.get(reason, h.abort)()
        self.search = None               # 어떤 경우든 탐색 회전을 멈춘다(이번 tick부터 decide 결과)
        s['done'].set_result(result)
        self.get_logger().info(f'/search 종료 {reason} elapsed={elapsed:.2f}s')
        return None


def main():
    rclpy.init()
    node = TrackingController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise                  # 종료 신호로 컨텍스트가 닫힌 경우만 무시
    finally:
        # 종료 시에도 정지 명령을 한 번 보낸다(컨텍스트가 살아 있을 때)
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
