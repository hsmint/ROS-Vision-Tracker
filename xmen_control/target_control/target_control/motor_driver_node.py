"""motor_driver — /cmd_vel(팬 angular.z, 틸트 angular.y)을 하드웨어에 전달하는 유일한 노드.

책임: 출력 허용 여부(output_enabled), 명령 감시(watchdog), 하드웨어 한계.
output_enabled=false(기본)이면 하드웨어에 쓰지 않고 '쓸 값'만 로그로 남긴다.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from tracking_common import interface as I


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.enabled = self.declare_parameter('output_enabled', False).value
        self.watchdog = self.declare_parameter('cmd_timeout', 0.2).value   # [s] 명령이 끊기면 정지
        self.hw_limit = self.declare_parameter('hw_max_speed', 0.6).value  # [rad/s] 팬
        self.hw_limit_tilt = self.declare_parameter('hw_max_speed_tilt', 0.4).value  # [rad/s] 틸트
        self.create_subscription(Twist, I.TOPICS['gimbal_cmd']['name'], self.on_cmd, I.CMD_QOS)
        self.create_timer(0.05, self.check)
        self.last_rx = None
        self.out = None
        if self.enabled:
            # 이 과제 환경에는 모터 하드웨어가 없다. 하드웨어 연결 시 write()를 구현한다.
            self.get_logger().error('output_enabled=true지만 연결된 모터 백엔드가 없다 → 출력하지 않는다.')
            self.enabled = False
        self.get_logger().info(f'모터 출력 {"ON" if self.enabled else "OFF(로그만)"}, '
                               f'명령 감시 {self.watchdog}s, 한계 팬 ±{self.hw_limit} 틸트 ±{self.hw_limit_tilt} rad/s')

    def on_cmd(self, msg):
        self.last_rx = self.get_clock().now()
        clamp = lambda v, lim: max(-lim, min(lim, v))
        self.apply((clamp(msg.angular.z, self.hw_limit), clamp(msg.angular.y, self.hw_limit_tilt)), '명령')

    def check(self):
        if self.last_rx is None:
            return
        if (self.get_clock().now() - self.last_rx).nanoseconds / 1e9 > self.watchdog and self.out != (0.0, 0.0):
            self.apply((0.0, 0.0), '명령 끊김(watchdog)')     # 두 축 모두 정지

    def apply(self, w, why):
        """w = (팬 rad/s, 틸트 rad/s)."""
        if w == self.out:
            return
        self.out = w
        mode = 'HW' if self.enabled else 'OFF'
        self.get_logger().info(f'[{mode}] pan={w[0]:+.3f} tilt={w[1]:+.3f} rad/s ({why})')


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
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
