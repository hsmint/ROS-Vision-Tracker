"""gimbal_sim — 팬·틸트 축과 카메라를 흉내 내는 폐루프 시뮬레이터(시험 도구, 실제 모터 아님).

/gimbal/cmd_vel을 적분해 팬·틸트 각도를 만들고, 고정된 목표 방향에서 ex·ey를 계산해 /target으로 낸다.
  팬: 방위각, REP-103처럼 왼쪽(반시계)이 +.  목표가 오른쪽(방위각 < 팬 각도)이면 ex > 0.
  틸트: 아래(피치 다운)가 +.                  목표가 아래(내려본 각 > 틸트 각도)이면 ey > 0.
화각(hfov·vfov) 밖이면 z=0(미검출). cmd_sign·cmd_sign_tilt를 뒤집었을 때의 현상을 보는 데 쓴다.
"""
import csv
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist

from tracking_common import interface as I


class GimbalSim(Node):
    def __init__(self):
        super().__init__('gimbal_sim')
        p = self.declare_parameter
        self.target_az = math.radians(p('target_az_deg', -15.0).value)       # 오른쪽 15°
        self.target_down = math.radians(p('target_down_deg', 8.0).value)    # 아래 8°
        self.half_hfov = math.radians(p('hfov_deg', 69.0).value) / 2        # D435 컬러
        self.half_vfov = math.radians(p('vfov_deg', 42.0).value) / 2
        self.duration = p('duration', 4.0).value
        self.tag = p('tag', 'sign_normal').value
        self.out = Path(p('out_dir', str(I.results_dir())).value)
        self.pan = self.tilt = 0.0
        self.wz = self.wy = 0.0
        self.pub = self.create_publisher(PointStamped, I.TOPICS['target']['name'], I.TARGET_QOS)
        self.create_subscription(Twist, I.TOPICS['gimbal_cmd']['name'], self.on_cmd, I.CMD_QOS)
        self.dt = 1 / 30
        self.t, self.rows, self.started = 0.0, [], False
        self.create_timer(self.dt, self.step)

    def on_cmd(self, msg):
        self.wz, self.wy = msg.angular.z, msg.angular.y
        self.started = True

    def step(self):
        if not self.started and self.pub.get_subscription_count() == 0:
            return
        self.pan += self.wz * self.dt
        self.tilt += self.wy * self.dt
        ex = -(self.target_az - self.pan) / self.half_hfov       # 오른쪽 +
        ey = (self.target_down - self.tilt) / self.half_vfov     # 아래 +
        visible = abs(ex) <= 1.0 and abs(ey) <= 1.0
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'camera_color_optical_frame'
        if visible:
            m.point.x, m.point.y, m.point.z = ex, ey, 0.05
        self.pub.publish(m)
        self.rows.append(dict(t=round(self.t, 3), pan_deg=round(math.degrees(self.pan), 2),
                              tilt_deg=round(math.degrees(self.tilt), 2), ex=round(ex, 4), ey=round(ey, 4),
                              visible=visible, angular_z=round(self.wz, 4), angular_y=round(self.wy, 4)))
        self.t += self.dt
        if self.t >= self.duration:
            self.finish()

    def finish(self):
        self.out.mkdir(parents=True, exist_ok=True)
        with open(self.out / f'{self.tag}_sim.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            w.writeheader()
            w.writerows(self.rows)
        first = lambda cond: next((r['t'] for r in self.rows if cond(r)), None)
        r0, r1 = self.rows[0], self.rows[-1]
        s = dict(tag=self.tag, target_az_deg=round(math.degrees(self.target_az), 2),
                 target_down_deg=round(math.degrees(self.target_down), 2),
                 start_ex=r0['ex'], end_ex=r1['ex'], start_ey=r0['ey'], end_ey=r1['ey'],
                 max_abs_ex=max(abs(r['ex']) for r in self.rows), max_abs_ey=max(abs(r['ey']) for r in self.rows),
                 end_pan_deg=r1['pan_deg'], end_tilt_deg=r1['tilt_deg'],
                 lost_at_s=first(lambda r: not r['visible']),
                 centered_at_s=first(lambda r: abs(r['ex']) < 0.05 and abs(r['ey']) < 0.05))
        (self.out / f'{self.tag}_sim.json').write_text(json.dumps(s, ensure_ascii=False, indent=2))
        self.get_logger().info(str(s))
        raise SystemExit(0)


def main():
    rclpy.init()
    node = GimbalSim()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
