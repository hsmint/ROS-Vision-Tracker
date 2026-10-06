"""input_test — 모터 출력 OFF 상태에서 /target 입력 시나리오를 보내고 명령·상태를 기록한다(시험 도구).

팬 = angular.z(ex>0 → 음수), 틸트 = angular.y(ey>0 → 양수).
phase      입력                                     기대
center     x=0,    y=0,    z=0.05                   CENTERED, 팬 0, 틸트 0
right      x=+0.4, y=0,    z=0.05                   TRACKING, 팬 <0 (오른쪽으로), 틸트 0
left       x=-0.4, y=0,    z=0.05                   TRACKING, 팬 >0, 틸트 0
down       x=0,    y=+0.4, z=0.05                   TRACKING, 팬 0, 틸트 >0 (아래로)
up         x=0,    y=-0.4, z=0.05                   TRACKING, 팬 0, 틸트 <0
diagonal   x=+0.4, y=+0.4, z=0.05                   TRACKING, 팬 <0, 틸트 >0 (두 축 동시)
lost       x=+0.4, y=+0.4, z=0 (x·y를 일부러 남김)   LOST, 두 축 0 — x·y로 제어하지 않음
silence    발행 중단                                TIMEOUT, 두 축 0
frozen     같은 영상(같은 stamp) 반복 재발행          TIMEOUT — 같은 stamp는 신선한 입력이 아님
old_stamp  1초 전 stamp로 새 메시지                  TIMEOUT — 촬영 시각이 너무 오래됨
recover    x=+0.4, y=+0.4, z=0.05 정상              TRACKING, 팬 <0, 틸트 >0
틸트를 끈 컨트롤러(tilt_enabled=false)를 시험하려면 tilt:=false — 틸트 기대값이 모두 0이 된다.
"""
import csv
import json
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

from tracking_common import interface as I

PHASES = [
    # name, x, y, z, stamp mode, 기대 상태, 기대 팬 부호, 기대 틸트 부호 (0, -1, +1)
    ('center',    0.0,  0.0, 0.05, 'now',    I.CENTERED, 0,  0),
    ('right',     0.4,  0.0, 0.05, 'now',    I.TRACKING, -1, 0),
    ('left',     -0.4,  0.0, 0.05, 'now',    I.TRACKING, +1, 0),
    ('down',      0.0,  0.4, 0.05, 'now',    I.TRACKING, 0, +1),
    ('up',        0.0, -0.4, 0.05, 'now',    I.TRACKING, 0, -1),
    ('diagonal',  0.4,  0.4, 0.05, 'now',    I.TRACKING, -1, +1),
    ('lost',      0.4,  0.4, 0.0,  'now',    I.LOST,     0,  0),
    ('silence',  None, None, None, None,     I.TIMEOUT,  0,  0),
    ('frozen',    0.4,  0.4, 0.05, 'frozen', I.TIMEOUT,  0,  0),
    ('old_stamp', 0.4,  0.4, 0.05, 'old',    I.TIMEOUT,  0,  0),
    ('recover',   0.4,  0.4, 0.05, 'now',    I.TRACKING, -1, +1),
]


def sign_ok(values, expected):
    return all((v == 0) if expected == 0 else (v * expected > 0) for v in values)


class InputTest(Node):
    def __init__(self):
        super().__init__('input_test')
        self.phase_s = self.declare_parameter('phase_duration', 1.5).value
        self.out = Path(self.declare_parameter(
            'out_dir', str(I.results_dir())).value)
        self.tag = self.declare_parameter('tag', 'inputs').value
        self.tilt = self.declare_parameter('tilt', True).value   # 컨트롤러 tilt_enabled와 맞춘다
        self.pub = self.create_publisher(PointStamped, I.TOPICS['target']['name'], I.TARGET_QOS)
        self.create_subscription(Twist, I.TOPICS['gimbal_cmd']['name'], self.on_cmd, I.CMD_QOS)
        self.create_subscription(String, I.TOPICS['tracking_status']['name'], self.on_state, I.STATE_QOS)
        self.rows, self.state, self.sent, self.cmd = [], None, None, None
        self.frozen_stamp = None
        self.i, self.t_phase, self.t0 = -1, None, None
        self.started = False
        self.create_timer(1 / 30, self.publish)    # 카메라 30 fps 흉내
        self.get_logger().info('컨트롤러 대기 중...')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish(self):
        if not self.started:
            # 컨트롤러가 붙고 상태를 내기 시작하면 시작
            if self.state is None or self.pub.get_subscription_count() == 0:
                return
            self.started, self.t0 = True, self.now_s()
            self.next_phase()
        if self.now_s() - self.t_phase >= self.phase_s:
            if not self.next_phase():
                self.finish()
                return
        name, x, y, z, mode, *_ = PHASES[self.i]
        if mode is None:
            self.sent = None
            return
        now = self.get_clock().now()
        if mode == 'now':
            stamp = now
        elif mode == 'frozen':
            if self.frozen_stamp is None:
                self.frozen_stamp = now          # 첫 장만 새 영상, 이후 같은 영상 재발행
            stamp = self.frozen_stamp
        else:
            stamp = now - Duration(seconds=1.0)
        m = PointStamped()
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = 'camera_color_optical_frame'
        m.point.x, m.point.y, m.point.z = float(x), float(y), float(z)
        self.pub.publish(m)
        self.sent = (x, y, z, mode)

    def next_phase(self):
        self.i += 1
        if self.i >= len(PHASES):
            return False
        self.t_phase = self.now_s()
        self.get_logger().info(f'--- phase {PHASES[self.i][0]}')
        return True

    def on_cmd(self, msg):
        self.cmd = (msg.angular.z, msg.angular.y)

    def on_state(self, msg):
        # 컨트롤러는 같은 tick에 명령 → 상태 순으로 낸다. 상태 수신 시 기록하면 둘이 같은 tick이다.
        self.state = msg.data
        if not self.started or self.i >= len(PHASES) or self.cmd is None:
            return
        t = self.now_s()
        x, y, z, mode = self.sent if self.sent else (None, None, None, 'none')
        self.rows.append(dict(t=round(t - self.t0, 3), phase=PHASES[self.i][0],
                              t_in_phase=round(t - self.t_phase, 3), in_x=x, in_y=y, in_z=z, in_stamp=mode,
                              angular_z=round(self.cmd[0], 4), angular_y=round(self.cmd[1], 4),
                              status=msg.data))

    def finish(self):
        self.out.mkdir(parents=True, exist_ok=True)
        with open(self.out / f'{self.tag}_log.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            w.writeheader()
            w.writerows(self.rows)
        summary = []
        for name, x, y, z, mode, exp_state, exp_pan, exp_tilt in PHASES:
            if not self.tilt:
                exp_tilt = 0
                if exp_state == I.TRACKING and exp_pan == 0:
                    exp_state = I.CENTERED          # 틸트 OFF면 세로 오차만 있는 입력은 중앙
            rows = [r for r in self.rows if r['phase'] == name]
            steady = [r for r in rows if r['t_in_phase'] >= 0.7]       # 타임아웃(0.5s) 이후 구간
            states = sorted({r['status'] for r in steady})
            pans = [r['angular_z'] for r in steady]
            tilts = [r['angular_y'] for r in steady]
            first = next((r['t_in_phase'] for r in rows if r['status'] == exp_state), None)
            summary.append(dict(
                phase=name, input=dict(x=x, y=y, z=z, stamp=mode) if mode else '발행 중단',
                expected_state=exp_state, expected_pan_sign=exp_pan, expected_tilt_sign=exp_tilt,
                steady_states=states, steady_angular_z=sorted(set(pans)), steady_angular_y=sorted(set(tilts)),
                first_expected_state_after_s=first,
                transitions=[(r['t_in_phase'], r['status'], r['angular_z'], r['angular_y']) for prev, r in
                             zip([None] + rows, rows) if prev is None or prev['status'] != r['status']],
                passed=bool(steady) and states == [exp_state] and sign_ok(pans, exp_pan)
                and sign_ok(tilts, exp_tilt)))
        (self.out / f'{self.tag}_summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        for s in summary:
            self.get_logger().info(f"{'PASS' if s['passed'] else 'FAIL'} {s['phase']:9s} "
                                   f"상태={s['steady_states']} 팬={s['steady_angular_z']} 틸트={s['steady_angular_y']} "
                                   f"기대상태 도달 {s['first_expected_state_after_s']}s")
        self.get_logger().info(f'저장: {self.out / (self.tag + "_log.csv")}')
        raise SystemExit(0 if all(s['passed'] for s in summary) else 1)


def main():
    rclpy.init()
    node = InputTest()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
