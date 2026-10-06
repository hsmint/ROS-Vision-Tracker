"""search_test — /search 액션의 요청·진행·성공/실패·취소를 시험한다(시험 도구).

case       조건                                   기대 결과
found      탐색 1.0 s 뒤 목표가 보임(z>0)          FOUND (succeeded)
timeout    목표 계속 안 보임, timeout 1.0 s         TIMEOUT (aborted)
cancel     0.8 s 뒤 취소 요청                      CANCELED (canceled)
no_input   /target 발행 중단                       NO_INPUT (aborted)
invalid    speed=0                                 요청 거부(rejected)
"""
import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

from tracking_interfaces.action import Search
from tracking_common import interface as I

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
          GoalStatus.STATUS_CANCELED: 'canceled'}


class SearchTest(Node):
    def __init__(self):
        super().__init__('search_test')
        self.out = Path(self.declare_parameter(
            'out_dir', str(I.results_dir())).value)
        self.client = ActionClient(self, Search, I.ACTIONS['search']['name'])
        self.pub = self.create_publisher(PointStamped, I.TOPICS['target']['name'], I.TARGET_QOS)
        self.create_subscription(Twist, I.TOPICS['gimbal_cmd']['name'], self.on_cmd, I.CMD_QOS)
        self.create_subscription(String, I.TOPICS['tracking_status']['name'], self.on_state, I.STATE_QOS)
        self.z, self.publishing = 0.0, True
        self.state, self.cmds = None, []
        self.create_timer(1 / 30, self.publish)

    def publish(self):
        if not self.publishing:
            return
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.point.x, m.point.z = (0.2, self.z) if self.z > 0 else (0.0, 0.0)
        self.pub.publish(m)

    def on_state(self, msg):
        self.state = msg.data

    def on_cmd(self, msg):
        self.cmds.append((time.monotonic(), msg.angular.z, self.state))

    def run_case(self, name, goal, appear_after=None, cancel_after=None, stop_input=False):
        self.z, self.publishing = 0.0, True
        time.sleep(0.6)                                     # LOST 상태에서 시작
        if stop_input:
            self.publishing = False
        t0 = time.monotonic()
        fb = []
        send = self.client.send_goal_async(goal, feedback_callback=lambda f: fb.append(
            (round(time.monotonic() - t0, 2), round(f.feedback.elapsed, 2), f.feedback.angular_z, f.feedback.state)))
        while not send.done():
            time.sleep(0.01)
        handle = send.result()
        rec = dict(case=name, goal=dict(direction=goal.direction, speed=goal.speed, timeout=goal.timeout))
        if not handle.accepted:
            rec.update(accepted=False, status='rejected')
            self.get_logger().info(f'{name}: 요청 거부')
            return rec
        res = handle.get_result_async()
        canceled = False
        while not res.done():
            el = time.monotonic() - t0
            if appear_after is not None and el >= appear_after:
                self.z = 0.05
            if cancel_after is not None and el >= cancel_after and not canceled:
                handle.cancel_goal_async()
                canceled = True
            time.sleep(0.01)
        r = res.result()
        t1 = time.monotonic()
        during = [(round(t - t0, 2), round(w, 3), s) for t, w, s in self.cmds if t0 <= t <= t1 + 0.3]
        rec.update(accepted=True, status=STATUS.get(r.status, r.status), found=r.result.found,
                   reason=r.result.reason, elapsed=round(r.result.elapsed, 2),
                   feedback_count=len(fb), feedback_first=fb[:2], feedback_last=fb[-1:] if fb else [],
                   cmd_states_during=sorted({s for _, _, s in during if s}),
                   cmd_after_end=[c for c in during if c[0] > r.result.elapsed][:3])
        self.get_logger().info(f'{name}: {rec["status"]} reason={rec["reason"]} elapsed={rec["elapsed"]} '
                               f'feedback={len(fb)}')
        self.publishing = True
        return rec

    def run_all(self):
        if not self.client.wait_for_server(timeout_sec=10):
            self.get_logger().error('/search 서버 없음')
            return 1
        G = lambda d, s, t: Search.Goal(direction=d, speed=s, timeout=t)
        recs = [
            self.run_case('found', G(1.0, 0.3, 3.0), appear_after=1.0),
            self.run_case('timeout', G(-1.0, 0.3, 1.0)),
            self.run_case('cancel', G(1.0, 0.3, 5.0), cancel_after=0.8),
            self.run_case('no_input', G(1.0, 0.3, 5.0), stop_input=True),
            self.run_case('invalid', G(1.0, 0.0, 1.0)),
        ]
        expect = {'found': ('succeeded', 'FOUND'), 'timeout': ('aborted', 'TIMEOUT'),
                  'cancel': ('canceled', 'CANCELED'), 'no_input': ('aborted', 'NO_INPUT'),
                  'invalid': ('rejected', None)}
        for r in recs:
            st, reason = expect[r['case']]
            r['passed'] = r['status'] == st and (reason is None or r['reason'] == reason)
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / 'search_summary.json').write_text(json.dumps(recs, ensure_ascii=False, indent=2))
        for r in recs:
            self.get_logger().info(f"{'PASS' if r['passed'] else 'FAIL'} {r['case']}")
        return 0 if all(r['passed'] for r in recs) else 1


def main():
    rclpy.init()
    node = SearchTest()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        code = node.run_all()
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
