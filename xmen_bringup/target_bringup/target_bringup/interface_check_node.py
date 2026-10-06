"""interface_check — 실행 중인 그래프가 interface.py(=보고서 인터페이스 표)와 일치하는지 검사하고,
/target·/perception_status·/tracking_status·/cmd_vel을 기록한다(시험 도구).

검사: 토픽 타입, 발행/구독 노드, 발행·구독 QoS(신뢰성·depth), 실측 주기,
구독 측 호환성(reliable 구독자는 best-effort /target을 받지 못함 → incompatible QoS 이벤트).
"""
import csv
import json
import time
from pathlib import Path

import rclpy
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

from tracking_common import interface as I

TYPES = {'geometry_msgs/msg/PointStamped': PointStamped, 'geometry_msgs/msg/Twist': Twist,
         'std_msgs/msg/String': String}


def qos_str(q):
    return f'{q.reliability.name.lower()}/{q.history.name.lower()}/depth={q.depth}'


class InterfaceCheck(Node):
    def __init__(self):
        super().__init__('interface_check')
        self.duration = self.declare_parameter('duration', 7.0).value
        self.tag = self.declare_parameter('tag', 'e2e').value
        self.out = Path(self.declare_parameter(
            'out_dir', str(I.results_dir())).value)
        self.rx = {k: [] for k in I.TOPICS}
        self.rows = []
        self.latest = {}
        for key, t in I.TOPICS.items():
            self.create_subscription(TYPES[t['type']], t['name'],
                                     lambda m, k=key: self.on_msg(k, m), t['qos'])
        # 호환성 시험: reliable 구독자로 best-effort 발행 /target을 구독
        self.incompatible = []
        ev = SubscriptionEventCallbacks(incompatible_qos=lambda e: self.incompatible.append(
            e.last_policy_kind.name if hasattr(e.last_policy_kind, 'name') else str(e.last_policy_kind)))
        self.reliable_rx = 0
        self.create_subscription(PointStamped, I.TOPICS['target']['name'], self.on_reliable,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
                                 event_callbacks=ev)
        self.t0 = time.monotonic()
        self.create_timer(0.5, self.check_done)

    def on_reliable(self, _):
        self.reliable_rx += 1

    def on_msg(self, key, m):
        t = time.monotonic() - self.t0
        self.rx[key].append(t)
        if key == 'target':
            self.latest['target'] = (round(m.point.x, 4), round(m.point.z, 4))
        elif key == 'gimbal_cmd':
            self.latest['cmd'] = round(m.angular.z, 4)
            self.latest['cmd_tilt'] = round(m.angular.y, 4)
        elif key == 'perception_status':
            self.latest['perception'] = m.data
        else:
            # 컨트롤러는 같은 tick에 명령 → 상태 순으로 낸다. 상태 수신 시 기록하면 둘이 같은 tick이다.
            self.rows.append(dict(t=round(t, 3), target_x=self.latest.get('target', (None,))[0],
                                  target_z=self.latest.get('target', (None, None))[1],
                                  perception=self.latest.get('perception'),
                                  status=m.data, angular_z=self.latest.get('cmd'),
                                  angular_y=self.latest.get('cmd_tilt')))

    def graph(self):
        res = {}
        for key, t in I.TOPICS.items():
            pubs = self.get_publishers_info_by_topic(t['name'])
            subs = [s for s in self.get_subscriptions_info_by_topic(t['name'])
                    if s.node_name != 'interface_check']
            exp = qos_str(t['qos'])
            res[key] = dict(
                topic=t['name'], expected_type=t['type'],
                types=sorted({p.topic_type for p in pubs + subs}),
                publishers=[(p.node_name, qos_str(p.qos_profile)) for p in pubs],
                subscribers=[(s.node_name, qos_str(s.qos_profile)) for s in subs],
                expected_qos=exp,
                expected_publisher=t['publisher'], expected_subscriber=t['subscriber'])
            r = res[key]
            r['ok'] = (r['types'] == [t['type']] and
                       any(n == t['publisher'] and q == exp for n, q in r['publishers']) and
                       (t['subscriber'].startswith('(') or
                        any(n == t['subscriber'] and q == exp for n, q in r['subscribers'])))
        return res

    def rate(self, ts, a, b):
        ts = [t for t in ts if a <= t < b]
        if len(ts) < 2:
            return None
        d = [y - x for x, y in zip(ts, ts[1:])]
        return dict(n=len(ts), hz=round((len(ts) - 1) / (ts[-1] - ts[0]), 2), max_gap=round(max(d), 3))

    def check_done(self):
        if time.monotonic() - self.t0 < self.duration:
            if not hasattr(self, 'graph_snapshot') and time.monotonic() - self.t0 > 1.5:
                self.graph_snapshot = self.graph()
            return
        self.out.mkdir(parents=True, exist_ok=True)
        if self.rows:
            with open(self.out / f'{self.tag}_log.csv', 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0]))
                w.writeheader()
                w.writerows(self.rows)
        rates = {k: self.rate(v, 1.0, self.duration) for k, v in self.rx.items()}
        trans, prev = [], None
        for r in self.rows:
            key = (r['perception'], r['status'], r['angular_z'])
            if key != prev:
                trans.append(r)
                prev = key
        s = dict(graph=getattr(self, 'graph_snapshot', self.graph()), rates=rates,
                 qos_compat=dict(reliable_subscriber_received=self.reliable_rx,
                                 incompatible_qos_events=sorted(set(self.incompatible))),
                 transitions=trans)
        (self.out / f'{self.tag}_summary.json').write_text(json.dumps(s, ensure_ascii=False, indent=2))
        for k, g in s['graph'].items():
            self.get_logger().info(f"{'OK ' if g['ok'] else 'NG '} {g['topic']} pubs={g['publishers']} "
                                   f"subs={g['subscribers']} rate={rates[k]}")
        self.get_logger().info(f"reliable 구독자 수신={self.reliable_rx} incompatible={s['qos_compat']['incompatible_qos_events']}")
        for r in trans:
            self.get_logger().info(f"t={r['t']:.2f} target=({r['target_x']},{r['target_z']}) "
                                   f"perception={r['perception']} status={r['status']} cmd={r['angular_z']}")
        raise SystemExit(0)


def main():
    rclpy.init()
    node = InterfaceCheck()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
