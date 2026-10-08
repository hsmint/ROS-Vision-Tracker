"""Receive matching color and aligned depth frames from realsense_node.

Detection uses xmen_tracker.detector with xmen_tracker/config/detector.yaml
(measured-lighting HSV, real-size check, box-shape check, partially visible cube).
xmen_tracker.cube_tracker keeps the same cube across frames and bridges frames the
detector misses (occlusion) with CSRT, verified on the current frame (tracking section).
"""

import math

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, Twist
from message_filters import Subscriber, TimeSynchronizer
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

from xmen_tracker import detector
from xmen_tracker.cube_tracker import CubeTracker

JOINT_LIMITS_DEG = {'pan': (-180.0, 180.0), 'tilt': (-120.0, 120.0)}
# Match control_lite's eight encoder ticks of position tolerance.
LIMIT_WARNING_TOLERANCE_DEG = 8 * 360.0 / 4096


def depth_range_text(cfg):
    """Describe the depth validation range for the startup log."""
    d = detector.depth_cfg(cfg)
    return f'{d["min_m"]}-{d["max_m"]} m' if d else 'off'


def clamp(value, minimum, maximum):
    """Limit a value to the inclusive minimum and maximum."""
    return max(minimum, min(maximum, value))


class TrackerNode(Node):
    """Provide synchronized RGB and metric depth arrays for tracking."""

    def __init__(self):
        super().__init__('tracker_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.pending_frames = None
        self.last_stamp_ns = None
        self.last_target = None
        self.last_rx = self.get_clock().now()
        self.status = 'CAMERA_STALL'
        defaults = {
            # 검출 설정(HSV·크기·모양·뎁스 검증). 빈 문자열이면 xmen_tracker/config/detector.yaml
            'detector_config': '', 'stall_timeout': 0.5,
            # 이 변경 전 realsense_node로 녹화한 bag은 RGB 데이터에 'bgr8'이 붙어 있다 → true면 R·B를 바꿔 읽는다
            'legacy_rgb_bag': False,
            'kp': 1.5, 'cmd_sign': -1.0, 'deadband': 0.05, 'max_speed': 0.9,
            'kp_tilt': 1.2, 'cmd_sign_tilt': 1.0, 'deadband_tilt': 0.05,
            'max_speed_tilt': 0.6, 'tilt_enabled': True,
            'rate_hz': 20.0, 'tracking_hz': 20.0, 'timeout': 0.5, 'max_input_age': 0.5,
        }
        self.settings = {
            name: self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)
            ).value for name, value in defaults.items()
        }
        p = self.settings
        config_path = p['detector_config'] or str(detector.DEFAULT_CONFIG)
        self.detector_cfg = detector.load_config(config_path)   # 형식 오류는 여기서 ValueError
        # camera_info가 오기 전에는 설정의 근사 초점거리를 쓴다(640×360 ≈ 460 px)
        self.fx = self.fy = detector.focal_px(self.detector_cfg)
        self.focal_from_camera = False
        # 같은 설정 객체를 공유하므로 camera_info로 다시 계산한 크기 기준도 그대로 쓴다
        self.cube_tracker = CubeTracker(self.detector_cfg)
        if self.cube_tracker.p['csrt'] and not self.cube_tracker.use_csrt:
            self.get_logger().warning('OpenCV has no TrackerCSRT; tracking without CSRT bridging.')
        if not 0 < p['stall_timeout'] < float('inf'):
            raise ValueError('stall_timeout must be finite and positive')
        for name in ('kp', 'kp_tilt', 'max_speed', 'max_speed_tilt'):
            if not math.isfinite(p[name]) or p[name] < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        for name in ('rate_hz', 'tracking_hz', 'timeout', 'max_input_age'):
            if not math.isfinite(p[name]) or p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('deadband', 'deadband_tilt'):
            if not 0 <= p[name] <= 1:
                raise ValueError(f'{name} must be in [0, 1]')
        for name in ('cmd_sign', 'cmd_sign_tilt'):
            if p[name] not in (-1.0, 1.0):
                raise ValueError(f'{name} must be -1 or 1')
        
        target_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE,
        )
        self.target_publisher = self.create_publisher(PointStamped, '/target', target_qos)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', status_qos)
        self.cmd_timer = self.create_timer(1.0 / p['rate_hz'], self.publish_command)
        self.tracking_timer = self.create_timer(
            1.0 / p['tracking_hz'], self.process_latest_frame
        )
        self.joint_subscriber = self.create_subscription(
            JointState, '/joint_states', self.on_joint_states,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.bbox_publisher = self.create_publisher(
            PolygonStamped, '/tracking/bbox', QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE
            )
        )

        self.status_publisher = self.create_publisher(
            String, '/perception_status', status_qos
        )
        
        self.status_timer = self.create_timer(0.1, self.check_stall)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_status)
        image_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        # 실제 컬러 초점거리 → 크기 기반 면적 기준 재계산(한 번). bag 재생처럼 없으면 설정값 사용
        self.info_subscriber = self.create_subscription(
            CameraInfo, 'camera/color/camera_info', self.on_camera_info,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.get_logger().info(
            f'Detector config {config_path}: HSV {self.detector_cfg["target"]["hsv_ranges"]}, '
            f'min_area {self.detector_cfg["selection"]["min_area"]} px, '
            f'depth {depth_range_text(self.detector_cfg)}, '
            f'CSRT {"on" if self.cube_tracker.use_csrt else "off"}'
        )
        self.color_subscriber = Subscriber(
            self, Image, 'camera/color/image_raw', qos_profile=image_qos
        )
        self.depth_subscriber = Subscriber(
            self, Image, 'camera/aligned_depth_to_color/image_raw',
            qos_profile=image_qos,
        )

        # realsense_node stamps both images of a pair with exactly the same time.
        self.synchronizer = TimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber], queue_size=5
        )
        
        self.synchronizer.registerCallback(self.on_frames)
        self.get_logger().info('Waiting for synchronized RGB and aligned depth images.')

    def on_camera_info(self, message):
        """Use the real color focal length once for size checks and auto area limits."""
        fx, fy = message.k[0], message.k[4]
        if self.focal_from_camera or not (fx > 0 and fy > 0):
            return
        self.fx, self.fy = float(fx), float(fy)
        detector.set_focal(self.detector_cfg, self.fx, message.width or 640)
        self.focal_from_camera = True
        s = self.detector_cfg['selection']
        self.get_logger().info(
            f'Camera focal length fx={self.fx:.1f} fy={self.fy:.1f} px; '
            f'min_area {s["min_area"]} px, max_area_ratio {s["max_area_ratio"]}'
        )

    def on_joint_states(self, message):
        """Warn from measured joint angles, including encoder settling tolerance."""
        if len(message.name) != len(message.position):
            return
        reached = []
        for name, radians in zip(message.name, message.position):
            if name not in JOINT_LIMITS_DEG or not math.isfinite(radians):
                continue
            degrees = math.degrees(radians)
            lower, upper = JOINT_LIMITS_DEG[name]
            if degrees <= lower + LIMIT_WARNING_TOLERANCE_DEG:
                reached.append(f'{name}={degrees:.2f} deg (lower limit {lower:g} deg)')
            elif degrees >= upper - LIMIT_WARNING_TOLERANCE_DEG:
                reached.append(f'{name}={degrees:.2f} deg (upper limit {upper:g} deg)')
        if reached:
            self.get_logger().warning(
                'Joint travel limit reached: ' + '; '.join(reached),
                throttle_duration_sec=5.0,
            )

    def on_frames(self, color_msg, depth_msg):
        """Retain only the newest synchronized pair until the detection timer."""
        stamp = color_msg.header.stamp
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return
        if self.pending_frames is not None:
            pending_stamp = self.pending_frames[0].header.stamp
            if stamp_ns <= pending_stamp.sec * 1000000000 + pending_stamp.nanosec:
                return
        self.pending_frames = (color_msg, depth_msg)

    def process_latest_frame(self):
        """Process at most one fresh pair per tick; never replay a stored pair."""
        pair, self.pending_frames = self.pending_frames, None
        if pair is not None:
            self.process_frame(*pair)

    def process_frame(self, color_msg, depth_msg):
        """Convert a matched pair; depth[y, x] is in meters at RGB pixel (x, y)."""
        stamp = color_msg.header.stamp
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not 0 <= age <= self.settings['max_input_age']:
            return
        if (
            color_msg.height != depth_msg.height
            or color_msg.width != depth_msg.width
            or color_msg.header.frame_id != depth_msg.header.frame_id
        ):
            self.get_logger().warning(
                'Dropping images with mismatched dimensions or optical frames.',
                throttle_duration_sec=5.0,
            )
            return
        if depth_msg.encoding != '32FC1':
            self.get_logger().warning(
                f'Expected 32FC1 depth in meters, received {depth_msg.encoding}.',
                throttle_duration_sec=5.0,
            )
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            if self.settings['legacy_rgb_bag']:
                bgr = bgr[..., ::-1].copy()
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(
                f'Could not decode image pair: {error}', throttle_duration_sec=5.0
            )
            return

        if bgr.size == 0 or depth.size == 0:
            return
        self.latest_frame = (color_msg.header, bgr, depth)
        target = PointStamped()
        target.header = color_msg.header
        # 영상이 끊겼다 이어지면 큐브 위치가 달라졌을 수 있으므로 추적 대상을 잊고 새로 찾는다
        if (self.last_stamp_ns is not None
                and (stamp_ns - self.last_stamp_ns) / 1e9 > self.settings['stall_timeout']):
            self.cube_tracker.reset()
        # 색(HSV) → 면적 → 모양 → 뎁스 거리·실제 크기 검출 + 같은 큐브 유지·가림 구간 CSRT 확인
        depth_frame = (detector.depth_from_meters(depth, self.fx, self.fy)
                       if detector.depth_cfg(self.detector_cfg) is not None else None)
        result = self.cube_tracker.step(bgr, depth_frame)
        coordinates = result.target or (0.0, 0.0, 0.0)
        bbox = result.bbox_original if result.target else None
        if result.source == 'csrt' or result.partial:
            self.get_logger().debug(
                f'{result.source} target ({result.partial or "full"}), '
                f'detect {result.detect_ms:.1f} ms, csrt {result.csrt_ms:.1f} ms')
        target.point.x, target.point.y, target.point.z = (float(v) for v in coordinates)
        self.target_publisher.publish(target)
        box = PolygonStamped()
        box.header = color_msg.header
        if bbox is not None:
            x, y, width, height = bbox
            box.polygon.points = [
                Point32(x=float(x), y=float(y)),
                Point32(x=float(x + width - 1), y=float(y + height - 1)),
            ]
        self.bbox_publisher.publish(box)
        self.last_stamp_ns = stamp_ns
        self.last_rx = self.get_clock().now()
        had_target = self.last_target is not None and self.last_target[2] > 0
        self.last_target = coordinates
        self.set_status('OK' if target.point.z > 0 else 'NO_TARGET')
        # Stop immediately on target loss; the 20 Hz timer maintains the stop.
        # Repeating this on every empty frame adds camera-rate command traffic.
        if had_target and target.point.z <= 0:
            self.publish_command()

    @staticmethod
    def axis_velocity(error, gain, sign, deadband, limit):
        """Convert normalized image error to a bounded angular velocity."""
        if abs(error) < deadband:
            return 0.0
        return float(clamp(sign * gain * error, -limit, limit))

    def calculate_command(self, now_ns):
        """Return pan/tilt rad/s; absent, invalid, or stale input yields zero."""
        command = Twist()
        if self.last_target is None or self.last_stamp_ns is None:
            return command
        p = self.settings
        image_age = (now_ns - self.last_stamp_ns) / 1e9
        receive_age = (now_ns - self.last_rx.nanoseconds) / 1e9
        if not (0 <= image_age <= p['max_input_age']
                and 0 <= receive_age <= p['timeout']):
            return command
        x, y, area = self.last_target
        if (not all(math.isfinite(value) for value in (x, y, area))
                or not 0 < area <= 1 or abs(x) > 1 or abs(y) > 1):
            return command
        command.angular.z = self.axis_velocity(
            x, p['kp'], p['cmd_sign'], p['deadband'], p['max_speed']
        )
        if p['tilt_enabled']:
            command.angular.y = self.axis_velocity(
                y, p['kp_tilt'], p['cmd_sign_tilt'],
                p['deadband_tilt'], p['max_speed_tilt'],
            )
        return command

    def publish_command(self):
        """Keep the motor driver supplied with commands, including stop commands."""
        now_ns = self.get_clock().now().nanoseconds
        command = self.calculate_command(now_ns)
        # Clear expired data so a clock jump cannot reactivate an old command.
        if self.last_stamp_ns is not None and not (
            0 <= (now_ns - self.last_stamp_ns) / 1e9 <= self.settings['max_input_age']
            and 0 <= (now_ns - self.last_rx.nanoseconds) / 1e9 <= self.settings['timeout']
        ):
            self.last_target = None
        self.cmd_publisher.publish(command)

    def set_status(self, status):
        """Publish perception state transitions immediately."""
        if self.status != status:
            self.status = status
            self.publish_status()

    def publish_status(self):
        """Publish the current perception state heartbeat."""
        self.status_publisher.publish(String(data=self.status))

    def check_stall(self):
        """Report silence without republishing an old target."""
        age = (self.get_clock().now() - self.last_rx).nanoseconds / 1e9
        if age > self.settings['stall_timeout']:
            self.set_status('CAMERA_STALL')


def main(args=None):
    """Run the synchronized image subscriber."""
    rclpy.init(args=args)
    node = None
    try:
        node = TrackerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.cmd_publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
