"""Send a rate-limited, annotated JPEG preview independently of tracking."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PolygonStamped
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from xmen_tracker.overlay import annotate_target, matched_bbox


class PreviewNode(Node):
    def __init__(self):
        super().__init__('preview_node')
        self.bridge = CvBridge()
        self.pending = None
        self.hz = self.declare_parameter(
            'preview_hz', 5.0, ParameterDescriptor(read_only=True)).value
        self.quality = self.declare_parameter(
            'jpeg_quality', 70, ParameterDescriptor(read_only=True)).value
        if not math.isfinite(self.hz) or not 0 < self.hz <= 30:
            raise ValueError('preview_hz must be in (0, 30]')
        if not 1 <= self.quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')
        self.publisher = self.create_publisher(
            CompressedImage, '/tracking/preview/compressed',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.color = Subscriber(
            self, Image, 'camera/color/image_raw',
            qos_profile=QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.boxes = Subscriber(
            self, PolygonStamped, '/tracking/bbox',
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.sync = TimeSynchronizer([self.color, self.boxes], queue_size=10)
        self.sync.registerCallback(self.on_pair)
        self.timer = self.create_timer(1.0 / self.hz, self.publish_preview)
        self.get_logger().info(
            f'Annotated JPEG preview: up to {self.hz:g} Hz, quality {self.quality}; '
            '/tracking/preview/compressed')

    def on_pair(self, image, box):
        # Keep only the latest matched pair; no encoding in the input callback.
        self.pending = (image, box)

    def publish_preview(self):
        pair, self.pending = self.pending, None
        if pair is None or self.publisher.get_subscription_count() == 0:
            return
        image, box = pair
        try:
            bbox = matched_bbox(image, box)
            rgb = self.bridge.imgmsg_to_cv2(image, desired_encoding='rgb8')
            overlay = annotate_target(rgb, bbox)
            ok, jpeg = cv2.imencode(
                '.jpg', cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR),
                [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                raise ValueError('JPEG encoding failed')
            message = CompressedImage()
            message.header = image.header
            message.format = 'rgb8; jpeg compressed bgr8'
            message.data = jpeg.tobytes()
            self.publisher.publish(message)
        except (CvBridgeError, ValueError, cv2.error) as error:
            self.get_logger().warning(f'Cannot encode preview: {error}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PreviewNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
