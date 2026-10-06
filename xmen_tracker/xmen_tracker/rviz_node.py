"""Render tracker results on a remote PC without repeating detection."""

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PolygonStamped, TransformStamped
from message_filters import Subscriber, TimeSynchronizer
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker

from xmen_tracker.markers import target_marker
from xmen_tracker.overlay import annotate_target, image_plane


class RvizNode(Node):
    """Synchronize color and bounding boxes and provide RViz outputs."""

    def __init__(self):
        super().__init__('rviz_node')
        self.bridge = CvBridge()
        self.image_aspect = 1.0
        self.marker_lifetime = self.declare_parameter(
            'marker_lifetime', 0.5, ParameterDescriptor(read_only=True)
        ).value
        if not 0 < self.marker_lifetime < float('inf'):
            raise ValueError('marker_lifetime must be finite and positive')
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.annotated_publisher = self.create_publisher(Image, '/tracking/image', output_qos)
        self.image_plane_publisher = self.create_publisher(
            PointCloud2, '/tracking/image_plane', output_qos
        )
        self.marker_publisher = self.create_publisher(Marker, '/target_marker', output_qos)
        self.target_subscriber = self.create_subscription(
            PointStamped, '/target', self.on_target,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.color_subscriber = Subscriber(
            self, Image, 'camera/color/image_raw', qos_profile=reliable
        )
        self.bbox_subscriber = Subscriber(
            self, PolygonStamped, '/tracking/bbox', qos_profile=reliable
        )
        # Never draw a delayed detection over a newer image. Allow network jitter
        # with a bounded queue; missing pairs are dropped instead of reused.
        self.synchronizer = TimeSynchronizer(
            [self.color_subscriber, self.bbox_subscriber], queue_size=30
        )
        self.synchronizer.registerCallback(self.on_image)
        self.marker_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'tracking_view'
        transform.child_frame_id = 'tracking_image'
        transform.transform.rotation.w = 1.0
        self.marker_tf.sendTransform(transform)
        self.get_logger().info(
            'RViz outputs: /tracking/image, /tracking/image_plane, /target_marker'
        )

    def on_target(self, target):
        """Visualize control errors independently of image delivery."""
        marker = target_marker(target, self.marker_lifetime)
        # Match the upright image screen; sit slightly toward the viewer.
        marker.pose.position.x = 0.99
        marker.pose.position.y = -target.point.x
        marker.pose.position.z = -target.point.y * self.image_aspect
        self.marker_publisher.publish(marker)

    def on_image(self, image_msg, box_msg):
        """Draw a same-frame bounding box; an empty polygon means no target."""
        if image_msg.width == 0 or image_msg.height == 0:
            return
        self.image_aspect = image_msg.height / image_msg.width
        want_image = self.annotated_publisher.get_subscription_count() > 0
        want_plane = self.image_plane_publisher.get_subscription_count() > 0
        if not (want_image or want_plane):
            return
        if image_msg.header.frame_id != box_msg.header.frame_id:
            return
        points = box_msg.polygon.points
        if len(points) not in (0, 2):
            return
        bbox = None
        if points:
            left, right = points
            if not (0 <= left.x <= right.x < image_msg.width
                    and 0 <= left.y <= right.y < image_msg.height):
                return
            bbox = (round(left.x), round(left.y),
                    round(right.x - left.x + 1), round(right.y - left.y + 1))
        try:
            rgb = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
            overlay = annotate_target(rgb, bbox)
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(f'Cannot draw RViz image: {error}', throttle_duration_sec=5.0)
            return
        if want_image:
            annotated = self.bridge.cv2_to_imgmsg(overlay, encoding='rgb8')
            annotated.header = image_msg.header
            self.annotated_publisher.publish(annotated)
        if want_plane:
            self.image_plane_publisher.publish(image_plane(overlay, image_msg.header.stamp))


def main(args=None):
    """Run remote visualization independently of the tracking node."""
    rclpy.init(args=args)
    node = None
    try:
        node = RvizNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
