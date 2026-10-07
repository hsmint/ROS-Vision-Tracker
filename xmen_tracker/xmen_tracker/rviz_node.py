"""Render tracker results on a remote PC without repeating detection."""

import math

import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseStamped, TransformStamped
from message_filters import Subscriber, TimeSynchronizer
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker

from xmen_tracker.markers import target_marker
from xmen_tracker.overlay import annotate_target, image_plane, matched_bbox


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
        self.display_width = self.declare_parameter('display_width', 2.0).value
        self.display_distance = self.declare_parameter('display_distance', 1.0).value
        if not (math.isfinite(self.display_width) and self.display_width > 0
                and math.isfinite(self.display_distance) and self.display_distance > 0):
            raise ValueError('display_width and display_distance must be positive and finite')
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
        transport = self.declare_parameter(
            'preview_transport', 'compressed', ParameterDescriptor(read_only=True)).value
        if transport == 'compressed':
            self.preview_subscriber = self.create_subscription(
                CompressedImage, '/tracking/preview/compressed', self.on_preview,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        elif transport == 'raw':
            self.color_subscriber = Subscriber(
                self, Image, 'camera/color/image_raw',
                qos_profile=QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.bbox_subscriber = Subscriber(
                self, PolygonStamped, '/tracking/bbox', qos_profile=reliable)
            self.synchronizer = TimeSynchronizer(
                [self.color_subscriber, self.bbox_subscriber], queue_size=30)
            self.synchronizer.registerCallback(self.on_image)
        else:
            raise ValueError('preview_transport must be compressed or raw')
        self.marker_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'tracking_view'
        transform.child_frame_id = 'tracking_image'
        transform.transform.rotation.w = 1.0
        self.marker_tf.sendTransform(transform)
        self.pose_frame = self.declare_parameter('pose_frame', 'root').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(PoseStamped, '/camera/pose', output_qos)
        self.pose_timer = self.create_timer(0.1, self.publish_camera_pose)
        self.get_logger().info(
            'RViz outputs: /tracking/image, /tracking/image_plane, /target_marker, /camera/pose'
        )

    def publish_camera_pose(self):
        """Publish the camera body's current model pose, including pan and tilt."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.pose_frame, 'camera_link', rclpy.time.Time()
            )
        except TransformException:
            return  # Wait for the model and joint transforms to arrive.
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_publisher.publish(pose)

    def on_target(self, target):
        """Visualize control errors independently of image delivery."""
        marker = target_marker(target, self.marker_lifetime)
        # Match the upright image screen; sit slightly toward the viewer.
        marker.pose.position.x = self.display_distance - 0.005
        marker.pose.position.y = -target.point.x * self.display_width / 2
        marker.pose.position.z = -target.point.y * self.image_aspect * self.display_width / 2
        marker.scale.x = marker.scale.y = marker.scale.z = self.display_width * 0.04
        self.marker_publisher.publish(marker)

    def on_preview(self, message):
        """Decode a Pi-annotated frame without requiring another network topic."""
        if not self.wants_preview():
            return
        try:
            bgr = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError('Invalid JPEG preview')
            self.publish_overlay(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), message.header)
        except (ValueError, cv2.error, CvBridgeError) as error:
            self.get_logger().warning(f'Cannot decode preview: {error}', throttle_duration_sec=5.0)

    def wants_preview(self):
        return (self.annotated_publisher.get_subscription_count() > 0
                or self.image_plane_publisher.get_subscription_count() > 0)

    def on_image(self, image_msg, box_msg):
        """Optional raw transport for older publishers or direct bag playback."""
        if not self.wants_preview():
            return
        try:
            bbox = matched_bbox(image_msg, box_msg)
            rgb = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
            self.publish_overlay(annotate_target(rgb, bbox), image_msg.header)
        except (CvBridgeError, ValueError, cv2.error) as error:
            self.get_logger().error(f'Cannot draw RViz image: {error}', throttle_duration_sec=5.0)

    def publish_overlay(self, overlay, header):
        self.image_aspect = overlay.shape[0] / overlay.shape[1]
        if self.annotated_publisher.get_subscription_count() > 0:
            annotated = self.bridge.cv2_to_imgmsg(overlay, encoding='rgb8')
            annotated.header = header
            self.annotated_publisher.publish(annotated)
        if self.image_plane_publisher.get_subscription_count() > 0:
            self.image_plane_publisher.publish(image_plane(
                overlay, header.stamp, display_width=self.display_width,
                distance=self.display_distance))


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
