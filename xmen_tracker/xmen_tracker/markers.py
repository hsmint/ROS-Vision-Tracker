"""RViz visualization of normalized image tracking coordinates."""

import math

from rclpy.duration import Duration
from visualization_msgs.msg import Marker


def target_marker(target, lifetime):
    """Build a blue target box in a dedicated, non-metric image-plane frame.

    Image center is (0, 0); image bounds are [-1, 1] on each axis.
    Flip image y so upward motion appears upward in RViz's XY view.
    """
    marker = Marker()
    # This is a live HUD on the moving camera plane, not a world-space
    # measurement. Use the latest transform, avoiding remote-clock extrapolation.
    marker.frame_locked = True
    marker.header.frame_id = 'tracking_image'
    marker.ns = 'normalized_target'
    marker.id = 0
    marker.type = Marker.CUBE
    valid = (all(math.isfinite(v) for v in (target.point.x, target.point.y, target.point.z))
             and -1 <= target.point.x <= 1 and -1 <= target.point.y <= 1
             and 0 < target.point.z <= 1)
    marker.action = Marker.ADD if valid else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = target.point.x if valid else 0.0
    marker.pose.position.y = -target.point.y if valid else 0.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.08
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.lifetime = Duration(seconds=lifetime).to_msg()
    return marker
