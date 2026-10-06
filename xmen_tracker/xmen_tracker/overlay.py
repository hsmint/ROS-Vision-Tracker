"""Draw remote debugging overlays without running detection."""

import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def image_plane(rgb, stamp):
    """Place RGB pixels upright at x=1, with forward X, left Y, and up Z."""
    height, width = rgb.shape[:2]
    points = np.empty((height, width), dtype=[
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4'),
    ])
    points['x'] = 1.0
    points['y'] = (width / 2 - np.arange(width, dtype=np.float32)) * (2.0 / width)
    points['z'] = ((height / 2 - np.arange(height, dtype=np.float32))
                   * (2.0 / width))[:, None]
    colors = rgb.astype(np.uint32)
    points['rgb'] = (colors[:, :, 0] << 16) | (colors[:, :, 1] << 8) | colors[:, :, 2]
    fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
              for name, offset in [('x', 0), ('y', 4), ('z', 8)]]
    fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
    return PointCloud2(
        header=Header(stamp=stamp, frame_id='tracking_image'),
        height=height, width=width, fields=fields, is_bigendian=False,
        point_step=16, row_step=16 * width, data=points.tobytes(), is_dense=True,
    )


def annotate_target(rgb, bbox):
    """Draw the selected blue object's bounding box on a copy of the RGB image."""
    image = rgb.copy()
    if bbox is not None:
        x, y, width, height = bbox
        cv2.rectangle(image, (x, y), (x + width - 1, y + height - 1), (0, 255, 0), 2)
        cv2.putText(image, 'Blue target', (x, max(15, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return image
