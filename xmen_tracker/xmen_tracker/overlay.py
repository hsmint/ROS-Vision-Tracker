"""Draw remote debugging overlays without running detection."""

import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def image_plane(rgb, stamp, *, display_width=2.0, distance=1.0):
    """Place RGB pixels on a configurable flat display (not measured geometry)."""
    height, width = rgb.shape[:2]
    points = np.empty((height, width), dtype=[
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4'),
    ])
    points['x'] = distance
    points['y'] = (width / 2 - np.arange(width, dtype=np.float32)) * (display_width / width)
    points['z'] = ((height / 2 - np.arange(height, dtype=np.float32))
                   * (display_width / width))[:, None]
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


def matched_bbox(image, box):
    """Validate a same-frame detection and return its inclusive pixel rectangle."""
    if (image.width <= 0 or image.height <= 0
            or image.header != box.header):
        raise ValueError('Image and detection headers must match')
    points = box.polygon.points
    if len(points) == 0:
        return None
    if len(points) != 2:
        raise ValueError('Bounding box must contain zero or two points')
    left, right = points
    if not (0 <= left.x <= right.x < image.width
            and 0 <= left.y <= right.y < image.height):
        raise ValueError('Bounding box is outside the image')
    return (round(left.x), round(left.y),
            round(right.x - left.x + 1), round(right.y - left.y + 1))
