"""Verify image conversion and paired publication without camera hardware."""

from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time
import numpy as np

from xmen_vision.realsense_node import RealSenseNode, image_message


def test_image_packs_noncontiguous_rgb():
    pixels = np.arange(36, dtype=np.uint8).reshape(2, 6, 3)[:, ::2]
    message = image_message(pixels, 'rgb8', Time(sec=12), 'optical')
    assert (message.height, message.width, message.step) == (2, 3, 9)
    assert bytes(message.data) == pixels.tobytes()
    assert message.header.stamp.sec == 12
    assert message.header.frame_id == 'optical'
    assert not message.is_bigendian


def test_publishes_aligned_pair_in_meters():
    color = np.zeros((2, 2, 3), dtype=np.uint8)
    depth = np.array([[0, 100], [250, 1000]], dtype=np.uint16)
    frames = Mock()
    frames.get_color_frame.return_value.get_data.return_value = color
    frames.get_depth_frame.return_value.get_data.return_value = depth
    node = SimpleNamespace(
        pipeline=Mock(), align=Mock(), depth_scale=0.002,
        frame_id='optical', color_publisher=Mock(), depth_publisher=Mock(),
        get_clock=Mock(),
    )
    node.align.process.return_value = frames
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=42)
    RealSenseNode.publish_frames(node)
    node.align.process.assert_called_once_with(node.pipeline.poll_for_frames.return_value)
    rgb = node.color_publisher.publish.call_args.args[0]
    metric = node.depth_publisher.publish.call_args.args[0]
    assert rgb.encoding == 'rgb8'
    assert metric.encoding == '32FC1'
    assert metric.step == 8
    assert rgb.header == metric.header
    actual = np.frombuffer(bytes(metric.data), dtype='<f4').reshape(2, 2)
    np.testing.assert_allclose(actual, [[np.nan, 0.2], [0.5, 2.0]])


def test_no_frames_does_not_publish():
    node = SimpleNamespace(pipeline=Mock(), color_publisher=Mock(), depth_publisher=Mock())
    node.pipeline.poll_for_frames.return_value = None
    RealSenseNode.publish_frames(node)
    node.color_publisher.publish.assert_not_called()
    node.depth_publisher.publish.assert_not_called()
