"""Publish RealSense color and aligned depth images."""

import math
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
import pyrealsense2 as rs

def image_message(array, encoding, stamp, frame_id):
    """Build a tightly packed, little-endian ROS image."""
    array = np.ascontiguousarray(array)
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height, message.width = array.shape[:2]
    message.encoding = encoding
    message.is_bigendian = False
    message.step = array.strides[0]
    message.data = array.tobytes()
    return message


class RealSenseNode(Node):
    """Capture synchronized frames without blocking the ROS executor."""

    def __init__(self):
        super().__init__('realsense_node')
        self.pipeline = None
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        defaults = {
            'width': 640,
            'height': 360,
            'depth_width': 640,
            'depth_height': 360,
            'fps': 30,
            'publish_hz': 30.0,
            'serial_number': '',
            'frame_id': 'camera_color_optical_frame',
            # 검출 HSV(xmen_tracker/config/detector.yaml)를 정한 실측 조건과 같게 둔다.
            # 화이트밸런스를 고정해야 조명이 바뀌어도 색상(H)이 흔들리지 않는다. 0 이하 = 자동
            'white_balance': 4600.0,
            'exposure': 0.0,                 # 0 이하 = 자동 노출(밝기 변화를 카메라가 흡수)
            'auto_exposure_priority': 1.0,   # 1 = 어두우면 프레임률을 낮춰서라도 노출을 늘림
            'backlight_compensation': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)
            )
        
        values = {name: self.get_parameter(name).value for name in defaults}

        for name in ('width', 'height', 'depth_width', 'depth_height', 'fps'):
            if values[name] <= 0:
                raise ValueError(f'{name} must be positive')
        if not math.isfinite(values['publish_hz']) or not 0 < values['publish_hz'] <= values['fps']:
            raise ValueError('publish_hz must be positive and no greater than fps')
        
        self.frame_id = values['frame_id']
        
        image_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.color_publisher = self.create_publisher(
            Image, 'camera/color/image_raw', image_qos
        )

        self.depth_publisher = self.create_publisher(
            Image, 'camera/aligned_depth_to_color/image_raw', image_qos
        )

        config = rs.config()
        if values['serial_number']:
            config.enable_device(values['serial_number'])
        # 메시지 encoding('bgr8')과 실제 데이터 순서를 맞춘다(예전: rgb8 데이터를 bgr8로 표시)
        for stream, pixel_format, width, height in (
            (rs.stream.color, rs.format.bgr8, values['width'], values['height']),
            (rs.stream.depth, rs.format.z16, values['depth_width'], values['depth_height']),
        ):
            config.enable_stream(
                stream, width, height, pixel_format, values['fps']
            )

        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        self.pipeline = pipeline
        self._apply_color_options(profile.get_device().first_color_sensor(), values)
        self.camera_info = self._camera_info(profile)
        # 초점거리는 바뀌지 않으므로 한 번 발행하고 늦게 붙는 구독자에게도 전달한다
        self.info_publisher = self.create_publisher(
            CameraInfo, 'camera/color/camera_info',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.info_publisher.publish(self.camera_info)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        # poll_for_frames returns the latest available frames; frames between
        # polls are dropped before costly alignment, conversion and ROS copies.
        self.timer = self.create_timer(1.0 / values['publish_hz'], self.publish_frames)

        self.get_logger().info(
            f"Capturing {values['width']}x{values['height']} at {values['fps']} FPS; "
            f"native depth {values['depth_width']}x{values['depth_height']}; "
            f"publishing up to {values['publish_hz']:g} Hz; "
            'aligned depth uses 32FC1 in meters.'
        )

    def _apply_color_options(self, sensor, values):
        """화이트밸런스·노출 등 컬러 센서 옵션. 지원하지 않는 옵션은 경고 후 넘어간다."""
        def set_option(option, value):
            if not sensor.supports(option):
                self.get_logger().warning(f'Color sensor does not support {option}')
                return
            r = sensor.get_option_range(option)
            sensor.set_option(option, min(max(float(value), r.min), r.max))
        if values['white_balance'] > 0:
            set_option(rs.option.enable_auto_white_balance, 0)
            set_option(rs.option.white_balance, values['white_balance'])
        else:
            set_option(rs.option.enable_auto_white_balance, 1)
        if values['exposure'] > 0:
            set_option(rs.option.enable_auto_exposure, 0)
            set_option(rs.option.exposure, values['exposure'])
        else:
            set_option(rs.option.enable_auto_exposure, 1)
        set_option(rs.option.auto_exposure_priority, values['auto_exposure_priority'])
        set_option(rs.option.backlight_compensation, values['backlight_compensation'])
        self.get_logger().info(
            f"Color white_balance={'auto' if values['white_balance'] <= 0 else values['white_balance']}, "
            f"exposure={'auto' if values['exposure'] <= 0 else values['exposure']}, "
            f"auto_exposure_priority={values['auto_exposure_priority']:g}")

    def _camera_info(self, profile):
        """컬러(정렬된 뎁스도 같은 좌표) 내부 파라미터."""
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width, info.height = intr.width, intr.height
        info.distortion_model = 'plumb_bob'
        info.d = [float(v) for v in intr.coeffs[:5]]
        info.k = [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish_frames(self):
        """Publish each available pair with a shared ROS receipt timestamp."""
        try:
            frames = self.pipeline.poll_for_frames()
            if not frames:
                return
            frames = self.align.process(frames)
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color or not depth:
                return
            stamp = self.get_clock().now().to_msg()
            color_array = np.asanyarray(color.get_data())
            raw_depth = np.asanyarray(depth.get_data())
            depth_array = raw_depth.astype('<f4')
            depth_array *= self.depth_scale
            depth_array[raw_depth == 0] = np.nan
            self.color_publisher.publish(
                image_message(color_array, 'bgr8', stamp, self.frame_id)
            )
            self.depth_publisher.publish(
                image_message(depth_array, '32FC1', stamp, self.frame_id)
            )
        except RuntimeError as error:
            self.get_logger().error(f'RealSense capture failed: {error}', throttle_duration_sec=5)

    def destroy_node(self):
        """Release the camera on shutdown or failed initialization."""
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except RuntimeError as error:
                self.get_logger().warning(f'Could not stop RealSense pipeline: {error}')
            finally:
                self.pipeline = None
        return super().destroy_node()


def main(args=None):
    """Run the RealSense publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = RealSenseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RuntimeError, ValueError) as error:
        rclpy.logging.get_logger('realsense_node').error(str(error))
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
