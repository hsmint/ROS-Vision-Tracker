RealSense image publisher
========================

Requirements: ROS 2, NumPy, and the RealSense SDK Python bindings
(`pyrealsense2`) installed for the same Python interpreter as ROS 2.
Install the SDK and USB permissions using the
[RealSense SDK instructions](https://github.com/realsenseai/librealsense).
If a compatible Python wheel is available, the bindings can be installed with
`python3 -m pip install pyrealsense2` in your ROS Python environment.

Build and run from your ROS workspace:

```bash
source /opt/ros/lyrical/setup.bash  # Substitute your ROS distribution if needed.
colcon build --packages-select xmen_tracker --symlink-install
source install/setup.bash
ros2 run xmen_tracker realsense_node
```

Topics (`sensor_msgs/msg/Image`, reliable QoS with a queue depth of 5).
The local tracker requests reliable delivery to preserve RGB/depth pairs; remote
visualization requests best-effort delivery to avoid image retransmissions:

| Topic | Encoding | Data |
| --- | --- | --- |
| `/camera/color/image_raw` | `bgr8` | BGR color |
| `/camera/color/camera_info` | — | Color intrinsics (`sensor_msgs/CameraInfo`, transient local, published once) |
| `/camera/aligned_depth_to_color/image_raw` | `32FC1` | Depth in meters; invalid pixels are NaN |

Depth is aligned to color so the two images have corresponding pixels. Each
pair shares a ROS receipt timestamp and `camera_color_optical_frame` frame ID
(x right, y down, z forward). The node publishes color intrinsics on
`camera/color/camera_info` (aligned depth uses the same pixels) but no TF.
Topic names are relative and support ROS namespaces and remapping.

Startup parameters: `width` (640), `height` (360), `depth_width` (640), `depth_height` (360),
`fps` (30), `publish_hz` (30.0),
`serial_number` (empty selects an available camera), `frame_id`
(`camera_color_optical_frame`), and color sensor options matching the conditions used to
tune detection: `white_balance` (4600.0 K fixed; ≤ 0 = auto), `exposure` (0.0 = auto),
`auto_exposure_priority` (1.0), `backlight_compensation` (0.0).
Parameters are read-only after startup.
Each stream must support its selected resolution and the shared frame rate.
Depth is aligned to color, so both published images are 640×360 by default.
`publish_hz` must be positive and no greater than `fps`. Capture stays at `fps`;
only the latest frames are aligned and published at up to `publish_hz`, reducing
CPU use and raw image traffic. Set `publish_hz:=15.0` to reduce processing load.

```bash
ros2 run xmen_tracker realsense_node --ros-args -p width:=640 -p height:=360 -p depth_width:=640 -p depth_height:=360 -p fps:=30 -p publish_hz:=30.0
ros2 topic hz /camera/color/image_raw --qos-reliability best_effort
ros2 topic hz /camera/aligned_depth_to_color/image_raw --qos-reliability best_effort
```

Connect a depth-capable RealSense camera and close other programs using it before
starting. A disconnected camera or unsupported stream profile produces a startup
error. After a USB disconnection, reconnect the camera and restart the node.
