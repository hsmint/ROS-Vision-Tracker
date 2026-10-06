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
cd /home/pa03/Develop/xmen
colcon build --packages-select xmen_vision --symlink-install
source install/setup.bash
ros2 run xmen_vision realsense_node
```

Topics (`sensor_msgs/msg/Image`, best-effort sensor-data QoS):

| Topic | Encoding | Data |
| --- | --- | --- |
| `/camera/color/image_raw` | `rgb8` | RGB color |
| `/camera/aligned_depth_to_color/image_raw` | `32FC1` | Depth in meters; invalid pixels are NaN |

Depth is aligned to color so the two images have corresponding pixels. Each
pair shares a ROS receipt timestamp and `camera_color_optical_frame` frame ID
(x right, y down, z forward). The node does not publish TF or camera calibration.
Topic names are relative and support ROS namespaces and remapping.

Startup parameters: `width` (640), `height` (480), `fps` (30),
`serial_number` (empty selects an available camera), and `frame_id`
(`camera_color_optical_frame`). Parameters are read-only after startup.
Both streams must support the selected resolution and frame rate.

```bash
ros2 run xmen_vision realsense_node --ros-args -p width:=640 -p height:=480 -p fps:=30
ros2 topic hz /camera/color/image_raw --qos-reliability best_effort
ros2 topic hz /camera/aligned_depth_to_color/image_raw --qos-reliability best_effort
```

Connect a depth-capable RealSense camera and close other programs using it before
starting. A disconnected camera or unsupported stream profile produces a startup
error. After a USB disconnection, reconnect the camera and restart the node.
