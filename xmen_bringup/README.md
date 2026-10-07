# xmen_bringup

ROS 2 Lyrical launches for the current `xmen_vision` and `xmen_tracker` nodes.

| Machine | Launch | Nodes |
|---|---|---|
| Raspberry Pi | `hardware_launch.py` | RealSense camera + tracker + JPEG preview + xmen_control/control |
| Remote PC | `rviz2_launch.py` | Overlay/marker node + RViz2 |

The tracker publishes `/target`, `/tracking/bbox`, `/perception_status`, and
`/cmd_vel`. The hardware launch starts `xmen_control/control`.
The tracker is the only `/cmd_vel` source; the legacy controller is not started.
Tracking continues without the remote PC.

## Build and live operation

On each machine, from the ROS workspace:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to xmen_bringup
source install/setup.bash
export ROS_DOMAIN_ID=50
```

Use the same domain on both machines, allow DDS discovery/data traffic across
the network, and synchronize system clocks. The Pi needs `pyrealsense2` and a
connected RealSense camera; the remote visualization process does not open it.

On the Pi:

```bash
ros2 launch xmen_bringup hardware_launch.py
```

The default camera capture is 30 FPS at 640×360 color and 640×360 native depth.
Depth is aligned to color, and image publication targets 30 Hz. Detection processes the newest synchronized pair at 20 Hz, dropping superseded
pairs before image conversion. Commands keep their independent 20 Hz timer.
Measured joint feedback from `control` remains 2 Hz. Override image processing
with `image_hz:=15.0` for lower CPU load (maximum 30). Camera publication targets 30 Hz; detection and commands target 20 Hz each.
Actual timing depends on CPU load and image delivery.
Local tracking keeps reliable raw RGB/depth delivery on the Pi. A separate
`preview_node` pairs the tracker boxes with their original color frames, draws
the selected target, and sends `/tracking/preview/compressed` as best-effort JPEG
at up to **5 Hz, quality 70**. The PC decodes it into the existing RViz image and
image-plane topics. Its default subscriptions do not request raw RGB or depth.
Preview encoding runs in a separate process and only when a preview subscriber
is connected. It still consumes some Pi CPU; it does not change detection or
command rates. JPEG size depends on the scene.

Adjust preview traffic independently of tracking:

```bash
# Pi: 15 Hz local tracking, 5 Hz annotated remote preview
ros2 launch xmen_bringup hardware_launch.py image_hz:=15.0 preview_hz:=5.0 jpeg_quality:=70
# Use preview_hz:=2.0 jpeg_quality:=50 for a lighter preview.
# Use start_preview:=false to disable the preview process entirely.
```

Deploy and rebuild `xmen_tracker` and `xmen_bringup` on both machines before
restarting the launches. The camera and tracker algorithms are unchanged by
this preview transport. Bag mode starts the same preview node using simulated
time. For an older Pi without the preview publisher, explicitly use
`preview_transport:=raw` on the PC (this sends full raw RGB over the network).

Measure actual receiving rates on the Pi while the launch is running:

```bash
ros2 topic hz /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /target /cmd_vel --qos-reliability best_effort --window 100
```

Expect up to 30 Hz images and approximately 20 Hz targets and commands (an immediate stop
command is also sent when a target is lost). Rates depend on scene load,
camera delivery and dropped frames; these are configured limits, not guarantees.

On the remote PC:

```bash
ros2 launch xmen_bringup rviz2_launch.py
```

RViz automatically loads `xmen_description/rviz/tracking.rviz`: CCTV RobotModel, annotated
image, image plane, target marker, grid, and optional TF axes. Saved views
include a robot close-up and a straight-on tracking view. `start_rviz:=false`
runs the publishers without opening RViz.

The launch loads `xmen_description/urdf/cctv.urdf` and its packaged STL meshes
through `robot_state_publisher`. `joint_state_publisher` supplies preview joint
positions by default. With actual `/joint_states` feedback, use
`publish_joint_states:=false` to avoid competing publishers. This preview does
not infer joint positions from `/cmd_vel`.

The flat tracking image plane is centered 0.25 m in front of `camera_link`
and follows the camera's pan/tilt chain. `camera_link` is attached to the CAD
camera housing (`part_3`) at its nominal front center, with X forward, Y left,
and Z up. `camera_color_optical_frame` has Z forward, X right, and Y down.
The mount is a CAD-based approximation, not calibrated lens extrinsics.

`/camera/pose` publishes `geometry_msgs/PoseStamped` at 10 Hz in the URDF root
frame. RViz shows it as red/green/blue X/Y/Z axes at the camera. This is the
model pose: physical motion requires correctly mapped joint feedback.

The image plane is a flat RGB preview, not a depth point cloud. Its width is
0.4 m and its target marker uses the same scale. Optional display offsets are
relative to the moving `camera_link` (zero by default):

```bash
ros2 launch xmen_bringup rviz2_launch.py display_y:=0.0 display_z:=0.0 \
  display_width:=0.4 display_distance:=0.25 display_yaw:=0.0
```

`display_x`, `display_roll`, and `display_pitch` are also available. Rotation
angles are radians. Restart the launch after changing layout parameters. If
changing image width, adjust RViz point size to approximately width / 640 for
640-pixel images. The fixed frame is `root`; a different URDF root requires a
matching RViz Fixed Frame. Override `urdf_file:=/path/robot.urdf` (which must
define `camera_link`) or `rviz_config:=/path/custom.rviz` as needed.

Install missing runtime dependencies before building:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Tracker tuning is in `param/tracker.yaml`; an alternate file can be supplied
with `params_file:=/absolute/path/tracker.yaml`.

### Check what the Pi is doing

Use domain 50 on both machines (`export ROS_DOMAIN_ID=50`). The PC's saved
RViz configuration displays the annotated image, image plane, and target marker.
The green box is drawn from the Pi's same-frame detection; an empty detection
produces an image without a box. Images retain the original capture header.

```bash
# On the PC: these checks do not subscribe to raw camera images.
ros2 topic hz /tracking/preview/compressed --qos-reliability best_effort
ros2 topic echo /perception_status
ros2 topic echo /target --qos-reliability best_effort
```

`/perception_status` reports `OK`, `NO_TARGET`, or `CAMERA_STALL` on changes and
once per second while the tracker is running. If the tracker stops, that
heartbeat also stops. RViz may retain its last picture when input stops: use
the heartbeat and incoming preview rate to distinguish live tracking from a
frozen display. Preview frames are published only for matching image/detection
pairs; no stale frame is repeatedly sent. Target markers remain independent of
preview delivery. Avoid raw image viewers or remote raw-topic rate checks when
measuring network savings, since they request the full stream again.

## Record color and depth

While live mode is running, record on the Pi (or on the PC if the network can
carry both raw image streams):

```bash
ros2 bag record -o ~/bags/target_run \
  /camera/color/image_raw /camera/aligned_depth_to_color/image_raw
```

Stop recording with Ctrl-C. Use a new output directory for each recording.
The camera publishes synchronized `rgb8` color and aligned `32FC1` depth in
meters with identical header timestamps. Both topics are required; arbitrary
bags containing unaligned depth, millimeter depth, or mismatched timestamps
are not compatible without conversion. Raw recording needs substantial disk
throughput and space (about 48 MB/s at 640×360, 30 FPS before storage overhead).

## Play a bag instead of opening the camera

Stop the live Pi launch first. Run the tracker and playback on either machine:

```bash
ros2 launch xmen_bringup hardware_launch.py \
  use_bag:=true bag_path:=/absolute/path/to/target_run
```

Run the visualization locally or on the remote PC:

```bash
ros2 launch xmen_bringup rviz2_launch.py use_bag:=true
```

`use_bag` is a **startup launch argument**: it selects the source and sets the
ROS `use_sim_time` parameter on the consuming nodes. It does not dynamically
detect a running bag or switch an already running camera. Restart the launches
when changing modes. Recording a live camera does not require `use_bag:=true`.

With a `bag_path`, the launch starts `ros2 bag play --clock`, waits two seconds
before playback, and replays only the two image inputs. It applies reliable
image QoS using `param/bag_qos.yaml` so the tracker subscriptions can connect.
Recorded `/cmd_vel`, targets, and detections are not replayed. The tracker and
visualization remain open at the end of playback; stop them with Ctrl-C.

### If ros2 bag play is already running separately

Leave `bag_path` empty and launch both machines with `use_bag:=true`. Start only
one player and ensure it publishes `/clock`:

```bash
ros2 launch xmen_bringup hardware_launch.py use_bag:=true
# Another sourced terminal:
ros2 bag play /absolute/path/to/target_run --clock 100 --delay 2 \
  --qos-profile-overrides-path "$(ros2 pkg prefix --share xmen_bringup)/param/bag_qos.yaml" \
  --topics /camera/color/image_raw /camera/aligned_depth_to_color/image_raw
```

Do not leave a live camera publishing the same topics during playback. If the
player was already running without `--clock`, restart it with that option.
Restart the tracker before replaying from the beginning or seeking backward:
the current tracker rejects older/duplicate image timestamps. Bag mode is for
offline analysis; keep hardware motor drivers stopped because pausing the bag
also pauses simulated-time timers.

Rosbag clock, topic filtering, and QoS behavior follow the
[upstream rosbag2 documentation](https://github.com/ros2/rosbag2).

## OpenCR control

Live `hardware_launch.py` starts only `xmen_control/control` for serial control.
It does not start `target_control` or the interactive `main_control` program.
The tracker remains the only `/cmd_vel` publisher. Bag mode skips control.

```bash
ros2 launch xmen_bringup hardware_launch.py port:=/dev/ttyACM0 auto_home:=true
# Camera and tracker only:
ros2 launch xmen_bringup hardware_launch.py start_control:=false
```

The control node opens the serial port at 115200 baud. `auto_home:=true` is its
existing default: it waits for verified firmware and an idle board, homes once,
and pauses tracking commands until homing completes. `auto_home:=false` skips
node-startup homing and saves automatic boot homing as disabled. Opening serial
may reset the board, so its previously saved startup behavior may happen before
the node can update that setting.

This node requires `OPENCR_CONTROL_V3_RAD_VELOCITY` firmware and uses radians
per second. It publishes `/joint_states` and `/status` and accepts `/opencr/home`
and `/opencr/stop`. The old `output_enabled` and `baud` launch arguments are
removed because this node does not support them. Its joint names (`pan`, `tilt`)
still need a verified mapping to the CAD URDF before physical joint feedback
can animate that model correctly.
