# xmen_tracker

The tracker also monitors measured `/joint_states` and logs a warning when pan
reaches ±180° or tilt reaches ±120°, allowing 8 encoder ticks (about 0.70°)
for settling near the boundary. Warnings include the measured angle and limit,
repeat at most once every 5 seconds, and do not change motion commands. The
tracker and controller must share the same ROS domain to receive joint feedback.

Tracks the blue target using synchronized RGB and aligned metric depth from
`xmen_vision`. The worker publishes `/target` and `/tracking/bbox`; a separate
remote RViz node draws overlays and RViz markers. This package performs
visual tracking and publishes pan/tilt angular velocities on `/cmd_vel`. A
separate motor driver handles serial communication and hardware limits.

From the ROS workspace:

```bash
colcon build --packages-select xmen_vision xmen_tracker
source install/setup.bash
ros2 run xmen_vision realsense_node
```

In another sourced terminal on the camera PC:

```bash
ros2 run xmen_tracker tracker_node
```

In a third Pi terminal, start the independent preview encoder (the hardware
bringup launch starts it automatically):

```bash
ros2 run xmen_tracker preview_node --ros-args -p preview_hz:=5.0 -p jpeg_quality:=70
```

Run only one detector publishing `/target` at a time.

On the remote PC, build this package and start the RViz node in a sourced ROS
terminal:

```bash
colcon build --packages-select xmen_tracker
source install/setup.bash
ros2 run xmen_tracker rviz_node
```

Run RViz2 on the remote PC. Both computers must use the same `ROS_DOMAIN_ID`
and have ROS discovery/network communication enabled between them. The RViz
node receives an annotated JPEG preview from `preview_node` on the Pi; raw color
and depth stay on the tracker computer by default. Tracking keeps running if
the remote RViz node disconnects. Start the camera and worker separately;
neither requires the RViz node to be running.

Settings live in node parameter defaults, with no YAML configuration file.
Tracker parameters are startup-only: `hsv_lower`, `hsv_upper`, `min_area`,
`max_area_ratio`, `min_depth_m`, `max_depth_m`, `min_valid_depth_ratio`, and
`stall_timeout`. Override them when running the tracker directly, for example:

```bash
ros2 run xmen_tracker tracker_node --ros-args -p max_depth_m:=2.0
```

HSV defaults come from `xmen_vision2/config/detector.yaml`. Area is measured at
the input resolution. Candidates need valid depth; absent, invalid, or
out-of-range depth rejects the candidate. There is no color-only fallback.
Physical-size filtering requires camera intrinsics, which `xmen_vision` does
not currently publish, so physical-size parameters are omitted.

The largest valid color contour is selected each frame, with image-center
proximity breaking equal-area ties. This does not maintain object identity
between multiple blue objects. Tune HSV and depth bounds for your target.

Local tracking input subscriptions use reliable, volatile, keep-last QoS with depth 5
and an exact-time synchronization queue of 5 pairs:

- `camera/color/image_raw`: RGB color (`rgb8`).
- `camera/aligned_depth_to_color/image_raw`: depth (`32FC1`, meters; missing
  depth is `NaN`).

Frames are paired by exact timestamp. `latest_frame` holds `(header, rgb, depth)`.
Output uses locally defined QoS (best effort, volatile, keep-last, depth 1):

- `point.x`: normalized horizontal error, right positive.
- `point.y`: normalized vertical error, down positive.
- `point.z`: contour area / image area, **not distance**; zero means no target.

`tracker_node` also publishes `geometry_msgs/Twist` on `/cmd_vel` at 20 Hz
using reliable, volatile, keep-last depth 1 QoS, matching `motor_driver_node`:

- `angular.z`: pan angular velocity, rad/s; right-side targets give negative values.
- `angular.y`: tilt angular velocity, rad/s; targets below center give positive values.
- All other components stay zero. These are velocities, not absolute joint angles.

The calculation is `clamp(sign * gain * normalized_error, -limit, limit)`,
with zero output inside the deadband. Startup parameters are
`kp=1.5`, `cmd_sign=-1.0`, `deadband=0.05`, `max_speed=0.9`,
`kp_tilt=1.2`, `cmd_sign_tilt=1.0`, `deadband_tilt=0.05`,
`max_speed_tilt=0.6`, `tilt_enabled=true`, `rate_hz=20.0`, `tracking_hz=20.0`, `timeout=0.5`,
and `max_input_age=0.5`. Gains convert normalized error to rad/s.

No target, invalid results, stale images, or missing input produce zero commands.
Target loss sends a stop immediately; the timer continues publishing stops.
Shutdown sends a stop while the ROS context is available. The motor driver's
watchdog handles process termination or communication loss. Run only one
`/cmd_vel` command source; do not also run the old tracking controller.
No motor-driver or shared-interface Python dependency is added to this package.

Each accepted pair produces one target with the original timestamp. Duplicate
or older pairs are ignored. Missing targets publish zeros; interrupted images
produce no target updates. The command timer stops motion when input expires.
`/perception_status` reports `OK`, `NO_TARGET`, or `CAMERA_STALL` on changes and
at 1 Hz.

The worker publishes `/tracking/bbox` as `geometry_msgs/PolygonStamped` with
reliable, volatile, keep-last depth 10 QoS. Its original image header identifies
the detection frame. Two pixel-coordinate points encode inclusive top-left and
bottom-right corners (z=0); an empty polygon means no target. The Pi preview node
pairs these boxes with color images by exact timestamp using a bounded queue
of 10. A separate 5 Hz timer encodes only the latest pair as JPEG (quality 70),
with the box already drawn, on `/tracking/preview/compressed`. It never reruns
detection or draws old boxes over newer images. The PC RViz node defaults to
this compressed transport; `preview_transport:=raw` restores direct RGB/box
synchronization for legacy setups. Debug
markers subscribe to `/target` with best-effort QoS independently of images.

With `rviz_node` running, to see a bounding box on the actual camera image in RViz2, add an **Image**
display, select `/tracking/image`, and use **Reliable** reliability. No TF or
Fixed Frame setting is needed for this Image display. The green rectangle
encloses the selected blue target that passes the depth checks. Frames without
a valid target are shown without a rectangle. The image retains the original
camera header and decoded RGB encoding, and is published when a subscriber is connected.
If the camera stops, no new image arrives and RViz may retain the last image.

To show the annotated camera image inside RViz's grid/3D view:

1. Set **Fixed Frame** to `tracking_image`.
2. Add a **PointCloud2** display with topic `/tracking/image_plane`.
3. Set **Color Transformer** to `RGB8`, **Style** to `Squares`, and
   **Size (m)** to `0.004` for 640-pixel-wide images.
4. Set **Decay Time** to `0` and **Reliability** to `Reliable`.
5. Select **Orbit** view. For a straight-on view, set **Focal Point** to
   `(1, 0, 0)`, **Yaw** to `3.14159`, **Pitch** to `0`, and **Distance** to `3`.

This is a flat colored image plane, not a depth-reconstructed point cloud.
It stands upright at X=1, with Z up and image-right along negative Y, like a
screen in front of the origin. It is two display units wide, preserves the camera aspect ratio, and includes
exactly the same bounding-box overlay as `/tracking/image`. Each pixel is one
colored point; a square size of approximately `2 / image_width` fills the plane.
The remote RViz node creates the cloud only while a subscriber is connected.
If the camera stops, RViz may retain the last image plane.

Optionally add a **Marker** display on `/target_marker` for the blue target box.
Its vertical position is adjusted to the image aspect ratio. The box is deleted
on target loss and expires after `marker_lifetime` seconds (default 0.5).
The RViz node publishes a static `tracking_view` → `tracking_image` transform
so the frame appears in the dropdown. This is a visualization-only frame tree;
its coordinates are not measured positions in the robot's physical workspace.

Detection runs on a separate `tracking_hz` timer (20 Hz by default). Incoming
synchronized pairs replace a single pending pair; superseded pairs are dropped
before conversion and detection. Each pair is processed at most once. Camera
publication remains 30 Hz and the independent command timer remains 20 Hz.
