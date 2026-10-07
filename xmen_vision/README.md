# xmen_vision

Publishes synchronized RealSense RGB (`rgb8`) and aligned metric depth
(`32FC1`, meters; invalid samples are NaN), plus color camera calibration:

- `/camera/color/image_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/color/camera_info`

```bash
ros2 run xmen_vision realsense_node
```

Target perception, tuning, and evaluation have moved into
[xmen_tracker](../xmen_tracker/README.md). Run `xmen_tracker/tracker_node`
for detection and pan/tilt tracking. See [REALSENSE.md](REALSENSE.md) for camera notes.
