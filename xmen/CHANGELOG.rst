^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xmen
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-10-08)
------------------
First release of the ROS-Vision-Tracker stack for ROS 2 Lyrical on
Ubuntu 26.04 (ARM64). ``xmen`` is a metapackage; it installs
``xmen_tracker``, ``xmen_control``, ``xmen_description`` and ``xmen_bringup``.

Perception (``xmen_tracker``)
* Add RealSense capture publishing color and aligned depth, with a direct
  pipeline and an RGBD-subscription source selectable at launch.
* Add ``cube_detector``: HSV color segmentation plus real-world size and shape
  checks against the depth image, so a blue background or a partially visible
  cube no longer passes as the target.
* Bridge brief occlusions with a CSRT tracker and publish ``/tracking/bbox``.
* Add the annotated JPEG preview and RViz2 marker and point cloud overlays.
* Tune HSV, depth and focal-length values against measurements taken in the
  lab, including brightness correction for the real room.
* Fix QoS settings so the published image and marker topics are compatible
  with RViz2.
* Fix rosbag timestamp synchronization for color and depth replay.

Control (``xmen_control``)
* Add the OpenCR serial driver: angular velocity commands out (``V``), current
  angle in (``P``), ``/joint_states`` published, and travel limited per axis.
* Apply pan and tilt commands simultaneously rather than one axis at a time.
* Replace the original driver with the compact ``control_lite`` version.
* Standardize the gimbal velocity command topic as ``/cmd_vel``.
* Raise the motor time-delay limit and lower the default tracking speed to
  stop the overshoot seen on the real rig.

Description (``xmen_description``)
* Add the pan/tilt URDF, CAD meshes and the RViz2 configuration.

Bringup (``xmen_bringup``)
* Add the full hardware launch, the remote visualization launch and
  ``tracking.yaml``.
* Add ``robot_state_publisher``, ``joint_state_publisher`` and rosbag record
  and replay wiring.
* Default launch targets 30 Hz image publication, 20 Hz tracking, 20 Hz
  commands and a 5 Hz preview.

Firmware
* Add ``opencr_lite.ino`` and a prebuilt ARM64 flashing bundle
  (``firmware/opencr.tar.gz``) that needs no Arduino toolchain on the Pi.

Documentation and tooling
* Add the project README, per-package READMEs, parameter reference,
  RealSense setup notes and the problem reports with measured plots.
* Add the GitHub Actions workflow that builds the Raspberry Pi release
  tarball.

* Contributors: HaeminKim, HongjuLee, SeokMin Hong, chj1319, hsmint
