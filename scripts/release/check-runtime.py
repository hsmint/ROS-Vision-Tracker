"""Check installed ROS/Python imports without starting nodes or hardware."""

from importlib import import_module


def main():
    modules = (
        "rclpy",
        "cv2",
        "numpy",
        "yaml",
        "serial",
        "xmen_tracker.detector",
        "xmen_tracker.cube_tracker",
        "xmen_tracker.tracker_node",
        "xmen_tracker.preview_node",
        "xmen_tracker.rviz_node",
        "xmen_control.control",
        "xmen_control.control_lite",
    )
    for module in modules:
        import_module(module)
        print(f"OK: {module}")
    print("Runtime import check passed. RealSense SDK and hardware require a separate check.")


if __name__ == "__main__":
    main()
