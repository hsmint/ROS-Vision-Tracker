"""Run the tracker only; start realsense_node (`ros2 run xmen_tracker realsense_node`) separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory('xmen_tracker')) / 'config/detector.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('detector_config', default_value=str(config)),
        Node(package='xmen_tracker', executable='tracker_node', output='screen',
             parameters=[{'detector_config': LaunchConfiguration('detector_config')}]),
    ])
