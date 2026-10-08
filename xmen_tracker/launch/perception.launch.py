"""Run the tracker only; start realsense_node (`ros2 run xmen_tracker realsense_node`) separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory('xmen_tracker')) / 'config/detector.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('detector_config', default_value=str(config)),
        # bag 재생(ros2 bag play --clock) 시 true: bag 시계로 스탬프 나이를 판단
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('debug_timing', default_value='false'),
        Node(package='xmen_tracker', executable='tracker_node', output='screen',
             parameters=[{
                 'detector_config': LaunchConfiguration('detector_config'),
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
                 'debug_timing': ParameterValue(
                     LaunchConfiguration('debug_timing'), value_type=bool),
             }]),
    ])
