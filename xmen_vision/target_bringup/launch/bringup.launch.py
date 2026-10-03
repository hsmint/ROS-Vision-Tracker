"""전체 실행: 인지(카메라 + target_detector) → tracking_controller → motor_driver(출력 OFF).

  ros2 launch target_bringup bringup.launch.py                  # RPi 기본
  ros2 launch target_bringup bringup.launch.py show:=true       # + 검출 화면 (PC)

  [D435] → target_detector(pyrealsense2 직접) → /target → tracking_controller → /gimbal/cmd_vel → motor_driver
                         └ /perception_status                  └ /tracking_status
노드 파라미터: target_bringup/config/tracking.yaml (게인·부호·데드밴드·주기·타임아웃·출력 ON/OFF)
검출 설정:     target_perception/config/detector.yaml
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arg = LaunchConfiguration
    params = str(Path(get_package_share_directory('target_bringup')) / 'config' / 'tracking.yaml')
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('target_perception')) / 'launch' / 'perception.launch.py')),
        launch_arguments={'show': arg('show')}.items())
    return LaunchDescription([
        DeclareLaunchArgument('show', default_value='false', description='검출 화면 표시(PC)'),
        perception,
        Node(package='target_control', executable='controller', name='tracking_controller',
             parameters=[params], output='screen'),
        Node(package='target_control', executable='motor_driver', name='motor_driver',
             parameters=[params], output='screen'),
    ])
