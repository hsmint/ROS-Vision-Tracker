"""/search 액션 시험: 성공·시간 초과·취소·입력 없음·잘못된 요청."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = str(Path(get_package_share_directory('target_bringup')) / 'config' / 'tracking.yaml')
    test = Node(package='target_bringup', executable='search_test', name='search_test', output='screen')
    return LaunchDescription([
        Node(package='target_control', executable='controller', name='tracking_controller',
             parameters=[params], output='screen'),
        Node(package='target_control', executable='motor_driver', name='motor_driver',
             parameters=[params], output='screen'),
        test,
        RegisterEventHandler(OnProcessExit(target_action=test, on_exit=[EmitEvent(event=Shutdown())])),
    ])
