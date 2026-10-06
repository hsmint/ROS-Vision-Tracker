"""전체 연결 검사(실제 카메라): target_detector → tracking_controller → motor_driver(출력 OFF)
+ interface_check(타입·QoS·주기·호환성 검사, 토픽 기록). duration 초 뒤 전체 종료.

  ros2 launch target_bringup e2e_check.launch.py                         # 10초, 결과 results/e2e_*
  ros2 launch target_bringup e2e_check.launch.py duration:=30.0 tag:=e2e_30s
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = str(Path(get_package_share_directory('target_bringup')) / 'config' / 'tracking.yaml')
    perception = Path(get_package_share_directory('target_perception'))
    a = {k: LaunchConfiguration(k) for k in ['duration', 'tag', 'config']}
    check = Node(package='target_bringup', executable='interface_check', name='interface_check',
                 output='screen', parameters=[{'duration': a['duration'], 'tag': a['tag']}])
    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='10.0', description='기록 시간 [s] (카메라 리셋 약 3초 포함)'),
        DeclareLaunchArgument('tag', default_value='e2e', description='결과 파일 이름'),
        DeclareLaunchArgument('config', default_value=str(perception / 'config' / 'detector.yaml'),
                              description='검출 설정'),
        Node(package='target_perception', executable='detector', name='target_detector', output='screen',
             parameters=[{'config': a['config']}]),
        Node(package='target_control', executable='controller', name='tracking_controller',
             parameters=[params], output='screen'),
        Node(package='target_control', executable='motor_driver', name='motor_driver',
             parameters=[params], output='screen'),
        check,
        RegisterEventHandler(OnProcessExit(target_action=check, on_exit=[EmitEvent(event=Shutdown())])),
    ])
