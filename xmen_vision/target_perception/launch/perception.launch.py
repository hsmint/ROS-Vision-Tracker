"""인지: target_detector가 pyrealsense2로 카메라를 직접 열어 검출 → /target, /perception_status.

  ros2 launch target_perception perception.launch.py                 # RPi 기본 (화면 없음)
  ros2 launch target_perception perception.launch.py show:=true      # 검출 화면(오버레이·mask) — PC
  ros2 launch target_perception perception.launch.py config:=<경로>  # 다른 검출 설정

카메라를 직접 열므로 realsense2_camera(camera.launch.py)·problem1/run.py와 동시에 실행하지 않는다.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arg = LaunchConfiguration
    share = Path(get_package_share_directory('target_perception'))
    as_bool = lambda name: PythonExpression(["'", arg(name), "'.lower() == 'true'"])
    detector = Node(package='target_perception', executable='detector', name='target_detector', output='screen',
                    parameters=[{'config': arg('config'), 'show': as_bool('show'),
                                 'initial_reset': as_bool('initial_reset')}])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config' / 'detector.yaml'),
                              description='검출 설정(HSV·면적·해상도·뎁스 검증·노출)'),
        DeclareLaunchArgument('show', default_value='false', description='검출 화면 표시(PC)'),
        DeclareLaunchArgument('initial_reset', default_value='true', description='시작 시 카메라 하드웨어 리셋'),
        detector,
    ])
