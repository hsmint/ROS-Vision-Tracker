"""인지: target_detector가 카메라 영상을 검출해 → /target, /perception_status.

  ros2 launch target_perception perception.launch.py                 # 방식 A: pyrealsense2로 카메라 직접 (PC)
  ros2 launch target_perception perception.launch.py source:=ros     # 방식 B: realsense2_camera + detector (RPi)
  ros2 launch target_perception perception.launch.py show:=true      # 검출 화면(오버레이·mask) — PC
  ros2 launch target_perception perception.launch.py config:=<경로>  # 다른 검출 설정

source
  realsense (기본)  detector가 카메라를 직접 연다. 영상 토픽 없음
  ros               camera.launch.py(realsense2_camera, viewer 없음)를 함께 띄우고 detector가 /camera/camera/rgbd를 구독
카메라는 한 프로세스만 연다 — 방식 A 실행 중에는 camera.launch.py·problem1/run.py를 따로 띄우지 않는다.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arg = LaunchConfiguration
    share = Path(get_package_share_directory('target_perception'))
    as_bool = lambda name: PythonExpression(["'", arg(name), "'.lower() == 'true'"])
    use_ros = PythonExpression(["'", arg('source'), "' == 'ros'"])
    # 방식 B: realsense2_camera만 띄운다(camera_viewer는 끔 — detector가 구독)
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'camera.launch.py')),
        launch_arguments={'viewer': 'false', 'initial_reset': arg('initial_reset')}.items(),
        condition=IfCondition(use_ros))
    detector = Node(package='target_perception', executable='detector', name='target_detector', output='screen',
                    parameters=[{'config': arg('config'), 'source': arg('source'), 'show': as_bool('show'),
                                 'initial_reset': as_bool('initial_reset')}])
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='realsense', choices=['realsense', 'ros'],
                              description='realsense = 카메라 직접(방식 A), ros = realsense2_camera 구독(방식 B)'),
        DeclareLaunchArgument('config', default_value=str(share / 'config' / 'detector.yaml'),
                              description='검출 설정(HSV·면적·해상도·뎁스 검증·노출)'),
        DeclareLaunchArgument('show', default_value='false', description='검출 화면 표시(PC)'),
        DeclareLaunchArgument('initial_reset', default_value='true', description='시작 시 카메라 하드웨어 리셋'),
        camera,
        detector,
    ])
