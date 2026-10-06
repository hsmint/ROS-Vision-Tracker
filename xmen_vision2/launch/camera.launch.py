"""RealSense 카메라 노드(realsense2_camera) + 영상 확인 노드(camera_viewer).

  ros2 launch target_perception camera.launch.py                 # 카메라 + 창
  ros2 launch target_perception camera.launch.py show:=false     # RPi: 로그만
  ros2 launch target_perception camera.launch.py viewer:=false   # 카메라만(다른 노드·bag이 구독)

카메라 설정: 컬러·뎁스 640×360@30, 뎁스를 컬러에 정렬(align_depth), 컬러·뎁스 시각 맞춤(enable_sync),
세 메시지를 한 묶음으로 발행(enable_rgbd → /camera/camera/rgbd).
쓰지 않는 스트림(적외선·IMU·포인트클라우드)은 꺼서 CPU·USB 대역폭을 아낀다.
initial_reset:=true(기본) — 시작 시 카메라를 하드웨어 리셋한다(약 5초). 뽑았다 꽂거나 이전 프로세스가 비정상
종료되면 뎁스 모듈이 멈춰 'Frames didn't arrive within 5 seconds'만 나오는 경우가 있었다(PC 실측, 리셋으로 해결).
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arg = LaunchConfiguration
    rs_launch = str(Path(get_package_share_directory('realsense2_camera')) / 'launch' / 'rs_launch.py')
    # realsense2_camera에 넘길 값만 골라 넘긴다 — 이 파일의 인자(show·viewer·profile)가 넘어가면
    # rs_launch.py가 "Parameter ... is not supported" 경고를 낸다. forwarding=False로 나머지는 막는다
    rs_args = {
        'rgb_camera.color_profile': arg('profile'),
        'depth_module.depth_profile': arg('profile'),
        'align_depth.enable': 'true',
        'enable_sync': 'true',
        'enable_rgbd': 'true',          # 컬러·정렬 뎁스·camera_info를 한 메시지(/camera/camera/rgbd)로
        'enable_infra1': 'false',
        'enable_infra2': 'false',
        'enable_gyro': 'false',
        'enable_accel': 'false',
        'pointcloud.enable': 'false',
        'initial_reset': arg('initial_reset'),
    }
    camera = GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(rs_launch))],
                         scoped=True, forwarding=False, launch_configurations=rs_args)
    viewer = Node(package='target_perception', executable='camera_viewer', name='camera_viewer', output='screen',
                  parameters=[{'show': arg('show')}], condition=IfCondition(arg('viewer')))
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='640,360,30', description='폭,높이,FPS (컬러·뎁스 동일)'),
        DeclareLaunchArgument('viewer', default_value='true', description='camera_viewer 실행'),
        DeclareLaunchArgument('show', default_value='true', description='창 표시(false = 로그만)'),
        DeclareLaunchArgument('initial_reset', default_value='true', description='시작 시 카메라 하드웨어 리셋'),
        camera,
        viewer,
    ])
