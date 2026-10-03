"""모터 출력 OFF로 모의 입력 시험(팬·틸트·미검출·침묵·정지 영상·오래된 시각·복귀). input_test가 끝나면 전체 종료.
tilt:=false → 컨트롤러를 팬 1축으로 두고 같은 입력을 시험한다."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = str(Path(get_package_share_directory('target_bringup')) / 'config' / 'tracking.yaml')
    tilt = PythonExpression(["'", LaunchConfiguration('tilt'), "'.lower() == 'true'"])
    tag = PythonExpression(["'inputs' if '", LaunchConfiguration('tilt'), "'.lower() == 'true' else 'inputs_pan_only'"])
    test = Node(package='target_bringup', executable='input_test', name='input_test', output='screen',
                parameters=[{'tilt': tilt, 'tag': tag}])
    return LaunchDescription([
        DeclareLaunchArgument('tilt', default_value='true', description='틸트 축 사용(false = 팬 1축)'),
        Node(package='target_control', executable='controller', name='tracking_controller',
             parameters=[params, {'tilt_enabled': tilt}], output='screen'),
        Node(package='target_control', executable='motor_driver', name='motor_driver',
             parameters=[params], output='screen'),
        test,
        RegisterEventHandler(OnProcessExit(target_action=test, on_exit=[EmitEvent(event=Shutdown())])),
    ])
