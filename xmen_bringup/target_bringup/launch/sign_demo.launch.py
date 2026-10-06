"""부호 시험: gimbal_sim 폐루프(팬·틸트).
cmd_sign:=-1.0 cmd_sign_tilt:=1.0 (정상) / 한 축이라도 뒤집으면 그 축의 오차가 커지다 화면 밖으로 사라진다."""
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
    sign, sign_t = LaunchConfiguration('cmd_sign'), LaunchConfiguration('cmd_sign_tilt')
    tag = PythonExpression(["'sign_' + ('normal' if float('", sign, "') < 0 else 'pan_reversed') + "
                            "('' if float('", sign_t, "') > 0 else '_tilt_reversed')"])
    sim = Node(package='target_bringup', executable='gimbal_sim', name='gimbal_sim', output='screen',
               parameters=[{'tag': tag}])
    return LaunchDescription([
        DeclareLaunchArgument('cmd_sign', default_value='-1.0', description='팬 부호(정상 -1.0)'),
        DeclareLaunchArgument('cmd_sign_tilt', default_value='1.0', description='틸트 부호(정상 1.0)'),
        Node(package='target_control', executable='controller', name='tracking_controller',
             parameters=[params, {'cmd_sign': PythonExpression(['float(', sign, ')']),
                                  'cmd_sign_tilt': PythonExpression(['float(', sign_t, ')'])}],
             output='screen'),
        sim,
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(event=Shutdown())])),
    ])
