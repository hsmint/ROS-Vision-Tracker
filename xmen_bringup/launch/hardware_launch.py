"""Run tracking with either the local RealSense or recorded RGB/depth."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context):
    use_bag = IfCondition(LaunchConfiguration('use_bag')).evaluate(context)
    start_control = IfCondition(LaunchConfiguration('start_control')).evaluate(context)
    bag_path = LaunchConfiguration('bag_path').perform(context).strip()
    share = Path(get_package_share_directory('xmen_bringup'))
    if bag_path and not use_bag:
        raise ValueError('bag_path requires use_bag:=true')
    if bag_path and not Path(bag_path).expanduser().exists():
        raise ValueError(f'Bag path does not exist: {bag_path}')

    actions = [Node(
        package='xmen_tracker', executable='tracker_node', output='screen',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_bag}],
    )]
    if not use_bag:
        if start_control:
            actions.append(Node(
                package='xmen_control', executable='control', output='screen',
                parameters=[LaunchConfiguration('control_params_file'), {
                    'use_sim_time': False,
                    'auto_home': ParameterValue(LaunchConfiguration('auto_home'), value_type=bool),
                    'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                }],
            ))
        actions.append(Node(
            package='xmen_vision', executable='realsense_node', output='screen',
            parameters=[{'use_sim_time': False}],
        ))
    elif bag_path:
        # Replay only sensor inputs: old commands/detections must not compete
        # with the tracker. Reliable QoS matches its image subscriptions.
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'play', str(Path(bag_path).expanduser()),
                 '--clock', '100', '--delay', '2',
                 '--qos-profile-overrides-path', str(share / 'param/bag_qos.yaml'),
                 '--topics', '/camera/color/image_raw',
                 '/camera/aligned_depth_to_color/image_raw'],
            output='screen',
        ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('xmen_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('use_bag', default_value='false', choices=['true', 'false'],
                              description='Use bag image topics and simulated time; disable camera.'),
        DeclareLaunchArgument('bag_path', default_value='',
                              description='Bag to play, or empty for externally started playback.'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'param/tracker.yaml'),
                              description='Tracker ROS parameter YAML.'),
        DeclareLaunchArgument('start_control', default_value='true', choices=['true', 'false'],
                              description='Start xmen_control/control in live mode.'),
        DeclareLaunchArgument('auto_home', default_value='true', choices=['true', 'false'],
                              description='Home on control startup and save firmware automatic homing setting.'),
        DeclareLaunchArgument('control_params_file', default_value=str(share / 'param/control.yaml')),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        OpaqueFunction(function=_setup),
    ])
