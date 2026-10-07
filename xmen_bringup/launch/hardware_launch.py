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
    actions.append(Node(
        package='xmen_tracker', executable='preview_node', output='screen',
        condition=IfCondition(LaunchConfiguration('start_preview')),
        parameters=[{'use_sim_time': use_bag,
                     'preview_hz': ParameterValue(LaunchConfiguration('preview_hz'), value_type=float),
                     'jpeg_quality': ParameterValue(LaunchConfiguration('jpeg_quality'), value_type=int)}],
    ))
    if not use_bag:
        if start_control:
            actions.append(Node(
                package='xmen_control', executable='control_lite', output='screen',
                parameters=[LaunchConfiguration('control_params_file'), {
                    'use_sim_time': False,
                    'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                }],
            ))
        actions.append(Node(
            package='xmen_vision', executable='realsense_node', output='screen',
            parameters=[{'use_sim_time': False,
                         'publish_hz': ParameterValue(
                             LaunchConfiguration('image_hz'), value_type=float)}],
        ))
    elif bag_path:
        # Replay only sensor inputs: old commands/detections must not compete
        # with the tracker. Reliable QoS matches its image subscriptions.
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'play', str(Path(bag_path).expanduser()),
                 '--clock', '100', '--delay', '2',
                 '--qos-profile-overrides-path', str(share / 'param/bag_qos.yaml'),
                 '--topics', '/camera/color/image_raw',
                 '/camera/aligned_depth_to_color/image_raw', '/camera/color/camera_info'],
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
                              description='Start xmen_control/control_lite in live mode (homes once at startup).'),
        DeclareLaunchArgument('control_params_file', default_value=str(share / 'param/control.yaml')),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('image_hz', default_value='30.0',
                              description='Image alignment/publication rate, up to 30 Hz.'),
        DeclareLaunchArgument('start_preview', default_value='true', choices=['true', 'false'],
                              description='Publish a separate annotated JPEG preview.'),
        DeclareLaunchArgument('preview_hz', default_value='5.0',
                              description='Remote preview rate; independent of image_hz and tracking.'),
        DeclareLaunchArgument('jpeg_quality', default_value='70',
                              description='JPEG preview quality, 1 to 100.'),
        OpaqueFunction(function=_setup),
    ])
