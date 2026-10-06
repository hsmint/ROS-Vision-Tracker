"""Visualize tracking from the Pi (or a bag-driven tracker)."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _robot(context):
    description = Path(LaunchConfiguration('urdf_file').perform(context)).expanduser().read_text()
    robot = ET.fromstring(description)
    children = {joint.find('child').attrib['link'] for joint in robot.findall('joint')}
    roots = [link.attrib['name'] for link in robot.findall('link') if link.attrib['name'] not in children]
    if len(roots) != 1:
        raise ValueError('URDF must have exactly one root link')
    if robot.find("link[@name='camera_link']") is None:
        raise ValueError('URDF must define camera_link at the camera mount')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_bag'), value_type=bool)}
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[clock, {'robot_description': ParameterValue(description, value_type=str)}],
             output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[clock], condition=IfCondition(LaunchConfiguration('publish_joint_states')),
             output='screen'),
        Node(package='xmen_tracker', executable='rviz_node',
             parameters=[clock, {
                 'pose_frame': roots[0],
                 'display_width': ParameterValue(LaunchConfiguration('display_width'), value_type=float),
                 'display_distance': ParameterValue(LaunchConfiguration('display_distance'), value_type=float),
             }], output='screen'),
        # Attach the preview to the moving camera, not the stationary robot root.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='robot_tracking_view_tf', parameters=[clock],
             arguments=['--frame-id', 'camera_link', '--child-frame-id', 'tracking_view',
                        '--x', LaunchConfiguration('display_x'),
                        '--y', LaunchConfiguration('display_y'),
                        '--z', LaunchConfiguration('display_z'),
                        '--roll', LaunchConfiguration('display_roll'),
                        '--pitch', LaunchConfiguration('display_pitch'),
                        '--yaw', LaunchConfiguration('display_yaw')], output='screen'),
    ]


def generate_launch_description():
    description_share = Path(get_package_share_directory('xmen_description'))
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_bag'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_bag', default_value='false', choices=['true', 'false'],
                              description='Match the tracker playback clock.'),
        DeclareLaunchArgument('start_rviz', default_value='true', choices=['true', 'false'],
                              description='Open RViz2 as well as the overlay node.'),
        DeclareLaunchArgument('rviz_config', default_value=str(description_share / 'rviz/tracking.rviz')),
        DeclareLaunchArgument('urdf_file', default_value=str(description_share / 'urdf/cctv.urdf')),
        DeclareLaunchArgument('publish_joint_states', default_value='true', choices=['true', 'false'],
                              description='Publish preview joint positions; disable for real joint feedback.'),
        DeclareLaunchArgument('display_width', default_value='0.4', description='Flat image width in display units.'),
        DeclareLaunchArgument('display_distance', default_value='0.25', description='Image plane X within its display frame.'),
        DeclareLaunchArgument('display_x', default_value='0.0'),
        DeclareLaunchArgument('display_y', default_value='0.0'),
        DeclareLaunchArgument('display_z', default_value='0.0'),
        DeclareLaunchArgument('display_roll', default_value='0.0'),
        DeclareLaunchArgument('display_pitch', default_value='0.0'),
        DeclareLaunchArgument('display_yaw', default_value='0.0'),
        OpaqueFunction(function=_robot),
        Node(package='rviz2', executable='rviz2', parameters=[clock],
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('start_rviz')), output='screen'),
    ])
