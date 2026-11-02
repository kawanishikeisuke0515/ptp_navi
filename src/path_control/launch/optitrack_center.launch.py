"""Convert externally published OptiTrack poses to robot center positions."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('path_control'), 'config', 'optitrack_center.yaml'])),
        Node(package='path_control', executable='optitrack_center',
             name='optitrack_center', parameters=[LaunchConfiguration('config_file')],
             output='screen'),
    ])
