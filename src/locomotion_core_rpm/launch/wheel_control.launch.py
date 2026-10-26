from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('locomotion_core_rpm'), 'config', 'locomotion_core_rpm.yaml'])),
        Node(package='locomotion_core_rpm', executable='rover_velocity', output='screen', parameters=[config]),
        Node(package='locomotion_core_rpm', executable='cmd_roboteq', output='screen', parameters=[config]),
    ])
