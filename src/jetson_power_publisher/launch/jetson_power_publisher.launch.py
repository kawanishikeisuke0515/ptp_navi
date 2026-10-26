from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('jetson_power_publisher'), 'config',
            'jetson_power_publisher.yaml'])),
        Node(package='jetson_power_publisher', executable='jetson_power_publisher',
             name='jetson_power_publisher', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
