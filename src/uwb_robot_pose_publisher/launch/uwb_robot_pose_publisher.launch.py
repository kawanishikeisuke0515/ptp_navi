from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('uwb_robot_pose_publisher'),
                'config',
                'uwb_robot_pose_publisher.yaml',
            ]),
            description='YAML config file for uwb_robot_pose_publisher',
        ),
        Node(
            package='uwb_robot_pose_publisher',
            executable='uwb_robot_pose_publisher',
            name='uwb_robot_pose_publisher',
            output='screen',
            parameters=[config_file],
        ),
    ])
