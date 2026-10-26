"""Start sensors, pose estimation, wheel control and autonomous path tracking."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def share_file(package, folder, filename):
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def include(package, filename, arguments=None, condition=None):
    # Child launch arguments must not overwrite another child's defaults.
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share_file(package, 'launch', filename)),
        launch_arguments=(arguments or {}).items(),
    )], condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=share_file(
            'path_control', 'config', 'path_control.yaml')),
        DeclareLaunchArgument('start_sensors', default_value='true',
                              description='Start UWB, ZED, heading and robot pose nodes.'),
        DeclareLaunchArgument('start_locomotion', default_value='true',
                              description='Start RPM conversion and Roboteq motor driver.'),
        DeclareLaunchArgument('start_power', default_value='true',
                              description='Start Jetson power telemetry.'),
        DeclareLaunchArgument('locomotion_config', default_value=share_file(
            'locomotion_core_rpm', 'config', 'locomotion_core_rpm.yaml')),
        DeclareLaunchArgument('robot_pose_config', default_value=share_file(
            'uwb_robot_pose_publisher', 'config', 'uwb_robot_pose_publisher.yaml')),
        include('zed_wrapper_data_hub', 'zed_data_hub.launch.py',
                condition=IfCondition(LaunchConfiguration('start_sensors'))),
        include('zed_heading_publisher', 'zed_heading_publisher.launch.py',
                condition=IfCondition(LaunchConfiguration('start_sensors'))),
        include('uwb_position_publisher', 'uwb_distance_publisher.launch.py',
                condition=IfCondition(LaunchConfiguration('start_sensors'))),
        include('uwb_robot_pose_publisher', 'uwb_robot_pose_publisher.launch.py',
                {'config_file': LaunchConfiguration('robot_pose_config')},
                IfCondition(LaunchConfiguration('start_sensors'))),
        include('locomotion_core_rpm', 'wheel_control.launch.py',
                {'config': LaunchConfiguration('locomotion_config')},
                IfCondition(LaunchConfiguration('start_locomotion'))),
        include('jetson_power_publisher', 'jetson_power_publisher.launch.py',
                condition=IfCondition(LaunchConfiguration('start_power'))),
        Node(package='path_control', executable='path_control', name='path_control',
             parameters=[LaunchConfiguration('config_file')], output='screen'),
    ])
