"""Start ZED, wheel control, and an automatic compass calibration turn."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        'start_zed': 'true',
        'start_wheels': 'true',
        'command_topic': '/rov_cmd_vel',
        'mag_topic': '/zed2i/zed_node/imu/mag',
        'odom_topic': '/zed2i/zed_node/odom',
        'angular_speed_rad_sec': '0.15',
        'startup_timeout_sec': '60.0',
        'sensor_timeout_sec': '1.0',
        'max_duration_sec': '90.0',
        'target_rotation_deg': '360.0',
        'output_config_path': '',
        'sample_csv_path': '',
    }
    arguments = [DeclareLaunchArgument(key, default_value=value)
                 for key, value in defaults.items()]
    arguments.append(DeclareLaunchArgument(
        'wheel_config', default_value=PathJoinSubstitution([
            FindPackageShare('locomotion_core_rpm'), 'config', 'locomotion_core_rpm.yaml'])))
    includes = []
    for package, filename, condition, launch_arguments in (
        ('zed_wrapper_data_hub', 'zed_data_hub.launch.py', 'start_zed', {}),
        ('locomotion_core_rpm', 'wheel_control.launch.py', 'start_wheels',
         {'config': LaunchConfiguration('wheel_config')}),
    ):
        includes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), 'launch', filename])),
            condition=IfCondition(LaunchConfiguration(condition)),
            launch_arguments=launch_arguments.items()))
    string_parameters = {'command_topic', 'mag_topic', 'odom_topic',
                         'output_config_path', 'sample_csv_path'}
    parameters = {
        key: ParameterValue(LaunchConfiguration(key),
                            value_type=str if key in string_parameters else float)
        for key in defaults if key not in {'start_zed', 'start_wheels'}
    }
    calibrator = Node(
        package='zed_heading_publisher', executable='auto_calibrate_zed_heading',
        output='screen', parameters=[parameters])
    return LaunchDescription(arguments + [
        RegisterEventHandler(OnProcessExit(
            target_action=calibrator,
            on_exit=[EmitEvent(event=Shutdown(reason='Calibration process exited'))])),
        *includes,
        calibrator,
    ])
