from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('zed_heading_publisher'))
    config_path = package_share / 'config' / 'zed_heading_publisher.yaml'
    mag_topic = LaunchConfiguration('mag_topic')
    raw_x_axis = LaunchConfiguration('raw_x_axis')
    raw_x_sign = LaunchConfiguration('raw_x_sign')
    raw_z_axis = LaunchConfiguration('raw_z_axis')
    raw_z_sign = LaunchConfiguration('raw_z_sign')
    magnetic_field_scale = LaunchConfiguration('magnetic_field_scale')
    diagnostic_log_interval_sec = LaunchConfiguration('diagnostic_log_interval_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mag_topic',
            default_value='/zed2i/zed_node/imu/mag',
            description='MagneticField topic published by the ZED data hub.',
        ),
        DeclareLaunchArgument(
            'raw_x_axis',
            default_value='y',
            description='MagneticField axis used as legacy right-axis raw_x.',
        ),
        DeclareLaunchArgument(
            'raw_x_sign',
            default_value='-1.0',
            description='Sign applied to raw_x_axis.',
        ),
        DeclareLaunchArgument(
            'raw_z_axis',
            default_value='x',
            description='MagneticField axis used as legacy forward-axis raw_z.',
        ),
        DeclareLaunchArgument(
            'raw_z_sign',
            default_value='1.0',
            description='Sign applied to raw_z_axis.',
        ),
        DeclareLaunchArgument(
            'magnetic_field_scale',
            default_value='1000000.0',
            description='Scale applied to MagneticField values before heading calculation.',
        ),
        DeclareLaunchArgument(
            'diagnostic_log_interval_sec',
            default_value='1.0',
            description='Interval for heading diagnostic logs; 0 disables logs.',
        ),
        Node(
            package='zed_heading_publisher',
            executable='zed_heading_publisher',
            name='zed_heading_publisher',
            output='screen',
            parameters=[
                str(config_path),
                {
                    'mag_topic': mag_topic,
                    'raw_x_axis': ParameterValue(raw_x_axis, value_type=str),
                    'raw_x_sign': raw_x_sign,
                    'raw_z_axis': ParameterValue(raw_z_axis, value_type=str),
                    'raw_z_sign': raw_z_sign,
                    'magnetic_field_scale': magnetic_field_scale,
                    'diagnostic_log_interval_sec': diagnostic_log_interval_sec,
                },
            ],
        ),
    ])
