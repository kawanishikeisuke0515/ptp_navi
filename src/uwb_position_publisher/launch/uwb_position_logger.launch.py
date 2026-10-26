from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('uwb_position_publisher'))
    config_path = package_share / 'config' / 'uwb_position_logger.yaml'

    return LaunchDescription([
        Node(
            package='uwb_position_publisher',
            executable='uwb_position_logger',
            name='uwb_position_logger',
            output='screen',
            parameters=[str(config_path)],
        ),
    ])
