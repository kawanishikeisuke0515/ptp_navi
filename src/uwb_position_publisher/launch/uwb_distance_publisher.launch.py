from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory('uwb_position_publisher'))
    distance_config_path = package_share / 'config' / 'uwb_distance_publisher.yaml'
    position_config_path = package_share / 'config' / 'uwb_position_publisher.yaml'

    return LaunchDescription([
        Node(
            package='uwb_position_publisher',
            executable='uwb_distance_publisher',
            name='uwb_distance_publisher',
            output='screen',
            parameters=[str(distance_config_path)],
        ),
        Node(
            package='uwb_position_publisher',
            executable='uwb_position_publisher',
            name='uwb_position_publisher',
            output='screen',
            parameters=[str(position_config_path)],
        ),
    ])
