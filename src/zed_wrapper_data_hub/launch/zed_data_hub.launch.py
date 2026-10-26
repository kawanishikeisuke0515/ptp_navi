from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_model = LaunchConfiguration('camera_model')
    camera_name = LaunchConfiguration('camera_name')
    namespace = LaunchConfiguration('namespace')
    node_name = LaunchConfiguration('node_name')
    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')
    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    enable_ipc = LaunchConfiguration('enable_ipc')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    ros_params_override_path = LaunchConfiguration('ros_params_override_path')

    default_override = PathJoinSubstitution([
        FindPackageShare('zed_wrapper_data_hub'),
        'config',
        'zed2i_data_hub.yaml',
    ])

    zed_camera_launch = PathJoinSubstitution([
        FindPackageShare('zed_wrapper'),
        'launch',
        'zed_camera.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='ZED camera model passed to zed_wrapper.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed2i',
            description='Camera name and default namespace for zed_wrapper.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional namespace passed to zed_wrapper.',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='zed_node',
            description='ZED wrapper node name.',
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='0',
            description='ZED camera serial number. 0 means wrapper default.',
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='-1',
            description='ZED camera device ID. -1 means wrapper default.',
        ),
        DeclareLaunchArgument(
            'publish_urdf',
            default_value='true',
            description='Start robot_state_publisher with zed_description URDF.',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish odom to camera_link TF from zed_wrapper.',
        ),
        DeclareLaunchArgument(
            'publish_map_tf',
            default_value='true',
            description='Publish map to odom TF from zed_wrapper.',
        ),
        DeclareLaunchArgument(
            'publish_imu_tf',
            default_value='false',
            description='Publish IMU TF from zed_wrapper.',
        ),
        DeclareLaunchArgument(
            'enable_ipc',
            default_value='true',
            description='Enable zed_wrapper intra-process communication.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Start zed_wrapper in simulation mode.',
        ),
        DeclareLaunchArgument(
            'ros_params_override_path',
            default_value=default_override,
            description='Override YAML passed to zed_wrapper.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_camera_launch),
            launch_arguments={
                'camera_model': camera_model,
                'camera_name': camera_name,
                'namespace': namespace,
                'node_name': node_name,
                'serial_number': serial_number,
                'camera_id': camera_id,
                'publish_urdf': publish_urdf,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_map_tf,
                'publish_imu_tf': publish_imu_tf,
                'enable_ipc': enable_ipc,
                'use_sim_time': use_sim_time,
                'sim_mode': sim_mode,
                'ros_params_override_path': ros_params_override_path,
            }.items(),
        ),
    ])
