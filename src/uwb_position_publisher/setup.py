from glob import glob

from setuptools import find_packages, setup

package_name = 'uwb_position_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/doc', glob('doc/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='UWB serial distance reader and ROS 2 distance publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_distance_publisher = uwb_position_publisher.uwb_distance_publisher:main',
            'uwb_position_publisher = uwb_position_publisher.uwb_position_publisher:main',
            'uwb_position_logger = '
            'uwb_position_publisher.uwb_position_logger:main',
            'uwb_optitrack_logger = uwb_position_publisher.uwb_optitrack_logger:main',
        ],
    },
)
