from glob import glob

from setuptools import find_packages, setup

package_name = 'zed_heading_publisher'

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
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ZED2i magnetometer topic subscriber and ROS 2 heading publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_calibrate_zed_heading = zed_heading_publisher.auto_calibrate_zed_heading:main',
            'zed_heading_publisher = zed_heading_publisher.zed_heading_publisher:main',
            'calibrate_zed_heading = zed_heading_publisher.calibrate_zed_heading:main',
            'compare_vio_heading = zed_heading_publisher.compare_vio_heading:main',
        ],
    },
)
