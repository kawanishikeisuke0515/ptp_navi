from glob import glob

from setuptools import find_packages, setup

package_name = 'zed_wrapper_data_hub'

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
        ('share/' + package_name + '/docs', glob('docs/*.md') + glob('docs/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description=(
        'Configuration and launch wrapper for using the official ZED ROS 2 '
        'wrapper as the system data hub'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
)
