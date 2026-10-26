from glob import glob
from setuptools import setup

package_name = 'locomotion_core_rpm'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/docs', glob('docs/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='keisuke-kawanishi', maintainer_email='keisuke-kawanishi@users.noreply.github.com',
    description='Twist to motor RPM and Roboteq serial speed commands',
    license='TODO: License declaration',
    entry_points={'console_scripts': [
        'rover_velocity = locomotion_core_rpm.rover_velocity:main',
        'cmd_roboteq = locomotion_core_rpm.cmd_roboteq:main',
    ]},
)
