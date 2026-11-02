from glob import glob
from setuptools import find_packages, setup

setup(name='path_control', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/path_control']),
                  ('share/path_control', ['package.xml']),
                  ('share/path_control/config', glob('config/*.yaml')),
                  ('share/path_control/launch', glob('launch/*.py')),
                  ('share/path_control/docs', glob('docs/*'))],
      install_requires=['setuptools'], tests_require=['pytest'],
      maintainer='user', maintainer_email='user@example.com', license='Apache-2.0',
      description='Autonomous straight path tracking using UWB robot poses',
      entry_points={'console_scripts': ['path_control = path_control.node:main',
                                          'path_control_logger = path_control.logger:main',
                                          'optitrack_center = path_control.optitrack_center:main']})
