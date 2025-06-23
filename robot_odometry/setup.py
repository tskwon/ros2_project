import os
from setuptools import find_packages, setup

package_name = 'robot_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), ['launch/odometry_launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/ekf_config.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xotn',
    maintainer_email='james190414@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = robot_odometry.odometry_node:main',
            'calibration_node = robot_odometry.calibration_node:main',
        ],
    },
)
