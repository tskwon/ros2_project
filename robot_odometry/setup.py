import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch 파일들
        (os.path.join('share', package_name, 'launch'), [
            'launch/odometry_launch.py',
            'launch/robot_odometry_launch.py', 
            'launch/robot_sensor_send_launch.py',
            'launch/differential_robot_launch.py',  # 새로운 차동구동 런치 파일
        ]),
        
        # 설정 파일들
        (os.path.join('share', package_name, 'config'), [
            'config/ekf_config.yaml',
            'config/slam_config.yaml',
            'config/config.yaml',
            'config/nav2_params.yaml',  # 로봇 설정 파일
        ]),
        
        # URDF 파일들
        (os.path.join('share', package_name, 'urdf'), [
            'urdf/differential_robot.urdf.xacro',   # 차동구동 로봇 URDF
        ]),
        
        # RViz 설정 파일들
        (os.path.join('share', package_name, 'rviz'), [
            'rviz/differential_robot.rviz',         # 차동구동 로봇 RViz 설정
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xotn',
    maintainer_email='james190414@gmail.com',
    description='Differential drive robot odometry and navigation package with RealSense D435, MyAHRS+ IMU, and ArUco marker navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 기존 노드들
            'odometry_node = robot_odometry.odometry_node:main',
            'calibration_node = robot_odometry.calibration_node:main',
            'odometry_reader = robot_odometry.odometry_reader:main',
            'test_navigation = robot_odometry.test_navigation_controller:main',
            'square_navigation = robot_odometry.square_navigation_controller:main',
            'wheel_joint_publisher = robot_odometry.wheel_joint_publisher:main',
        ],
    },
)