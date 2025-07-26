#!/usr/bin/env python3
"""
AMR 로봇 통합 Launch 파일 with RTAB-Map
카메라, MyAHRS+ IMU, 모터 엔코더를 활용한 Visual-Inertial-Wheel SLAM
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch 인자 선언
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_odometry = 'robot_odometry'
    pkg_myahrs_driver = 'myahrs_ros2_driver' # IMU 드라이버 패키지 이름 확인
    pkg_robot_localization = 'robot_localization' # robot_localization 패키지 이름
    config_dir = get_package_share_directory('robot_odometry')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')
    ekf_config_path = os.path.join(get_package_share_directory(pkg_odometry), 'config', 'ekf_config.yaml')
    # 패키지 경로
    pkg_share = FindPackageShare('robot_odometry')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # RealSense D435
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            parameters=[{
                # 'depth_module.profile': '240x240x20',
                # 'rgb_camera.profile': '240x240x20',
                'enable_depth': True,
                'enable_color': True,
                'align_depth.enable': True,
                'enable_sync': True,
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package=pkg_myahrs_driver, # 'myahrs_ros2_driver' 패키지
            executable='myahrs_ros2_driver', # 해당 패키지의 실행 파일 이름
            name='myahrs_ros2_driver',
            output='screen',
            parameters=[
                config_file,
                # IMU 센서 파라미터 최적화
                {'imu_frame_id': 'imu_link'},
                {'publish_rate': 15.0},  # 발행 속도 제한 (필요시 조정)
                {'use_sim_time': use_sim_time}
            ]
            # IMU 드라이버에 필요한 파라미터가 있다면 여기에 추가
        ),
        # 모터 컨트롤러
        Node(
            package='md_controller',
            executable='md_controller',
            name='md_controller_node',
        ),

        # 엔코더 오도메트리
        Node(
            package='robot_odometry',
            executable='odometry_node',
            name='odometry_node',
        )
    ])
    