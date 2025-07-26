#!/usr/bin/env python3
"""
AMR 로봇 통합 Launch 파일 with RTAB-Map
RealSense D435i 카메라, MyAHRS+ IMU, 모터 엔코더를 활용한 Visual-Inertial-Wheel SLAM
ArUco 마커 네비게이션 포함
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
    pkg_myahrs_driver = 'myahrs_ros2_driver'
    pkg_robot_localization = 'robot_localization'
    config_dir = get_package_share_directory('robot_odometry')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')
    ekf_config_path = os.path.join(get_package_share_directory(pkg_odometry), 'config', 'ekf_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ========== RealSense D435i 카메라 ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py'
            ]),
            launch_arguments={
                'enable_depth': 'true',
                'enable_color': 'true',
                'align_depth.enable': 'true',  # Depth를 Color에 정렬
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'color_width': '640',
                'color_height': '480',
                'depth_width': '640',
                'depth_height': '480',
                'use_sim_time': use_sim_time
            }.items()
        ),
        
        # ========== ArUco 마커 검출 및 네비게이션 ==========
        # ArUco 검출기
        Node(
            package='aruco_navigator',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[{
                'marker_size': 0.1,  # 실제 마커 크기 (미터)
                'camera_frame': 'camera_color_optical_frame',  # RealSense 컬러 프레임
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),
        
        # 안정적인 ArUco 네비게이션 노드
        Node(
            package='aruco_navigator',  # 해당 패키지 이름으로 변경
            executable='stable_aruco_navigator',
            name='stable_aruco_navigator',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),
        # ========== 센서 노드들 (순차 실행) ==========
        # 모터 컨트롤러 (가장 먼저 실행)
        Node(
            package='md_controller',
            executable='md_controller_node',
            name='md_controller_node',
            output='screen',
            parameters=[
                config_file,  # 설정 파일 추가
                {
                    'use_sim_time': use_sim_time
                }
            ],
            respawn=True,  # 노드가 죽으면 재시작
            respawn_delay=2.0  # 2초 후 재시작
        ),

        # MyAHRS+ IMU (모터 컨트롤러 이후 실행)
        Node(
            package=pkg_myahrs_driver,
            executable='myahrs_ros2_driver',
            name='myahrs_ros2_driver',
            output='screen',
            parameters=[
                config_file,
                {
                    'imu_frame_id': 'imu_link',  # TF와 일치
                    'publish_rate': 50.0,
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # 엔코더 오도메트리
        Node(
            package='robot_odometry',
            executable='odometry_node',
            name='odometry_node',
            parameters=[{
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'use_sim_time': use_sim_time
            }]
        ),

        # Robot Localization (EKF)
        Node(
            package=pkg_robot_localization,
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                ekf_config_path
            ]
        ),

    ])