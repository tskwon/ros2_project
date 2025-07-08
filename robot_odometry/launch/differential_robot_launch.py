#!/usr/bin/env python3
"""
차동구동 AMR 로봇 통합 Launch 파일
RealSense D435i 카메라, MyAHRS+ IMU, 모터 엔코더를 활용한 Visual-Inertial-Wheel SLAM
ArUco 마커 네비게이션 포함 - 수정된 TF 설정
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
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    # Launch 인자 선언
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_odometry = 'robot_odometry'
    pkg_myahrs_driver = 'myahrs_ros2_driver'
    pkg_robot_localization = 'robot_localization'
    config_dir = get_package_share_directory('robot_odometry')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')
    ekf_config_path = os.path.join(get_package_share_directory(pkg_odometry), 'config', 'ekf_config.yaml')
    
    # URDF 파일 경로
    urdf_file = os.path.join(get_package_share_directory(pkg_odometry), 'urdf', 'differential_robot.urdf.xacro')
    robot_description = ParameterValue(
          Command(['xacro ', urdf_file]),
          value_type=str
          )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ========== Robot State Publisher (URDF 로드) ==========
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # ========== Joint State Publisher (고정 조인트용) ==========
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        # ========== RealSense D435i 카메라 ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py'
            ]),
            launch_arguments={
                'enable_depth': 'true',
                'enable_color': 'true',
                'align_depth.enable': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'color_width': '640',
                'color_height': '480',
                'depth_width': '640',
                'depth_height': '480',
                'use_sim_time': use_sim_time
            }.items()
        ),

        # ========== TF Static Publishers (필요한 것만) ==========
        # RealSense 카메라 프레임 연결 (카메라 드라이버가 발행하지 않는 경우에만)
        # camera_link (URDF) -> camera_color_optical_frame (RealSense)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical_tf',
            # RealSense 광학 프레임 변환 (카메라 좌표계 -> 광학 좌표계)
            # 90도 회전: X축이 오른쪽, Y축이 아래쪽, Z축이 앞쪽을 향하도록
            arguments=['0.0', '0.0', '0.0', '-1.5708', '0.0', '-1.5708', 
                      'camera_link', 'camera_color_optical_frame'],
            output='screen'
        ),

        # ArUco 마커 맵 프레임 (글로벌 좌표계)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_aruco_map_tf',
            # map과 aruco_map을 동일하게 설정 (필요시 조정)
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 
                      'map', 'aruco_map'],
            output='screen'
        ),

        # ========== ArUco 마커 네비게이션 ==========
        Node(
            package='aruco_navigator',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        Node(
            package='aruco_navigator',
            executable='stable_aruco_detection_node',
            name='stable_aruco_detection_node',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # ========== 센서 및 제어 노드들 ==========
        # 모터 컨트롤러 (차동구동)
        Node(
            package='md_controller',
            executable='md_controller_node',
            name='md_controller_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # MyAHRS+ IMU
        Node(
            package=pkg_myahrs_driver,
            executable='myahrs_ros2_driver',
            name='myahrs_ros2_driver',
            output='screen',
            parameters=[
                config_file,
                {
                    'imu_frame_id': 'imu_link',
                    'publish_rate': 30.0,
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # 차동구동 엔코더 오도메트리
        Node(
            package='robot_odometry',
            executable='odometry_node',
            name='odometry_node',
            parameters=[{
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'use_sim_time': use_sim_time,
            }]
        ),

        # Robot Localization (EKF) - 차동구동 특화
        Node(
            package=pkg_robot_localization,
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                ekf_config_path,
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

    ])