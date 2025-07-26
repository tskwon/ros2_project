#!/usr/bin/env python3
"""
AMR 로봇 통합 Launch 파일 with ArUco Navigation
RealSense D435i 카메라, MyAHRS+ IMU, 모터 엔코더를 활용한 Visual-Inertial-Wheel SLAM
ArUco 마커 네비게이션 포함 (odom 좌표계 기준)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch 인자
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_odometry = 'robot_odometry'
    
    # 설정 파일 경로
    config_dir = get_package_share_directory(pkg_odometry)
    config_file = os.path.join(config_dir, 'config', 'config.yaml')
    ekf_config_path = os.path.join(config_dir, 'config', 'ekf_config.yaml')
    urdf_file = os.path.join(config_dir, 'urdf', 'differential_robot.urdf.xacro')

    # Xacro 파일을 파싱하여 로봇 설명 생성
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    nav2_params_file_path = os.path.join(
        get_package_share_directory('robot_odometry'), # 또는 your_robot_nav_package
        'config',
        'nav2_params.yaml' # 위에서 저장한 YAML 파일 이름
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # ========== 로봇 기본 시스템 ==========
        # Robot State Publisher: URDF에 정의된 모든 링크의 TF를 발행
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

        # Joint State Publisher: URDF의 Fixed가 아닌 조인트 상태를 발행
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 30
            }]
        ),

        
        # 바퀴 조인트 퍼블리셔: 엔코더를 통해 바퀴 회전 정보를 조인트 상태로 발행
        Node(
            package='robot_odometry',
            executable='wheel_joint_publisher',
            name='wheel_joint_publisher',
            output='screen',
            parameters=[{
                'wheel_radius': 0.103,
                'wheel_base': 0.503,
                'left_wheel_joint': 'left_wheel_joint',
                'right_wheel_joint': 'right_wheel_joint',
                'publish_rate': 30.0,
                'use_sim_time': use_sim_time
            }]
        ),

        # ========== 센서 시스템 ==========
        # RealSense D435i 카메라 드라이버
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py'
            ]),
            launch_arguments={
                'enable_depth': 'true',
                'enable_color': 'true',
                'align_depth.enable': 'true',
                'base_frame_id': 'camera_link',  # URDF의 camera_link와 연결
                'publish_tf': 'false',  # TF 충돌 방지: URDF가 TF 담당
                'use_sim_time': use_sim_time
            }.items()
        ),

        # MyAHRS+ IMU 드라이버 (선택사항 - 에러 발생 시 주석 처리)
        Node(
            package='myahrs_ros2_driver',
            executable='myahrs_ros2_driver',
            name='myahrs_ros2_driver',
            output='screen',
            parameters=[
                config_file,  # config.yaml 파일 경로 (시리얼 포트 등 설정)
                {
                    'publish_rate': 30.0,
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # ========== 오도메트리 및 센서 융합 ==========
        # 엔코더 오도메트리 노드
        Node(
            package='robot_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'use_sim_time': use_sim_time,
            }]
        ),

        # Robot Localization (EKF) 노드: 센서 융합을 통해 정확한 odom 추정
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
        ),

        # ========== 모터 제어 ==========
        # 모터 컨트롤러 노드
        Node(
            package='md_controller',
            executable='md_controller_node',
            name='md_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # ========== ArUco 마커 네비게이션 시스템 ==========
        # ArUco 마커 검출 노드들 (센서 시스템 안정화 후 실행)
        TimerAction(
            period=5.0,  # 5초 후 실행
            actions=[
                Node(
                    package='aruco_navigator',
                    executable='aruco_detector',
                    name='aruco_detector',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'camera_frame': 'camera_color_optical_frame',  # RealSense optical frame
                        'reference_frame': 'odom',  # odom 좌표계 사용
                        'marker_size': 0.1,  # 마커 크기 (미터)
                        'use_ground_projection': True,  # 지면 투영 활성화
                        'ground_z_offset': 0.0,  # 지면 높이
                        'use_latest_transform': True,  # 시간 동기화 문제 해결
                    }],
                    output='screen',
                    emulate_tty=True
                ),

                Node(
                    package='aruco_navigator',
                    executable='stable_aruco_detection_node',
                    name='stable_aruco_detection_node',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'detection_timeout': 2.0,  # 검출 안정성 타임아웃
                        'min_detections': 5,  # 안정 검출 최소 횟수
                    }],
                    output='screen'
                ),
            ]
        ),

        
    ])