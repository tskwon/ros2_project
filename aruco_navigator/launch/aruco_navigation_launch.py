# realsense_aruco.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Launch 인자 선언
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ArUco 검출기
    aruco_detector = Node(
        package='aruco_navigator',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[{
            'marker_size': 0.1,  # 실제 마커 크기 (미터)
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ========== RealSense D435i 카메라 ==========
    realsense_camera = IncludeLaunchDescription(
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
    )
    
    # ========== TF Static Publishers ==========
    # odom -> base_link 변환 (정적 변환 - 로봇이 움직이지 않는 경우)
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                  'odom', 'base_link'],
        output='screen'
    )
    
    # base_link -> camera_link 변환 (실제 로봇의 카메라 위치에 맞게 조정)
    # 원래 코드에서 이 부분만 수정:
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.15', '0.0', '0.25', '0.0', '-0.25', '0.0',  # pitch를 -0.25로 (아래쪽 향함)
                'base_link', 'camera_link'],
        output='screen'
    )
    
    # camera_link -> camera_color_frame 추가
    camera_color_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame_tf',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                'camera_link', 'camera_color_frame'],
        output='screen'
    )

    # camera_color_frame -> camera_color_optical_frame 변환
    camera_to_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical_tf',
        arguments=['0.0', '0.0', '0.0', '-0.5', '0.5', '-0.5', '0.5',
                'camera_color_frame', 'camera_color_optical_frame'],
        output='screen'
    )

    # ========== 선택적: Map 프레임 추가 ==========
    # map -> odom 변환 (SLAM 없이 사용하는 경우)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                  'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        # Launch 인자
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # 노드들
        realsense_camera,
        aruco_detector,
        
        # TF 변환들
        map_to_odom_tf,      # map -> odom
        odom_to_base_tf,     # odom -> base_link
        base_to_camera_tf,   # base_link -> camera_link
        camera_to_optical_tf # camera_link -> camera_color_optical_frame
    ])