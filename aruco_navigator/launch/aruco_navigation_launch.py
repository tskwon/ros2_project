# realsense_aruco.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # RealSense 카메라 런치
    # ArUco 검출기
    aruco_detector = Node(
        package='aruco_navigator',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[{
            'marker_size': 0.1  # 실제 마커 크기 (미터)
        }],
        output='screen'
    )
    marker_navigator = Node(
        package='aruco_navigator',
        executable='marker_navigator',
        name='marker_navigator',
        output='screen'
    )
    return LaunchDescription([
        aruco_detector,
        marker_navigator
    ])