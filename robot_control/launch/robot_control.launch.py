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
    
    mqtt_bridge = Node(
        package='mqtt_bridge_py',
        executable='mqtt_listener',
        name='mqtt_listener',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    robot_control = Node(
        package='robot_control',
        executable='robot_control_node',
        name='robot_control_node',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # # ========== RealSense D435 카메라 ==========
    # realsense_camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('realsense2_camera'),
    #         '/launch/rs_launch.py'
    #     ]),
    #     launch_arguments={
    #         'enable_color': 'true',
    #         'enable_infra1': 'false',
    #         'enable_infra2': 'false',
    #         'use_sim_time': use_sim_time
    #     }.items()
    # )
    
    # md_controller = Node(
    #         package='md_controller',
    #         executable='md_controller_node',
    #         name='md_controller_node',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time}]
    #     )
    

    

    return LaunchDescription([
        # Launch 인자
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # 노드들
        mqtt_bridge,
        # realsense_camera,
        # aruco_detector,
        robot_control
        # md_controller
    ])