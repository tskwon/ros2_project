from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ArUco 마커 검출 노드
    aruco_detector = Node(
        package='aruco_navigator',
        executable='aruco_marker_detector',
        name='aruco_marker_detector',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # RealSense D435 카메라 노드
    realsense_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_d435',
        namespace='d435',
        output='screen',
        parameters=[{
            'serial_no': '938422073188',  # D435 시리얼 번호
            'device_type': 'D435',
            'camera_name': 'd435_camera',
            'enable_color': True,
            'enable_depth': True,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            'color_fps': 30,
            'depth_fps': 30
        }]
    )
    md_controller = Node(
            package='md_controller',
            executable='md_controller_node',
            name='md_controller_node',
            output='screen'
        )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        realsense_camera,
        aruco_detector,
        md_controller 
    ])
