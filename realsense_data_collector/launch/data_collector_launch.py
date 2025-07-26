from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_d415',
            namespace='d415',
            output='screen',
            parameters=[{
                'device_type': 'D415',  # 디바이스 타입으로 지정
                'camera_name': 'd415_camera',
                'enable_color': True,
                'color_width': 640,
                'color_height': 480
            }]
        ),
        Node(
                package='realsense_data_collector',
                executable='rgb_display_node',
                name='realsense_rgb_display_node',
                output='screen'
            )
    ])