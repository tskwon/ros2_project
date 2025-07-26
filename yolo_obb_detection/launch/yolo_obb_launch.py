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
                'enable_depth': True,
                'color_width': 640,
                'color_height': 480,
                'depth_width': 640,
                'depth_height': 480,
                'color_fps': 30,
                'depth_fps': 30,
            }]
        ),
        TimerAction(
            period=5.0,  # 5초 대기
            actions=[
                Node(
                    package='yolo_obb_detection',
                    executable='yolo_obb_node',
                    name='yolo_obb_node',
                    output='screen',
                )
            ]
        )
    ])