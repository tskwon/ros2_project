from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense', # 노드 이름 설정
            output='screen',
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