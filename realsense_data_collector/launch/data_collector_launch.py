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
                    package='realsense_data_collector',
                    executable='rgb_display_node',
                    name='realsense_rgb_display_node',
                    output='screen'
                )
            ]
        )
    ])