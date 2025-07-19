from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 로봇 팔 비전 테스트 노드
        Node(
            package='robot_arm_test',  # 패키지 이름에 맞게 수정
            executable='robot_arm_vision_test',
            name='robot_arm_vision_test_node',
            output='screen',
            parameters=[
                # 필요시 파라미터 추가
            ]
        ),
        
        # 테스트 명령 전송 노드 (옵션)
        Node(
            package='robot_arm_test',
            executable='test_commander',
            name='test_commander_node',
            output='screen'
        )
    ])