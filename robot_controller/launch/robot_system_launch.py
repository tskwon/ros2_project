from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MQTT 브리지 노드 추가 (mqtt_bridge 패키지 실행파일)
        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge_node',
            name='mqtt_bridge_node',
            output='screen'
        ),

        # 로봇 시스템 메인 제어 노드
        Node(
            package='robot_controller',
            executable='robot_controller_node',
            name='robot_controller_node',
            output='screen'
        ),

        # md_teleop 노드
        Node(
            package='md_teleop',
            executable='md_teleop_key',
            name='md_teleop_node',
            output='screen'
        ),

        # md_controller 노드
        Node(
            package='md_controller',
            executable='md_controller',
            name='md_controller_node',
            output='screen'
        ),

        # IR 센서 노드
        Node(
            package='ir_sensor_reader',
            executable='ir_sensor_reader',
            name='ir_sensor_reader_node',
            output='screen'
        ),

        # 라인트레이싱 제어 노드
        Node(
            package='line_tracing',
            executable='line_tracing',
            name='line_tracing_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])
