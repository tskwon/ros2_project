from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # QoS 설정 (필요 시 재사용 가능)
    qos = {'reliability': 'RELIABLE', 'durability': 'VOLATILE', 'history': 'KEEP_LAST', 'depth': 10}

    # md_teleop 노드 (기존 패키지에서 가져왔다고 가정)
    md_teleop_node = Node(
        package='md_teleop',
        executable='md_teleop_key',  # 올바른 실행 파일 이름
        name='md_teleop_node',           # 코드와 일치 (필요 시 조정)
        output='screen'
    )

    # md_controller 노드 (기존 패키지에서 가져왔다고 가정)
    md_controller_node = Node(
        package='md_controller',  
        executable='md_controller',  
        name='md_controller_node',
        output='screen'
    )

    # ir_sensor 노드 (가상의 IR 센서 노드)
    ir_sensor_node = Node(
        package='ir_sensor_reader',  # IR 센서 노드가 line_tracing 패키지에 있다고 가정
        executable='ir_sensor_reader',  # 가상의 실행 파일 이름
        name='ir_sensor_reader_node',
        output='screen'
    )

    # line_tracing 노드
    line_tracing_node = Node(
        package='line_tracing',
        executable='line_tracing',
        name='line_tracing_node',
        output='screen'
    )

    # 런치 설명 생성
    return LaunchDescription([
        md_teleop_node,
        md_controller_node,
        ir_sensor_node,
        line_tracing_node
    ])