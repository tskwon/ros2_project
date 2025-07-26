from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 카메라 이름 인자 선언
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='realsense_d415', # detector 노드와 토픽명 일치시키기 위해 변경
        description='Name of the Realsense camera'
    )

    # 카메라 이름 설정 (LaunchArgument에서 가져오기)
    camera_name = LaunchConfiguration('camera_name')

    return LaunchDescription([
        camera_name_arg, # 인자 선언 추가

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=camera_name, # 인자 사용
            namespace='d415',
            output='screen',
            parameters=[{
                'device_type': 'D415',
                'camera_name': camera_name, # 인자 사용 (토픽명과 일치)
                'enable_color': True,
                'enable_depth': True,
                'color_width': 640,
                'color_height': 480,
                'depth_width': 640,
                'depth_height': 480,
                'color_fps': 30,
                'depth_fps': 30,
                'enable_pointcloud': True, # 포인트 클라우드 활성화 (선택 사항)
                'align_depth': True,       # 깊이 이미지를 컬러 이미지에 정합 (매우 중요!)
                'optical_frame_id': 'd415_camera_link' # 프레임 ID 설정 (디버깅 및 TF 트리 확인에 유용)
            }]
        ),
        TimerAction(
            period=5.0, # Realsense 카메라 노드 초기화 대기
            actions=[
                Node(
                    package='yolo_obb_detection',
                    executable='black_object_detector_node',
                    name='black_object_detector_node',
                    output='screen',
                    # 만약 detector 노드에서 카메라 이름이 필요하다면 여기에 파라미터로 전달할 수 있습니다.
                    # parameters=[{'camera_base_frame': 'd415_camera_link'}] # 예시
                )
            ]
        )
    ])