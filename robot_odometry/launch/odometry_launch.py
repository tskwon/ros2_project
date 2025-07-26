import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_odometry = 'robot_odometry'
    pkg_myahrs_driver = 'myahrs_ros2_driver' # IMU 드라이버 패키지 이름 확인
    pkg_robot_localization = 'robot_localization' # robot_localization 패키지 이름
    config_dir = get_package_share_directory('robot_odometry')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')
    
    # EKF 설정 파일 경로
    # 일반적으로 ekf_config.yaml은 robot_odometry 패키지 내 config 폴더에 둡니다.
    ekf_config_path = os.path.join(get_package_share_directory(pkg_odometry), 'config', 'ekf_config.yaml')

    return LaunchDescription([
        # 1. 당신이 만든 휠 오도메트리 노드 실행
        Node(
            package=pkg_odometry,
            executable='odometry_node',
            name='odometry_node', # 이 노드의 ROS 이름은 'odometry_node'
            output='screen',
        ),

        # 2. IMU 드라이버 노드 실행
        Node(
            package=pkg_myahrs_driver, # 'myahrs_ros2_driver' 패키지
            executable='myahrs_ros2_driver', # 해당 패키지의 실행 파일 이름
            name='myahrs_ros2_driver',
            output='screen',
            parameters=[
                config_file,
                # IMU 센서 파라미터 최적화
                {'imu_frame_id': 'imu_link'},
                {'publish_rate': 30.0}  # 발행 속도 제한 (필요시 조정)
            ]
            # IMU 드라이버에 필요한 파라미터가 있다면 여기에 추가
        ),
            # md_controller 노드 (기존 패키지에서 가져왔다고 가정)
        Node(
            package='md_controller',  
            executable='md_controller_node',  
            name='md_controller_node',
            output='screen'
        ),
        # 3. robot_localization 패키지의 EKF 노드 실행 (센서 퓨전 담당)
        Node(
            package=pkg_robot_localization, # 'robot_localization' 패키지
            executable='ekf_node',          # 'ekf_node' 실행 파일
            name='ekf_node',         # EKF 노드의 ROS 이름 (ekf_config.yaml과 일치해야 함)
            output='screen',
            parameters=[ekf_config_path]    # EKF 설정 파일 로드
        )
    ])