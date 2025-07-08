import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations
import math

class DifferentialDriveOdometry(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- 로봇 파라미터 선언 및 가져오기 ---
        # UKF는 정확한 파라미터에 민감하므로, 실제 로봇에 맞게 조정해야 합니다.
        self.declare_parameter('wheel_radius', 0.103)  # 바퀴 반지름 (미터) - Ø206mm 바퀴의 경우 0.103m (206/2/1000)
        self.declare_parameter('wheel_base', 0.503)    # 좌우 바퀴 중심 간 거리 (미터) - 예시 값, 실제 측정 필요
        self.declare_parameter('ticks_per_revolution', 90) # 90 적용

        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base_ = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.ticks_per_revolution_ = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value

        # --- 로봇 상태 변수 ---
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_timestamp = None # 마지막 엔코더 데이터 수신 시간

        # 로봇의 현재 위치 (odom 프레임 기준)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # yaw 각도 (라디안)

        # 로봇의 현재 속도 (base_link 프레임 기준)
        self.linear_velocity_x = 0.0
        self.angular_velocity_z = 0.0

        # --- 구독자 및 발행자 설정 ---
        # 엔코더 틱 값을 수신할 구독자
        self.encoder_sub_ = self.create_subscription(
            Int32MultiArray,
            '/encoder_values', # 엔코더 데이터가 발행되는 토픽
            self.encoder_callback,
            10
        )

        # 오도메트리 메시지를 발행할 발행자
        self.odom_pub = self.create_publisher(Odometry, '/odom/robot', 10)

        self.get_logger().info('오도메트리 노드 시작됨.')
        self.get_logger().info(f'바퀴 반지름: {self.wheel_radius_:.4f} m')
        self.get_logger().info(f'바퀴 간 거리: {self.wheel_base_:.4f} m')
        self.get_logger().info(f'엔코더 PPR: {self.ticks_per_revolution_} 틱/회전')


    def encoder_callback(self, msg):
        current_time = self.get_clock().now() # 현재 메시지 수신 시간

        # 엔코더 틱 값 (좌, 우 순서)
        left_ticks, right_ticks = msg.data[0], msg.data[1] 

        # --- 디버그 출력: 수신된 엔코더 틱 ---
        self.get_logger().debug(f'수신된 엔코더 틱: Left={left_ticks}, Right={right_ticks}')

        # 첫 번째 콜백 처리: 이전 틱 값과 시간을 초기화
        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_timestamp = current_time
            self.get_logger().info('첫 엔코더 데이터 수신. 오도메트리 초기화.')
            return

        # --- 시간 간격 (dt) 계산 ---
        dt = (current_time - self.last_timestamp).nanoseconds / 1e9 # 초 단위

        if dt <= 0: # 동일 시간대에 여러 메시지가 올 경우 0으로 나누는 것 방지
            self.get_logger().warn('시간 간격(dt)이 0 이거나 음수 입니다. 오도메트리 계산을 건너뜜.')
            return

        # --- 엔코더 틱 변화량 계산 ---
        delta_left_ticks = left_ticks - self.last_left_ticks
        delta_right_ticks = right_ticks - self.last_right_ticks

        # 다음 콜백을 위해 현재 틱 값과 시간 저장
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_timestamp = current_time

        # --- 틱을 거리로 변환 (미터) ---
        distance_per_tick = (2 * math.pi * self.wheel_radius_) / self.ticks_per_revolution_
        
        # 각 바퀴가 이동한 거리
        d_left = delta_left_ticks * distance_per_tick
        d_right = delta_right_ticks * distance_per_tick

        # --- 디버그 출력: 바퀴별 이동 거리 ---
        # self.get_logger().debug(f'dt: {dt:.4f}s, Delta Ticks: L={delta_left_ticks}, R={delta_right_ticks}')
        # self.get_logger().debug(f'바퀴 이동 거리: Left={d_left:.4f}m, Right={d_right:.4f}m')


        # --- 로봇의 이동량 및 각도 변화 계산 ---
        # 로봇 중심의 선형 이동 거리
        d_center = (d_left + d_right) / 2.0
        
        # 로봇의 회전 각도 변화 (라디안)
        d_theta = (d_right - d_left) / self.wheel_base_ if self.wheel_base_ != 0 else 0.0

        # --- 디버그 출력: 로봇의 미소 이동 및 회전 ---
        # self.get_logger().debug(f'로봇 미소 이동: Center={d_center:.4f}m, Angle={d_theta:.4f} rad ({math.degrees(d_theta):.2f} deg)')


        # --- 로봇 위치 (Pose) 업데이트 ---
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta
        self.theta = self.normalize_angle(self.theta)

        # --- 로봇 속도 (Twist) 업데이트 ---
        self.linear_velocity_x = d_center / dt
        self.angular_velocity_z = d_theta / dt

        # --- 디버그 출력: 현재 로봇 위치 및 속도 ---
        # self.get_logger().info(
        #     f'현재 오도메트리: X={self.x:.3f}m, Y={self.y:.3f}m, Yaw={math.degrees(self.theta):.2f}deg | '
        #     f'Vx={self.linear_velocity_x:.3f}m/s, Vyaw={self.angular_velocity_z:.3f}rad/s'
        # )

        # --- Odometry 메시지 발행 (이하 기존 코드와 동일) ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # 1. Pose (위치 및 방향) 정보 채우기
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # 2. Twist (선형 및 각속도) 정보 채우기
        odom_msg.twist.twist.linear.x = self.linear_velocity_x
        odom_msg.twist.twist.angular.z = self.angular_velocity_z

        # 3. Covariance (불확실성) 행렬 설정 (수정된 부분)
        # 대각 행렬 이외의 값은 0으로 유지합니다.
        # P_x, P_y, P_z, P_roll, P_pitch, P_yaw
        pose_covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-9
        ]
        odom_msg.pose.covariance = pose_covariance

        # V_x, V_y, V_z, V_roll, V_pitch, V_yaw
        twist_covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-9
        ]
        odom_msg.twist.covariance = twist_covariance

        self.odom_pub.publish(odom_msg)

    def normalize_angle(self, angle):
        """각도를 -π ~ π 라디안 범위로 정규화합니다."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()