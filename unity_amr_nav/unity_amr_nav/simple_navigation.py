# ~/ros2_ws/src/unity_amr_nav/unity_amr_nav/simple_navigation.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
import math

class SimpleNavigation(Node):
    def __init__(self):
        super().__init__('simple_navigation')
        
        # Publishers & Subscribers
        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.goal_callback, 10)
        # self.odom_sub = self.create_subscription(
        #     Odometry, '/odometry/filtered', self.odom_callback, 10)  # 기존 오도메트리 토픽
        
        # Navigation state
        # self.current_x = 0.0
        # self.current_y = 0.0
        # self.current_yaw = 0.0
        # self.goal_x = None
        # self.goal_y = None
        # self.is_navigating = False
        
        # Control parameters
        # self.goal_tolerance = 0.25  # 25cm 허용 오차
        # self.rotation_tolerance = 0.1  # 약 5.7도
        # self.max_linear_speed = 1.2
        # self.max_angular_speed = 1.8
        # self.min_linear_speed = 0.1
        # self.min_angular_speed = 0.1
        
        # PID gains
        # self.kp_linear = 1.8
        # self.kp_angular = 2.5
        # self.kd_angular = 0.1
        
        # State tracking
        # self.last_angle_error = 0.0
        # self.last_update_time = self.get_clock().now()
        
        # Control timer (25Hz for smooth control)
        # self.timer = self.create_timer(0.04, self.navigation_loop)
        
        self.get_logger().info('AMR Simple Navigation 시작됨 - Check Point 모드')
        # self.get_logger().info(f'오도메트리 토픽: {self.odom_sub.topic_name}')
        self.get_logger().info(f'목표 토픽: {self.goal_sub.topic_name}')

    def goal_callback(self, msg):
        """Unity NavigationClicker에서 클릭한 목표 위치 수신 - 터미널 출력"""
        goal_x = msg.point.x
        goal_y = msg.point.y
        goal_z = msg.point.z
        
        # 터미널에 상세 정보 출력
        self.get_logger().info("=" * 50)
        self.get_logger().info("새 Check Point 수신!")
        self.get_logger().info(f"X 좌표: {goal_x:.3f}")
        self.get_logger().info(f"Y 좌표: {goal_y:.3f}")
        self.get_logger().info(f"Z 좌표: {goal_z:.3f}")
        self.get_logger().info(f"좌표 (X, Y, Z): ({goal_x:.3f}, {goal_y:.3f}, {goal_z:.3f})")
        self.get_logger().info("=" * 50)

    # def odom_callback(self, msg):
    #     """기존 오도메트리 시스템에서 현재 로봇 위치 수신"""
    #     self.current_x = msg.pose.pose.position.x
    #     self.current_y = msg.pose.pose.position.y
    #     
    #     # 쿼터니언을 yaw 각도로 변환
    #     q = msg.pose.pose.orientation
    #     siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    #     self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # def navigation_loop(self):
    #     """메인 네비게이션 제어 루프"""
    #     if not self.is_navigating or self.goal_x is None:
    #         return
    #         
    #     # 목표까지 거리 및 방향 계산
    #     dx = self.goal_x - self.current_x
    #     dy = self.goal_y - self.current_y
    #     distance = math.sqrt(dx*dx + dy*dy)
    #     
    #     cmd = Twist()
    #     current_time = self.get_clock().now()
    #     dt = (current_time - self.last_update_time).nanoseconds / 1e9
    #     
    #     if distance < self.goal_tolerance:
    #         # 목표 도달 - 완전 정지
    #         cmd.linear.x = 0.0
    #         cmd.angular.z = 0.0
    #         self.stop_navigation()
    #     else:
    #         # 목표 방향 계산
    #         target_angle = math.atan2(dy, dx)
    #         angle_error = self.normalize_angle(target_angle - self.current_yaw)
    #         
    #         # 각속도 PD 제어
    #         angular_derivative = (angle_error - self.last_angle_error) / dt if dt > 0 else 0.0
    #         angular_velocity = (self.kp_angular * angle_error + 
    #                           self.kd_angular * angular_derivative)
    #         
    #         # 제어 전략: 각도 오차에 따른 동작 분리
    #         if abs(angle_error) > 0.5:  # 약 28도 - 제자리 회전
    #             cmd.linear.x = 0.0
    #             cmd.angular.z = self.limit_angular(angular_velocity)
    #             
    #         elif abs(angle_error) > 0.15:  # 약 8.6도 - 느린 전진 + 회전
    #             linear_velocity = self.kp_linear * distance * 0.3  # 속도 감소
    #             cmd.linear.x = self.limit_linear(linear_velocity)
    #             cmd.angular.z = self.limit_angular(angular_velocity * 0.7)
    #             
    #         else:  # 직진 위주
    #             # 거리에 비례한 선속도 (거리가 가까우면 속도 감소)
    #             speed_factor = min(1.0, distance / 2.0)  # 2m 이내에서 속도 감소
    #             linear_velocity = self.kp_linear * distance * speed_factor
    #             
    #             cmd.linear.x = self.limit_linear(linear_velocity)
    #             cmd.angular.z = self.limit_angular(angular_velocity * 0.3)  # 미세 조정
    #     
    #     # 최소 속도 적용 (정지 방지)
    #     if cmd.linear.x != 0.0 and abs(cmd.linear.x) < self.min_linear_speed:
    #         cmd.linear.x = self.min_linear_speed if cmd.linear.x > 0 else -self.min_linear_speed
    #         
    #     if cmd.angular.z != 0.0 and abs(cmd.angular.z) < self.min_angular_speed:
    #         cmd.angular.z = self.min_angular_speed if cmd.angular.z > 0 else -self.min_angular_speed
    #     
    #     # 속도 명령 발행
    #     self.cmd_vel_pub.publish(cmd)
    #     
    #     # 상태 업데이트
    #     self.last_angle_error = angle_error
    #     self.last_update_time = current_time

    # def stop_navigation(self):
    #     """네비게이션 완료 및 정지"""
    #     self.is_navigating = False
    #     self.goal_x = None
    #     self.goal_y = None
    #     
    #     # 완전 정지 명령
    #     stop_cmd = Twist()
    #     stop_cmd.linear.x = 0.0
    #     stop_cmd.angular.z = 0.0
    #     self.cmd_vel_pub.publish(stop_cmd)
    #     
    #     self.get_logger().info('목표 도달! 네비게이션 완료')

    # def normalize_angle(self, angle):
    #     """각도를 -π ~ π 범위로 정규화"""
    #     while angle > math.pi:
    #         angle -= 2 * math.pi
    #     while angle < -math.pi:
    #         angle += 2 * math.pi
    #     return angle
    
    # def limit_linear(self, vel):
    #     """선속도 제한"""
    #     return max(-self.max_linear_speed, min(self.max_linear_speed, vel))
    
    # def limit_angular(self, vel):
    #     """각속도 제한"""
    #     return max(-self.max_angular_speed, min(self.max_angular_speed, vel))
    
    # def emergency_stop(self):
    #     """비상 정지"""
    #     stop_cmd = Twist()
    #     self.cmd_vel_pub.publish(stop_cmd)
    #     self.stop_navigation()
    #     self.get_logger().warn('비상 정지 실행!')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 신호 수신')
        # node.emergency_stop()
    except Exception as e:
        node.get_logger().error(f'예상치 못한 오류: {str(e)}')
        # node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()