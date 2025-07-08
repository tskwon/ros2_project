import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import math
import tf_transformations
import numpy as np

class CorrectDirectionNavigator(Node):
    def __init__(self):
        super().__init__('correct_direction_navigator')
        
        # 화면 중심 좌표 (320x240 해상도 기준)
        self.image_center_x = 160  # 320/2
        self.image_center_y = 120  # 240/2
        
        # 제어 파라미터
        self.max_linear_speed = 0.25   # 최대 선속도
        self.max_angular_speed = 0.6   # 최대 각속도
        
        # 허용 오차 (정밀한 제어)
        self.pixel_tolerance_x = 5    # 좌우 10픽셀 허용
        self.pixel_tolerance_y = 5    # 상하 15픽셀 허용  
        self.depth_tolerance = 0.05    # 거리 8cm 허용
        
        # 목표 거리 (미터)
        self.target_distance = 0.4     # 40cm
        
        # 제어 게인 (부호 수정됨)
        self.linear_kp = 0.8           # 전후 이동 게인
        self.lateral_kp = 0.003        # 좌우 오차 → 각속도 변환 게인
        self.vertical_kp = 0.002       # 상하 오차 → 선속도 조정 게인
        
        # 안전 파라미터
        self.min_safe_distance = 0.15
        self.max_work_distance = 1.2
        
        # 구독자
        self.marker_pose_sub = self.create_subscription(
            PoseStamped, '/aruco/marker_pose', self.marker_pose_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/robot', self.odom_callback, 10)
        
        self.target_id_sub = self.create_subscription(
            Int32, '/aruco/target_id', self.target_id_callback, 10)
        
        # 발행자
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 제어 타이머
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 상태 변수
        self.current_robot_pose = None
        self.marker_pose = None
        self.marker_pixel_x = None
        self.marker_pixel_y = None
        self.marker_distance = None
        self.marker_last_seen = None
        self.target_id = -1
        
        # 내부 상태
        self.navigation_state = "WAITING"  # WAITING, SIMULTANEOUS_CONTROL, ALIGNED, LOST
        self.marker_timeout = 3.0
        
        self.get_logger().info("🎯 정확한 방향 제어 네비게이터 시작됨")
        self.get_logger().info(f"📐 목표: 화면 중심({self.image_center_x}, {self.image_center_y}), 거리 {self.target_distance}m")
        
    def target_id_callback(self, msg):
        self.target_id = msg.data
        self.get_logger().info(f"🎯 목표 마커 ID: {self.target_id}")
        
    def odom_callback(self, msg):
        self.current_robot_pose = msg.pose.pose
        
    def marker_pose_callback(self, msg):
        """마커 위치와 픽셀 좌표 업데이트"""
        self.marker_pose = msg.pose
        self.marker_last_seen = self.get_clock().now()
        
        # 카메라 좌표계에서 거리 계산
        self.marker_distance = msg.pose.position.z
        
        # 3D 위치를 픽셀 좌표로 변환
        camera_fx = 154.25
        camera_fy = 154.25
        camera_cx = 160.0
        camera_cy = 120.0
        
        if self.marker_distance > 0:
            self.marker_pixel_x = (msg.pose.position.x * camera_fx / self.marker_distance) + camera_cx
            self.marker_pixel_y = (msg.pose.position.y * camera_fy / self.marker_distance) + camera_cy
        
        self.get_logger().debug(
            f"📍 마커: 픽셀({self.marker_pixel_x:.1f}, {self.marker_pixel_y:.1f}), "
            f"거리: {self.marker_distance:.2f}m"
        )
        
    def control_loop(self):
        """정확한 방향 제어 루프"""
        current_time = self.get_clock().now()
        
        # 마커 타임아웃 체크
        if (self.marker_last_seen is None or 
            (current_time - self.marker_last_seen).nanoseconds / 1e9 > self.marker_timeout):
            self.navigation_state = "LOST"
            self.stop_robot()
            self.get_logger().warn("📡 마커 신호 끊김")
            return
            
        if (self.marker_pose is None or self.marker_pixel_x is None or 
            self.marker_distance is None):
            self.get_logger().debug("🔍 마커 데이터 대기 중...")
            return
            
        # 안전 거리 체크
        if self.marker_distance < self.min_safe_distance:
            self.stop_robot()
            self.get_logger().warn(f"⚠️ 안전 거리: {self.marker_distance:.2f}m")
            return
            
        # 오차 계산
        pixel_error_x = self.marker_pixel_x - self.image_center_x  # 좌우 오차
        pixel_error_y = self.marker_pixel_y - self.image_center_y  # 상하 오차
        distance_error = self.marker_distance - self.target_distance  # 거리 오차
        
        self.get_logger().info(
            f"📊 오차 - X: {pixel_error_x:.1f}px, Y: {pixel_error_y:.1f}px, "
            f"거리: {distance_error:.2f}m, 상태: {self.navigation_state}"
        )
        
        # 정렬 완료 체크 (모든 축 고려)
        if (abs(pixel_error_x) <= self.pixel_tolerance_x and 
            abs(pixel_error_y) <= self.pixel_tolerance_y and 
            abs(distance_error) <= self.depth_tolerance):
            
            self.navigation_state = "ALIGNED"
            self.stop_robot()
            self.get_logger().info("✅ 완벽 정렬 완료! (X, Y, 거리 모두)")
            return
        
        # 정확한 방향 제어 계산
        self.navigation_state = "SIMULTANEOUS_CONTROL"
        twist = Twist()
        
        # 1. 거리 제어 (부호 수정!)
        if distance_error > 0:
            # 현재 거리가 목표보다 멀면 → 가까이 가야 함 → 전진
            base_linear_velocity = self.linear_kp * distance_error  # 양수 (전진)
        else:
            # 현재 거리가 목표보다 가까우면 → 멀어져야 함 → 후진
            base_linear_velocity = self.linear_kp * distance_error  # 음수 (후진)
        
        # 2. Y오차를 선속도로 조정 (상하 위치 조정)
        if pixel_error_y < 0:
            # 마커가 화면 위쪽에 있으면 → 더 가까이 가야 함
            vertical_adjustment = self.vertical_kp * abs(pixel_error_y)
        else:
            # 마커가 화면 아래쪽에 있으면 → 조금 멀어져야 함
            vertical_adjustment = -self.vertical_kp * abs(pixel_error_y)
        
        # 3. X오차를 각속도로 조정 (좌우 방향 조정)
        if pixel_error_x > 0:
            # 마커가 화면 오른쪽에 있으면 → 시계방향 회전
            angular_velocity = self.lateral_kp * pixel_error_x
        else:
            # 마커가 화면 왼쪽에 있으면 → 반시계방향 회전
            angular_velocity = self.lateral_kp * pixel_error_x
        
        # 4. 거리에 따른 각속도 조정 (멀리 있을 때 더 큰 각속도)
        distance_factor = min(2.0, self.marker_distance / self.target_distance)
        angular_velocity *= distance_factor
        
        # 5. 최종 속도 계산
        twist.linear.x = base_linear_velocity + vertical_adjustment
        twist.angular.z = -angular_velocity  # 카메라 좌표계 보정
        
        # 6. 속도 제한
        twist.linear.x = max(-self.max_linear_speed, 
                           min(twist.linear.x, self.max_linear_speed))
        twist.angular.z = max(-self.max_angular_speed, 
                            min(twist.angular.z, self.max_angular_speed))
        
        # 7. 미세 조정 단계에서는 속도 감소
        total_error = abs(pixel_error_x) + abs(pixel_error_y) + abs(distance_error) * 50
        if total_error < 50:  # 오차가 작으면 천천히
            twist.linear.x *= 0.5
            twist.angular.z *= 0.6
            self.get_logger().info(f"🎯 미세 조정 모드: 총 오차 {total_error:.1f}")
        
        # 8. 매우 큰 오차가 있을 때 안전하게 제어
        if abs(pixel_error_x) > 50 or abs(distance_error) > 0.3:
            twist.linear.x *= 0.7  # 속도 감소
            twist.angular.z *= 0.8
            self.get_logger().info("⚠️ 큰 오차 감지 - 안전 모드")
        
        # 제어 명령 발송
        self.cmd_pub.publish(twist)
        
        self.get_logger().info(
            f"🚗 제어: 선속도 {twist.linear.x:.3f}, 각속도 {twist.angular.z:.3f} "
            f"(거리오차: {distance_error:.2f}, 로직: {'전진' if distance_error > 0 else '후진'})"
        )
        
    def stop_robot(self):
        """로봇 완전 정지"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        
    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = CorrectDirectionNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 사용자 중단")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()