#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
from enum import Enum

class RobotState(Enum):
    """로봇 상태 정의"""
    IDLE = "IDLE"
    ROTATING = "ROTATING"
    MOVING_FORWARD = "MOVING_FORWARD"
    FINE_POSITIONING = "FINE_POSITIONING"
    REPOSITIONING = "REPOSITIONING"
    STOPPED = "STOPPED"

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # 구독자 및 퍼블리셔 설정
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.state_publisher = self.create_publisher(
            String,
            '/robot_state',
            10)
        
        # 현재 상태
        self.current_state = RobotState.IDLE
        
        # 현재 위치 및 자세
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # 목표 설정
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        
        # 정밀 제어 파라미터
        self.position_tolerance = 0.02          # 2cm 허용 오차
        self.fine_tolerance = 0.01              # 1cm 미세 조정
        self.angle_tolerance = math.radians(2)  # 2도 각도 허용 오차
        
        # 거리별 구간 설정
        self.deceleration_distance = 0.3        # 80cm에서 감속 시작
        self.slow_approach_distance = 0.2       # 30cm에서 저속 접근
        self.emergency_brake_distance = 0.5     # 10cm에서 긴급 제동
        
        # 속도 제한
        self.max_speed = 0.3                    # 최대 속도
        self.decel_speed = 0.2                  # 감속 구간 속도
        self.slow_speed = 0.1                   # 저속 구간 속도  
        self.fine_speed = 0.05                  # 미세 조정 속도
        self.angular_speed = 0.5                # 회전 속도
        
        # PID 제어 파라미터
        self.linear_kp = 1.0
        self.angular_kp = 1.0
        self.deceleration_factor = 0.5          # 감속도 (m/s²)
        
        # 미션 설정 (정사각형 경로)
        self.mission_points = [
            (1.0, 0.0),   # 동쪽으로 1미터
            (1.0, 1.0),   # 북쪽으로 1미터
            (0.0, 1.0),   # 서쪽으로 1미터
            (0.0, 0.0),   # 남쪽으로 1미터 (원점 복귀)
        ]
        self.current_mission_index = 0
        
        # 상태 변수
        self.mission_started = False
        self.data_received = False
        
        # 정지 확인 및 안정성 카운터
        self.fine_positioning_count = 0
        self.position_stable_threshold = 15     # 1.5초간 안정 상태 유지
        self.repositioning_attempts = 0
        self.max_repositioning_attempts = 3
        
        # 이전 상태 저장 (속도 계산용)
        self.prev_time = time.time()
        self.prev_x = 0.0
        self.prev_y = 0.0
        
        # 타이머 (제어 루프)
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('🤖 Navigation Controller 시작됨')
        self.get_logger().info('📡 미션 시작을 위해 5초 대기 중...')
        
        # 미션 시작 타이머
        self.mission_start_timer = self.create_timer(5.0, self.start_mission)

    def odometry_callback(self, msg):
        """오도메트리 데이터 수신 및 속도 계산"""
        # 위치 정보 추출
        prev_x, prev_y = self.current_x, self.current_y
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 쿼터니언에서 yaw 각도 계산
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                                     1.0 - 2.0 * (qy * qy + qz * qz))
        
        # 현재 속도 계산 (추정값)
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt > 0 and self.data_received:
            dx = self.current_x - prev_x
            dy = self.current_y - prev_y
            self.current_linear_velocity = math.sqrt(dx*dx + dy*dy) / dt
        
        self.prev_time = current_time
        self.data_received = True

    def start_mission(self):
        """미션 시작"""
        if self.data_received:
            self.mission_started = True
            self.set_next_target()
            self.get_logger().info('🚀 미션 시작!')
            self.mission_start_timer.cancel()
        else:
            self.get_logger().warn('⚠️  오도메트리 데이터 대기 중...')

    def set_next_target(self):
        """다음 목표점 설정"""
        if self.current_mission_index < len(self.mission_points):
            self.target_x, self.target_y = self.mission_points[self.current_mission_index]
            
            # 목표 방향 계산
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            self.target_yaw = math.atan2(dy, dx)
            
            # 재접근 시도 횟수 초기화
            self.repositioning_attempts = 0
            
            self.get_logger().info(
                f'🎯 목표점 {self.current_mission_index + 1}: '
                f'({self.target_x:.2f}, {self.target_y:.2f}), '
                f'목표각도: {math.degrees(self.target_yaw):.1f}°'
            )
            
            # 회전 상태로 전환
            self.change_state(RobotState.ROTATING)
        else:
            self.get_logger().info('🏁 모든 미션 완료!')
            self.change_state(RobotState.STOPPED)

    def control_loop(self):
        """메인 제어 루프"""
        if not self.mission_started or not self.data_received:
            return
        
        # 상태별 제어 로직
        if self.current_state == RobotState.ROTATING:
            self.handle_rotation()
        elif self.current_state == RobotState.MOVING_FORWARD:
            self.handle_forward_movement()
        elif self.current_state == RobotState.FINE_POSITIONING:
            self.handle_fine_positioning()
        elif self.current_state == RobotState.REPOSITIONING:
            self.handle_repositioning()
        elif self.current_state == RobotState.STOPPED:
            self.handle_stop()

    def handle_rotation(self):
        """회전 제어"""
        angle_error = self.normalize_angle(self.target_yaw - self.current_yaw)
        
        if abs(angle_error) < self.angle_tolerance:
            # 회전 완료 → 직진 상태로 전환
            self.get_logger().info('✅ 회전 완료')
            self.change_state(RobotState.MOVING_FORWARD)
            return
        
        # 각속도 제어 (PID)
        angular_vel = self.angular_kp * angle_error
        angular_vel = max(-self.angular_speed, min(self.angular_speed, angular_vel))
        
        # 제어 명령 전송
        self.send_velocity(0.0, angular_vel)
        
        self.get_logger().info(
            f'🔄 회전중: 현재각도 {math.degrees(self.current_yaw):.1f}°, '
            f'목표각도 {math.degrees(self.target_yaw):.1f}°, '
            f'오차 {math.degrees(angle_error):.1f}°'
        )

    def handle_forward_movement(self):
        """개선된 직진 제어 (다단계 감속)"""
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 목표점 도달 판정
        if distance < self.position_tolerance:
            self.get_logger().info('🎯 목표점 도달! 미세 위치 조정 시작')
            self.change_state(RobotState.FINE_POSITIONING)
            return
        
        # 목표 방향 재계산 (실시간 보정)
        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.current_yaw)
        
        # 다단계 속도 제어
        linear_vel = self.calculate_target_speed(distance)
        
        # 오버슈트 방지 로직
        braking_distance = self.calculate_braking_distance()
        if distance < braking_distance:
            linear_vel = min(linear_vel, distance * 2)  # 강제 감속
        
        # 방향 보정 (5도 이상 벗어나면 보정)
        angular_vel = 0.0
        if abs(heading_error) > math.radians(5):
            angular_vel = self.angular_kp * heading_error * 0.3  # 약한 보정
            # 큰 방향 오차 시 속도 감소
            if abs(heading_error) > math.radians(15):
                linear_vel *= 0.5
        
        # 제어 명령 전송
        self.send_velocity(linear_vel, angular_vel)
        
        self.get_logger().info(
            f'🚗 직진중: 현재위치 ({self.current_x:.2f}, {self.current_y:.2f}), '
            f'목표위치 ({self.target_x:.2f}, {self.target_y:.2f}), '
            f'거리 {distance:.3f}m, 속도 {linear_vel:.2f}m/s'
        )

    def calculate_target_speed(self, distance):
        """거리에 따른 목표 속도 계산"""
        if distance < self.emergency_brake_distance:
            # 긴급 제동 구간
            return self.fine_speed * 0.5
        elif distance < self.slow_approach_distance:
            # 저속 접근 구간
            return min(self.slow_speed, distance * 3)
        elif distance < self.deceleration_distance:
            # 감속 구간
            return min(self.decel_speed, distance * 1.5)
        else:
            # 정속 구간
            return self.max_speed

    def calculate_braking_distance(self):
        """현재 속도 기반 제동 거리 계산"""
        if self.current_linear_velocity > 0:
            # 제동 거리 = v²/(2*a)
            braking_dist = (self.current_linear_velocity ** 2) / (2 * self.deceleration_factor)
            return max(braking_dist, 0.05)  # 최소 5cm
        return 0.05

    def handle_fine_positioning(self):
        """미세 위치 조정"""
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.fine_tolerance:
            # 정지 상태 유지
            self.send_velocity(0.0, 0.0)
            self.fine_positioning_count += 1
            
            self.get_logger().info(
                f'🔧 미세조정: 오차 {distance:.3f}m, 안정성 {self.fine_positioning_count}/{self.position_stable_threshold}'
            )
            
            # 연속으로 정지 조건 만족 시 완료
            if self.fine_positioning_count >= self.position_stable_threshold:
                self.get_logger().info('✅ 미세 위치 조정 완료!')
                self.current_mission_index += 1
                self.fine_positioning_count = 0
                self.set_next_target()
                return
        else:
            # 미세 조정이 필요한 경우
            if distance > self.position_tolerance:
                # 너무 멀어진 경우 재접근 모드로 전환
                self.get_logger().warn(f'⚠️  위치 이탈 감지: {distance:.3f}m')
                self.change_state(RobotState.REPOSITIONING)
                return
            
            # 극저속 미세 조정
            desired_yaw = math.atan2(dy, dx)
            heading_error = self.normalize_angle(desired_yaw - self.current_yaw)
            
            linear_vel = min(self.fine_speed, distance * 5)
            angular_vel = heading_error * 0.5
            
            self.send_velocity(linear_vel, angular_vel)
            self.fine_positioning_count = 0

    def handle_repositioning(self):
        """재접근 모드"""
        self.repositioning_attempts += 1
        
        if self.repositioning_attempts > self.max_repositioning_attempts:
            self.get_logger().error('❌ 재접근 시도 한계 초과! 다음 목표점으로 이동')
            self.current_mission_index += 1
            self.set_next_target()
            return
        
        self.get_logger().warn(f'🔄 재접근 모드 (시도 {self.repositioning_attempts}/{self.max_repositioning_attempts})')
        
        # 목표 방향 재계산
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        self.target_yaw = math.atan2(dy, dx)
        
        # 회전 상태로 전환하여 재시작
        self.change_state(RobotState.ROTATING)

    def handle_stop(self):
        """정지 제어"""
        self.send_velocity(0.0, 0.0)
        self.get_logger().info('🛑 로봇 정지 상태')

    def send_velocity(self, linear, angular):
        """속도 명령 전송"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)

    def change_state(self, new_state):
        """상태 변경"""
        self.current_state = new_state
        
        # 상태 퍼블리시
        state_msg = String()
        state_msg.data = new_state.value
        self.state_publisher.publish(state_msg)
        
        self.get_logger().info(f'🔄 상태 변경: {new_state.value}')

    def normalize_angle(self, angle):
        """각도 정규화 (-π ~ π)"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = NavigationController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\n⚠️  키보드 인터럽트 감지됨. 노드를 종료합니다.')
    except Exception as e:
        print(f'❌ 예상치 못한 오류: {e}')
    finally:
        if 'controller' in locals():
            controller.send_velocity(0.0, 0.0)  # 안전 정지
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
