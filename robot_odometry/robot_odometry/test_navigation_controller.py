#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
import threading
from enum import Enum

class RobotState(Enum):
    IDLE = "IDLE"
    ROTATING = "ROTATING"
    MOVING_FORWARD = "MOVING_FORWARD"
    FINE_POSITIONING = "FINE_POSITIONING"
    WAITING_FOR_INPUT = "WAITING_FOR_INPUT"
    MISSION_RUNNING = "MISSION_RUNNING"
    STOPPED = "STOPPED"

class TestNavigationController(Node):
    def __init__(self):
        super().__init__('test_navigation_controller')
        
        # 구독자 설정 - IMU 직접 사용
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',  # myAHRS+ 직접 사용
            self.imu_callback,
            10)
        
        # 위치는 EKF에서 가져오기 (x, y만)
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
        
        # 위치는 EKF, 각도는 IMU 직접 사용
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw_rad = 0.0
        self.current_yaw_deg = 0.0
        
        # 시작 위치 저장
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw_deg = 0.0
        
        # 목표 설정
        self.target_yaw_deg = 0.0
        self.target_yaw_rad = 0.0
        self.target_distance = 0.0
        self.traveled_distance = 0.0
        
        # 제어 파라미터
        self.position_tolerance = 0.01
        self.fine_tolerance = 0.02
        self.angle_tolerance = math.radians(0.8)  # 0.8도 허용 오차
        
        self.max_speed = 0.2
        self.angular_speed = 0.3
        self.fine_speed = 0.05
        
        self.linear_kp = 1.0
        
        # PI 제어 파라미터 (각도 제어용)
        self.angular_kp = 1.2  # 비례 게인
        self.angular_ki = 0.01  # 적분 게인
        
        # PI 제어 변수
        self.integral_error = 0.0  # 적분 누적 오차
        self.max_integral = math.radians(10)  # 적분 windup 방지 (10도)
        self.prev_time = None
        
        # 정사각형 미션 설정 추가
        self.square_size = 1.5  # 1.5m 정사각형
        # self.mission_steps = [
        #     # (action_type, value, description)
        #     ("rotate", 90, "동쪽 방향으로 회전"),
        #     ("forward", self.square_size, "동쪽으로 직진"),
        #     ("rotate", 180, "남쪽 방향으로 회전"), 
        #     ("forward", self.square_size, "남쪽으로 직진"),
        #     ("rotate", 270, "서쪽 방향으로 회전"),
        #     ("forward", self.square_size, "서쪽으로 직진"),
        #     ("rotate", 0, "북쪽 방향으로 회전"),
        #     ("forward", self.square_size, "북쪽으로 직진 (원점 복귀)")
        # ]
        self.mission_steps = [
            # (action_type, value, description)
            ("rotate", -166.0, "서쪽으로 회전"),
            ("forward", 2.5, "서쪽으로 직진"),
            
            ("rotate", 113.0, "북쪽 으로 회전"), 
            ("forward", 1.5, "북쪽으로 직진"),
            
            ("rotate", 10.2, "동쪽으로 회전"),
            ("forward", 2.5, "동쪽으로 직진"),
            
            ("rotate", -74.5, "남쪽 으로 회전"), 
            ("forward", 1.5, "남쪽으로 직진"),
            # ("rotate", 270, "서쪽 방향으로 회전"),
            # ("forward", self.square_size, "서쪽으로 직진"),
            # ("rotate", 0, "북쪽 방향으로 회전"),
            # ("forward", self.square_size, "북쪽으로 직진 (원점 복귀)")
        ]
        self.current_step_index = 0
        self.mission_active = False
        self.mission_start_time = None
        self.step_start_time = None
        self.step_completion_times = []
        
        # 상태 변수
        self.imu_data_received = False
        self.odom_data_received = False
        self.fine_positioning_count = 0
        self.position_stable_threshold = 10
        
        # 타이머
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🧭 PI 제어 Navigation Controller 시작됨 (정사각형 미션 지원)')
        self.get_logger().info('📡 센서 데이터 대기 중...')
        
        # 초기화 타이머
        self.init_timer = self.create_timer(3.0, self.initialize_robot)
        
        # 사용자 입력 스레드
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()

    def get_absolute_yaw_deg(self, qx, qy, qz, qw):
        """myAHRS+ 쿼터니언에서 절대 각도 계산"""
        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 
                           1.0 - 2.0 * (qy * qy + qz * qz))
        yaw_deg = math.degrees(yaw_rad)
        
        if yaw_deg < 0:
            yaw_deg += 360
            
        return yaw_deg, yaw_rad

    def imu_callback(self, msg):
        """IMU 데이터 직접 사용 (각도만)"""
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # 절대 각도 계산
        self.current_yaw_deg, self.current_yaw_rad = self.get_absolute_yaw_deg(qx, qy, qz, qw)
        self.imu_data_received = True

    def odometry_callback(self, msg):
        """EKF에서 위치만 가져오기"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_data_received = True

    @property
    def data_received(self):
        return self.imu_data_received and self.odom_data_received

    def initialize_robot(self):
        """로봇 초기화"""
        if self.data_received:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw_deg = self.current_yaw_deg
            
            self.get_logger().info('✅ 로봇 초기화 완료!')
            self.get_logger().info(f'📍 시작 위치: ({self.current_x:.3f}, {self.current_y:.3f})')
            self.get_logger().info(f'🧭 시작 방향: {self.current_yaw_deg:.1f}° (IMU 직접값)')
            
            self.change_state(RobotState.WAITING_FOR_INPUT)
            self.init_timer.cancel()
        else:
            self.get_logger().warn('⚠️  센서 데이터 대기 중...')

    def user_input_loop(self):
        """사용자 입력 처리"""
        time.sleep(4)
        
        while rclpy.ok():
            try:
                if self.current_state == RobotState.WAITING_FOR_INPUT:
                    self.print_status()
                    self.get_user_command()
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'입력 처리 오류: {e}')

    def print_status(self):
        """현재 상태 출력"""
        print("\n" + "="*60)
        print("🧭 PI 제어 네비게이션 컨트롤러 (정사각형 미션)")
        print("="*60)
        print(f"📍 현재 위치: ({self.current_x:.3f}, {self.current_y:.3f}) [EKF]")
        print(f"🧭 현재 방향: {self.current_yaw_deg:.1f}° [IMU 직접]")
        print(f"⚙️  제어 방식: PI 제어 (Kp={self.angular_kp}, Ki={self.angular_ki})")
        
        if self.mission_active:
            print(f"🎯 미션 진행중: {self.current_step_index}/{len(self.mission_steps)} 단계")
        
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        total_distance = math.sqrt(dx*dx + dy*dy)
        print(f"📏 시작점으로부터: {total_distance:.3f}m")
        print("-"*60)

    def get_user_command(self):
        """사용자 명령 입력"""
        print("명령어:")
        print("  a <각도>  : 절대 각도로 회전 (예: a 90) [PI 제어]")
        print("  f <거리>  : 직진 (예: f 1.0)")
        print("  m        : 정사각형 미션 시작 🟦")
        print("  s        : 정지")
        print("  q        : 종료")
        print("-"*60)
        print("💡 절대 각도: 0°=북쪽, 90°=동쪽, 180°=남쪽, 270°=서쪽")
        print("🎯 PI 제어로 정착 오차 없는 정밀 회전")
        print("-"*60)
        
        try:
            command = input("명령 입력: ").strip().lower()
            
            if command == 'q':
                rclpy.shutdown()
                return
            elif command == 's':
                self.stop_mission()
                self.change_state(RobotState.STOPPED)
                return
            elif command == 'm':
                self.start_square_mission()
                return
            elif command.startswith('a '):
                try:
                    angle_deg = float(command.split()[1])
                    self.execute_absolute_rotation(angle_deg)
                except (IndexError, ValueError):
                    print("❌ 잘못된 명령! 예: a 90")
            elif command.startswith('f '):
                try:
                    distance = float(command.split()[1])
                    self.execute_forward(distance)
                except (IndexError, ValueError):
                    print("❌ 잘못된 명령! 예: f 1.0")
            else:
                print("❌ 알 수 없는 명령!")
                
        except KeyboardInterrupt:
            rclpy.shutdown()

    def start_square_mission(self):
        """정사각형 미션 시작"""
        if self.mission_active:
            print("⚠️  이미 미션이 진행 중입니다!")
            return
            
        self.mission_active = True
        self.mission_start_time = time.time()
        self.current_step_index = 0
        self.step_completion_times = []
        
        self.get_logger().info('🟦 정사각형 미션 시작!')
        self.get_logger().info(f'📐 정사각형 크기: {self.square_size}m x {self.square_size}m')
        self.get_logger().info('=' * 60)
        
        self.execute_next_step()

    def stop_mission(self):
        """미션 중지"""
        if self.mission_active:
            self.mission_active = False
            self.send_velocity(0.0, 0.0)
            self.get_logger().info('🛑 미션이 중지되었습니다!')

    def execute_next_step(self):
        """다음 단계 실행"""
        if not self.mission_active:
            return
            
        if self.current_step_index >= len(self.mission_steps):
            self.complete_mission()
            return
        
        action_type, value, description = self.mission_steps[self.current_step_index]
        self.step_start_time = time.time()
        
        self.get_logger().info(f'🎯 단계 {self.current_step_index + 1}/{len(self.mission_steps)}: {description}')
        
        if action_type == "rotate":
            self.get_logger().info(f'회전중 입니다.')
            self.execute_absolute_rotation(value)
        elif action_type == "forward":
            self.get_logger().info(f'직진중 입니다.')
            self.execute_forward(value)
        
        # self.change_state(RobotState.MISSION_RUNNING)

    def complete_mission(self):
        """미션 완료"""
        total_time = time.time() - self.mission_start_time
        
        self.get_logger().info('🏁 정사각형 미션 완료!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'⏱️  총 소요 시간: {total_time:.2f}초')
        self.get_logger().info('📊 단계별 소요 시간:')
        
        for step_time in self.step_completion_times:
            self.get_logger().info(f'   {step_time}')
        
        # 최종 위치 오차 계산
        final_error_x = abs(self.current_x - self.start_x)
        final_error_y = abs(self.current_y - self.start_y)
        final_error_total = math.sqrt(final_error_x**2 + final_error_y**2)
        
        self.get_logger().info(f'🎯 최종 위치 오차: X={final_error_x:.3f}m, Y={final_error_y:.3f}m, 총={final_error_total:.3f}m')
        self.get_logger().info(f'🧭 최종 방향 오차: {abs(self.current_yaw_deg - self.start_yaw_deg):.2f}°')
        self.get_logger().info('=' * 60)
        
        self.mission_active = False
        self.change_state(RobotState.WAITING_FOR_INPUT)

    def execute_absolute_rotation(self, target_angle_deg):
        """절대 각도로 회전 (PI 제어 초기화)"""
        target_angle_deg = target_angle_deg % 360
        self.target_yaw_deg = target_angle_deg
        self.target_yaw_rad = math.radians(target_angle_deg)
        
        # PI 제어 변수 초기화
        self.reset_pi_controller()
        
        self.get_logger().info(f'🧭 PI 제어 회전 시작: {target_angle_deg:.1f}° (IMU 기준)')
        self.get_logger().info(f'{self.target_yaw_rad:.3f} rad (라디안)')
        
        self.change_state(RobotState.ROTATING)

    def reset_pi_controller(self):
        """PI 제어기 초기화"""
        self.integral_error = 0.0
        self.prev_time = time.time()
        self.get_logger().info('🔄 PI 제어기 초기화 완료')

    def execute_forward(self, distance):
        """직진 실행"""
        self.target_distance = distance
        self.traveled_distance = 0.0
        self.start_x = self.current_x
        self.start_y = self.current_y
        
        self.get_logger().info(f'🚗 직진: {distance:.2f}m')
        self.change_state(RobotState.MOVING_FORWARD)

    def control_loop(self):
        """제어 루프"""
        if not self.data_received:
            return
        
        if self.current_state == RobotState.ROTATING:
            self.handle_pi_rotation()
        elif self.current_state == RobotState.MOVING_FORWARD:
            self.handle_forward_movement()
        elif self.current_state == RobotState.STOPPED:
            self.handle_stop()

    def handle_pi_rotation(self):
        """PI 제어 기반 회전 제어"""
        self.get_logger().info('🔄 PI 제어 회전 중...')
        # 현재 시간
        current_time = time.time()
        
        # 각도 오차 계산
        angle_error_deg = self.target_yaw_deg - self.current_yaw_deg
        self.get_logger().info(f'각도 오차: {angle_error_deg}')
        # 최단 경로 계산
        if angle_error_deg > 180:
            angle_error_deg -= 360
        elif angle_error_deg < -180:
            angle_error_deg += 360
            
        angle_error_rad = math.radians(angle_error_deg)
        
        # 목표 도달 확인
        if abs(angle_error_rad) < self.angle_tolerance:
            self.send_velocity(0.0, 0.0)
            
            # 단계 완료 시간 기록 (미션 중일 때만)
            if self.mission_active and self.step_start_time is not None:
                step_time = time.time() - self.step_start_time
                self.step_completion_times.append(f"회전 단계 {self.current_step_index + 1}: {step_time:.2f}초")
                self.get_logger().info(f'✅ 회전 완료! 최종 오차: {angle_error_deg:.2f}° (소요시간: {step_time:.2f}초)')
                
                # 다음 단계로 진행
                self.current_step_index += 1
                self.execute_next_step()
            else:
                # 수동 명령일 때
                self.get_logger().info(f'✅ PI 제어 회전 완료! 최종 오차: {angle_error_deg:.2f}°')
                self.change_state(RobotState.WAITING_FOR_INPUT)
            return
        
        # 시간 간격 계산 (dt)
        if self.prev_time is not None:
            dt = current_time - self.prev_time
        else:
            dt = 0.1  # 첫 번째 실행 시 기본값
            self.prev_time = current_time
        
        # PI 제어 계산
        if dt > 0:
            # P항 (비례 제어)
            p_term = self.angular_kp * angle_error_rad
            
            # I항 (적분 제어)
            self.integral_error += angle_error_rad * dt
            
            # 적분 windup 방지
            if self.integral_error > self.max_integral:
                self.integral_error = self.max_integral
            elif self.integral_error < -self.max_integral:
                self.integral_error = -self.max_integral
            
            i_term = self.angular_ki * self.integral_error
            
            # PI 제어 신호 계산
            angular_vel = p_term + i_term
            
            # 속도 제한
            angular_vel = max(-self.angular_speed, min(self.angular_speed, angular_vel))
            
            # 제어 신호 적용
            self.send_velocity(0.0, angular_vel)
            
            # 상세 로그 출력
            self.get_logger().info(
                f'🎯 PI회전: 현재 {self.current_yaw_deg:.1f}°, '
                f'목표 {self.target_yaw_deg:.1f}°, 오차 {angle_error_deg:.2f}°, '
                f'P={p_term:.3f}, I={i_term:.3f}, 적분누적={math.degrees(self.integral_error):.2f}°'
            )
        
        # 시간 업데이트
        self.prev_time = current_time

    def handle_forward_movement(self):
        """직진 제어"""
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        self.traveled_distance = math.sqrt(dx*dx + dy*dy)
        
        remaining_distance = self.target_distance - self.traveled_distance
        
        if remaining_distance <= self.position_tolerance:
            self.send_velocity(0.0, 0.0)
            
            # 단계 완료 시간 기록 (미션 중일 때만)
            if self.mission_active and self.step_start_time is not None:
                step_time = time.time() - self.step_start_time
                self.step_completion_times.append(f"직진 단계 {self.current_step_index + 1}: {step_time:.2f}초")
                self.get_logger().info(f'✅ 직진 완료! (소요시간: {step_time:.2f}초)')
                
                # 다음 단계로 진행
                self.current_step_index += 1
                self.execute_next_step()
            else:
                # 수동 명령일 때
                self.get_logger().info('✅ 직진 완료!')
                self.change_state(RobotState.WAITING_FOR_INPUT)
            return
        
        if remaining_distance < 0.2:
            linear_vel = max(self.fine_speed, remaining_distance * 0.5)
        else:
            linear_vel = self.max_speed
        
        self.send_velocity(linear_vel, 0.0)
        
        self.get_logger().info(
            f'🚗 직진중: {self.traveled_distance:.3f}m / {self.target_distance:.3f}m, '
            f'방향 {self.current_yaw_deg:.1f}°'
        )

    def handle_stop(self):
        self.send_velocity(0.0, 0.0)

    def send_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)

    def change_state(self, new_state):
        self.get_logger().info(f'🔄 상태 변경: {self.current_state.value} -> {new_state.value}')
         
        self.current_state = new_state
        state_msg = String()
        state_msg.data = new_state.value
        self.state_publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = TestNavigationController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\n🛑 PI 제어 컨트롤러 종료')
    finally:
        if 'controller' in locals():
            controller.send_velocity(0.0, 0.0)
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
