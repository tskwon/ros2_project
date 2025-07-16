#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
from enum import Enum
from collections import deque

class RobotState(Enum):
    IDLE = 0
    SEARCHING = 1
    APPROACHING = 2
    ALIGNING = 3
    FINISHED = 4
    SEARCH_TIMEOUT = 5
    WAITING_NEXT_SEQUENCE = 6  # 새로 추가: 다음 시퀀스 대기

class LogisticsCommand:
    """물류 명령 클래스"""
    def __init__(self, target_id, target_distance):
        self.target_id = target_id
        self.target_distance = target_distance
    
    def __str__(self):
        return f"ID={self.target_id}, Distance={self.target_distance}cm"

class LogisticsRobotController(Node):
    def __init__(self):
        super().__init__('logistics_robot_controller')
        
        # 기존 구독자들
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco/marker_pose',
            self.pose_callback,
            10)
        
        # 물류 명령 구독 (0-7 번호)
        self.logistics_command_sub = self.create_subscription(
            Int32,
            '/logistics/command',
            self.logistics_command_callback,
            10)
        
        # 엔코더 값 구독
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/encoder_values',
            self.encoder_callback,
            10)
        
        # 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 로봇 파라미터
        self.wheel_radius = 0.103
        self.wheel_separation = 0.503
        self.gear_ratio = 1
        self.poles = 30
        
        # 엔코더 관련 변수
        self.encoder_ppr = self.poles * self.gear_ratio * 4.0
        self.left_encoder = 0
        self.right_encoder = 0
        self.search_start_left_encoder = 0
        self.search_start_right_encoder = 0
        self.current_rotation_angle = 0.0
        self.target_rotation_angle = math.pi / 2.8  # 60도
        
        # 제어 파라미터
        self.target_distance = None
        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.04
        self.lateral_tolerance = 0.1
        
        # 속도 제한
        self.max_linear_vel = 0.28
        self.max_angular_vel = 0.7
        self.search_angular_vel = 0.3
        
        # PI 제어 게인
        self.kp_distance = 0.4
        self.ki_distance = 0.01
        self.kp_lateral = 0.3
        self.ki_lateral = 0.007
        self.kp_angular = 0.5
        self.ki_angular = 0.08
        
        # 적분항 관리
        self.integral_distance = 0.0
        self.integral_lateral = 0.0
        self.integral_angular = 0.0
        self.integral_limit_distance = 0.5
        self.integral_limit_lateral = 0.3
        self.integral_limit_angular = 0.5
        
        # 시간 관리
        self.last_control_time = self.get_clock().now()
        
        # 상태 관리
        self.state = RobotState.IDLE
        self.last_pose_time = self.get_clock().now()
        self.pose_timeout = 3.0
        
        # 타겟 ID 관리
        self.target_id = None
        self.current_marker_id = None
        
        # 마커 정보
        self.marker_distance = None
        self.marker_lateral = None
        self.marker_pitch = None
        self.current_pose = None
        
        # 제어 타이머
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # 상태 변경 디바운싱
        self.state_change_counter = 0
        self.state_change_threshold = 8
        
        # 미션 관리
        self.mission_completed = False
        self.current_mission_id = None
        self.current_mission_distance = None
        
        # 검색 타임아웃 관리
        self.search_timeout_counter = 0
        
        # 물류 시스템 관리
        self.command_queue = deque()  # 명령 큐
        self.current_logistics_command = None
        self.logistics_mission_active = False
        self.sequence_delay_counter = 0
        self.sequence_delay_threshold = 20  # 2초 대기 (0.1초 * 20)
        
        # 물류 명령 정의
        self.logistics_commands = {
            0: [LogisticsCommand(0, 200)],  # 단순 이동
            1: [LogisticsCommand(0, 160)],  # 단순 이동
            2: [LogisticsCommand(0, 130)],  # 단순 이동
            3: [LogisticsCommand(0, 100)],  # 단순 이동
            4: [  # 복합 이동: 0→70cm → 1→70cm → 2→200cm
                LogisticsCommand(0, 70),
                LogisticsCommand(1, 70),
                LogisticsCommand(2, 200)
            ],
            5: [LogisticsCommand(2, 160)],  # 단순 이동
            6: [LogisticsCommand(2, 130)],  # 단순 이동
            7: [  # 복합 이동: 2→70cm → 3→70cm → 0→230cm
                LogisticsCommand(2, 70),
                LogisticsCommand(3, 70),
                LogisticsCommand(0, 230)
            ]
        }
        
        self.get_logger().info('=== LOGISTICS ROBOT CONTROLLER STARTED ===')
        self.get_logger().info('Logistics commands available: 0-7')
        self.get_logger().info('Send command via: ros2 topic pub /logistics/command std_msgs/msg/Int32 "{data: X}"')
        self.get_logger().info('Robot in IDLE state - waiting for logistics command')
        
    def logistics_command_callback(self, msg):
        """물류 명령 수신 처리"""
        command_num = msg.data
        
        if command_num < 0 or command_num > 7:
            self.get_logger().warn(f'Invalid logistics command: {command_num}. Valid range: 0-7')
            return
        
        # 기존 명령 큐 클리어
        self.command_queue.clear()
        
        # 새로운 명령 시퀀스 로드
        command_sequence = self.logistics_commands[command_num]
        self.command_queue.extend(command_sequence)
        
        self.get_logger().info(f'=== LOGISTICS COMMAND {command_num} RECEIVED ===')
        self.get_logger().info(f'Command sequence ({len(command_sequence)} steps):')
        for i, cmd in enumerate(command_sequence):
            self.get_logger().info(f'  Step {i+1}: {cmd}')
        
        # 물류 미션 시작
        self.logistics_mission_active = True
        self.start_next_sequence()
    
    def start_next_sequence(self):
        """다음 시퀀스 시작"""
        if not self.command_queue:
            # 모든 시퀀스 완료
            self.complete_logistics_mission()
            return
        
        # 다음 명령 가져오기
        next_command = self.command_queue.popleft()
        self.current_logistics_command = next_command
        
        # 타겟 설정
        self.target_id = next_command.target_id
        self.target_distance = next_command.target_distance / 100.0  # cm to m
        
        self.get_logger().info(f'=== STARTING SEQUENCE ===')
        self.get_logger().info(f'Target: {next_command}')
        self.get_logger().info(f'Remaining steps: {len(self.command_queue)}')
        
        # 미션 시작
        self.start_mission()
    
    def complete_logistics_mission(self):
        """물류 미션 완료"""
        self.logistics_mission_active = False
        self.current_logistics_command = None
        self.target_id = None
        self.target_distance = None
        self.state = RobotState.IDLE
        
        self.get_logger().info('=== LOGISTICS MISSION COMPLETED ===')
        self.get_logger().info('All sequences finished successfully!')
        self.get_logger().info('Robot returned to IDLE state')
        self.get_logger().info('Send new logistics command (0-7) to start new mission')
    
    def encoder_callback(self, msg):
        """엔코더 값 수신 콜백"""
        if len(msg.data) >= 2:
            self.left_encoder = msg.data[0]
            self.right_encoder = msg.data[1]
            
            if self.state == RobotState.SEARCHING:
                self.calculate_rotation_angle()
    
    def calculate_rotation_angle(self):
        """엔코더 값을 기반으로 회전 각도 계산"""
        delta_left = self.left_encoder - self.search_start_left_encoder
        delta_right = self.right_encoder - self.search_start_right_encoder
        
        left_distance = (delta_left / self.encoder_ppr) * 2.0 * math.pi * self.wheel_radius
        right_distance = (delta_right / self.encoder_ppr) * 2.0 * math.pi * self.wheel_radius
        
        self.current_rotation_angle = abs((right_distance - left_distance) / self.wheel_separation)
        
        if self.current_rotation_angle > 0.1:
            self.get_logger().info(f'Rotation angle: {self.current_rotation_angle:.3f}rad '
                                 f'({math.degrees(self.current_rotation_angle):.1f}°)')
    
    def reset_integrals(self):
        """적분항 초기화"""
        self.integral_distance = 0.0
        self.integral_lateral = 0.0
        self.integral_angular = 0.0
        self.last_control_time = self.get_clock().now()
        self.get_logger().info('PI control integrals reset')
    
    def normalize_angle(self, angle):
        """각도 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def start_mission(self):
        """미션 시작"""
        self.state = RobotState.SEARCHING
        self.state_change_counter = 0
        self.mission_completed = False
        self.search_timeout_counter = 0
        
        # 검색 시작 엔코더 값 저장
        self.search_start_left_encoder = self.left_encoder
        self.search_start_right_encoder = self.right_encoder
        self.current_rotation_angle = 0.0
        
        # 현재 미션 파라미터 저장
        self.current_mission_id = self.target_id
        self.current_mission_distance = self.target_distance
        
        # 적분항 초기화
        self.reset_integrals()
        
        self.get_logger().info(f'Starting search for marker {self.target_id}...')
    
    def pose_callback(self, msg):
        """ArUco 마커 포즈 수신"""
        if self.state == RobotState.IDLE or self.target_id is None or self.target_distance is None:
            return
        
        # 마커 ID 추출
        try:
            if hasattr(msg, 'header') and msg.header.frame_id.startswith('marker_'):
                marker_id = int(msg.header.frame_id.split('_')[1])
                self.current_marker_id = marker_id
            else:
                self.current_marker_id = self.target_id
        except:
            self.current_marker_id = self.target_id
        
        # 타겟 ID와 일치하는 마커만 처리
        if self.current_marker_id != self.target_id:
            return
        
        self.current_pose = msg
        self.last_pose_time = self.get_clock().now()
        
        # 위치 정보 추출
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        if z <= 0.05 or z > 15.0:
            self.get_logger().warn(f'Invalid pose data: distance={z:.3f}m. Ignoring...')
            return
        
        # 쿼터니언을 오일러 각도로 변환
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        sin_pitch = 2 * (qw * qy - qz * qx)
        cos_pitch = 1 - 2 * (qx * qx + qy * qy)
        self.marker_pitch = math.atan2(sin_pitch, cos_pitch)
        self.marker_pitch = self.normalize_angle(self.marker_pitch)
        
        self.marker_distance = z
        self.marker_lateral = x
        
        # 상태 업데이트
        if self.state in [RobotState.SEARCHING, RobotState.SEARCH_TIMEOUT]:
            if (self.marker_distance is not None and self.marker_lateral is not None and 
                self.marker_pitch is not None and self.target_distance is not None):
                self.state = RobotState.APPROACHING
                self.state_change_counter = 0
                self.reset_integrals()
                self.get_logger().info(f'Target marker {self.target_id} detected! Starting approach...')
    
    def control_loop(self):
        """메인 제어 루프"""
        if self.state == RobotState.IDLE:
            self.stop_robot()
            return
        
        if self.target_id is None or self.target_distance is None:
            if self.state != RobotState.IDLE:
                self.get_logger().info('Missing target information, returning to IDLE state')
                self.state = RobotState.IDLE
                self.state_change_counter = 0
                self.reset_integrals()
            self.stop_robot()
            return
        
        current_time = self.get_clock().now()
        
        # 마커 검출 시간 체크
        if self.state not in [RobotState.FINISHED, RobotState.SEARCH_TIMEOUT, RobotState.WAITING_NEXT_SEQUENCE]:
            if (current_time - self.last_pose_time).nanoseconds / 1e9 > self.pose_timeout:
                if self.state != RobotState.SEARCHING:
                    self.get_logger().warn(f'Target marker {self.target_id} lost! Searching again...')
                    self.state = RobotState.SEARCHING
                    self.state_change_counter = 0
                    self.search_start_left_encoder = self.left_encoder
                    self.search_start_right_encoder = self.right_encoder
                    self.current_rotation_angle = 0.0
                    self.reset_integrals()
                self.search_behavior()
                return
        
        # 상태별 제어
        if self.state == RobotState.SEARCHING:
            self.search_behavior()
        elif self.state == RobotState.APPROACHING:
            self.approach_behavior()
        elif self.state == RobotState.ALIGNING:
            self.align_behavior()
        elif self.state == RobotState.FINISHED:
            self.finish_behavior()
        elif self.state == RobotState.SEARCH_TIMEOUT:
            self.search_timeout_behavior()
        elif self.state == RobotState.WAITING_NEXT_SEQUENCE:
            self.waiting_next_sequence_behavior()
    
    def search_behavior(self):
        """60도 제한 마커 검색"""
        if self.target_id is None or self.target_distance is None:
            self.stop_robot()
            return
        
        if self.current_rotation_angle >= self.target_rotation_angle:
            self.get_logger().info(f'60-degree rotation completed ({math.degrees(self.current_rotation_angle):.1f}°)')
            self.get_logger().info('Marker not found in 60-degree search. Stopping...')
            # self.state = RobotState.FINISHED
            self.sequence_delay_counter = 0
            self.stop_robot()
            return
    
        twist = Twist()
        twist.angular.z = -self.search_angular_vel
        self.cmd_vel_pub.publish(twist)
        
        if self.state_change_counter % 10 == 0:
            remaining_angle = self.target_rotation_angle - self.current_rotation_angle
            self.get_logger().info(f'Searching... Rotated: {math.degrees(self.current_rotation_angle):.1f}°, '
                                f'Remaining: {math.degrees(remaining_angle):.1f}°')
        self.state_change_counter += 1
    
    def search_timeout_behavior(self):
        """검색 실패 처리"""
        self.stop_robot()
        
        if self.search_timeout_counter == 0:
            self.get_logger().info('=== SEARCH TIMEOUT ===')
            self.get_logger().info(f'Marker {self.target_id} not found in 60-degree search')
            
            if self.logistics_mission_active:
                self.get_logger().info('Logistics mission failed - marker not found')
                self.complete_logistics_mission()
            else:
                self.get_logger().info('Send new logistics command to retry')
        
        self.search_timeout_counter += 1
        
        if self.search_timeout_counter % 100 == 0:
            self.get_logger().info('Robot waiting for new command...')
    
    def approach_behavior(self):
        """마커 접근 행동"""
        if self.current_pose is None or self.marker_distance is None or self.target_distance is None:
            self.stop_robot()
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_time
        
        if dt > 0.5:
            self.reset_integrals()
            return
        
        twist = Twist()
        
        # 거리 제어
        distance_error = self.marker_distance - self.target_distance
        
        self.integral_distance += distance_error * dt
        self.integral_distance = max(-self.integral_limit_distance, 
                                   min(self.integral_limit_distance, self.integral_distance))
        
        if abs(distance_error) < self.distance_tolerance:
            self.state_change_counter += 1
            if self.state_change_counter >= self.state_change_threshold:
                self.state = RobotState.ALIGNING
                self.state_change_counter = 0
                self.reset_integrals()
                self.get_logger().info('Target distance reached! Starting alignment...')
                return
        else:
            self.state_change_counter = 0
        
        linear_vel = (self.kp_distance * distance_error + 
                     self.ki_distance * self.integral_distance)
        
        if abs(linear_vel) < 0.02:
            linear_vel = 0.02 if linear_vel > 0 else -0.02
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        
        # 좌우 정렬
        lateral_error = self.marker_lateral
        
        self.integral_lateral += lateral_error * dt
        self.integral_lateral = max(-self.integral_limit_lateral, 
                                  min(self.integral_limit_lateral, self.integral_lateral))
        
        angular_vel = -(self.kp_lateral * lateral_error + 
                       self.ki_lateral * self.integral_lateral)
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        if self.marker_distance < 0.15:
            twist.linear.x = -0.05
            twist.angular.z = angular_vel * 0.5
        else:
            twist.linear.x = linear_vel 
            twist.angular.z = angular_vel
        
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(f'Approaching - Distance: {self.marker_distance:.3f}m, '
                             f'Lateral: {self.marker_lateral:.3f}m, '
                             f'Linear: {twist.linear.x:.3f}, Angular: {twist.angular.z:.3f}')
    
    def align_behavior(self):
        """마커 정렬 행동 - 2초 타임아웃 추가"""
        if (self.current_pose is None or self.marker_distance is None or 
            self.marker_lateral is None or self.marker_pitch is None or 
            self.target_distance is None):
            self.stop_robot()
            return
        
        # 정렬 시작 시간 체크
        if not hasattr(self, 'align_start_time'):
            self.align_start_time = self.get_clock().now()
            self.get_logger().info('Starting alignment with 2-second timeout...')
        
        current_time = self.get_clock().now()
        align_duration = (current_time - self.align_start_time).nanoseconds / 1e9
        
        # 2초 타임아웃 체크
        if align_duration > 2.0:
            self.get_logger().info('Alignment timeout (2s) reached - forcing completion!')
            delattr(self, 'align_start_time')
            self.state = RobotState.FINISHED
            self.state_change_counter = 0
            self.reset_integrals()
            return
        
        dt = (current_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_time
        
        if dt > 0.5:
            self.reset_integrals()
            return
        
        twist = Twist()
        
        # 거리 재확인
        distance_error = self.marker_distance - self.target_distance
        if abs(distance_error) > self.distance_tolerance * 3:
            self.state = RobotState.APPROACHING
            self.state_change_counter = 0
            delattr(self, 'align_start_time')  # 타이머 리셋
            self.reset_integrals()
            self.get_logger().info('Distance too far, returning to approach mode...')
            return
        
        # 정렬 상태 확인
        lateral_error = self.marker_lateral
        angle_error = self.marker_pitch
        
        lateral_aligned = abs(lateral_error) <= self.lateral_tolerance
        angle_aligned = abs(angle_error) <= self.angle_tolerance
        distance_ok = abs(distance_error) <= self.distance_tolerance
        
        if lateral_aligned and angle_aligned and distance_ok:
            self.state_change_counter += 1
            if self.state_change_counter >= self.state_change_threshold:
                self.get_logger().info('Perfect alignment completed!')
                delattr(self, 'align_start_time')
                self.state = RobotState.FINISHED
                self.state_change_counter = 0
                self.reset_integrals()
                return
        else:
            self.state_change_counter = 0
            
            if not lateral_aligned:
                self.integral_lateral += lateral_error * dt
                self.integral_lateral = max(-self.integral_limit_lateral, 
                                        min(self.integral_limit_lateral, self.integral_lateral))
                
                angular_vel = -(self.kp_lateral * lateral_error * 0.4 + 
                            self.ki_lateral * self.integral_lateral * 0.4)
                angular_vel = max(-self.max_angular_vel * 0.4, min(self.max_angular_vel * 0.4, angular_vel))
                twist.angular.z = angular_vel
                
            elif not angle_aligned:
                self.integral_angular += angle_error * dt
                self.integral_angular = max(-self.integral_limit_angular, 
                                        min(self.integral_limit_angular, self.integral_angular))
                
                angular_vel = -(self.kp_angular * angle_error + 
                            self.ki_angular * self.integral_angular)
                angular_vel = max(-self.max_angular_vel * 0.4, min(self.max_angular_vel * 0.4, angular_vel))
                twist.angular.z = angular_vel
        
        # 미세 거리 조정
        if abs(distance_error) > self.distance_tolerance * 0.5:
            self.integral_distance += distance_error * dt
            self.integral_distance = max(-self.integral_limit_distance * 0.1, 
                                    min(self.integral_limit_distance * 0.1, self.integral_distance))
            
            linear_vel = (self.kp_distance * distance_error * 1.1 + 
                        self.ki_distance * self.integral_distance * 1.1)
            linear_vel = max(-0.03, min(0.03, linear_vel))
            twist.linear.x = linear_vel
        
        self.cmd_vel_pub.publish(twist)
        
        remaining_time = 2.0 - align_duration
        self.get_logger().info(f'Aligning - Lateral: {self.marker_lateral:.3f}m, '
                            f'Pitch: {math.degrees(self.marker_pitch):.1f}°, '
                            f'Distance: {self.marker_distance:.3f}m, '
                            f'L_OK: {lateral_aligned}, A_OK: {angle_aligned}, D_OK: {distance_ok}, '
                            f'Time: {remaining_time:.1f}s')
        
    def finish_behavior(self):
        """시퀀스 완료 처리"""
        self.stop_robot()
        
        if not self.mission_completed:
            self.mission_completed = True
            self.get_logger().info('=== SEQUENCE COMPLETED ===')
            self.get_logger().info(f'Successfully reached marker {self.target_id} at {self.target_distance:.2f}m')
            
            if self.logistics_mission_active and self.command_queue:
                # 다음 시퀀스가 있으면 대기 상태로 전환
                self.state = RobotState.WAITING_NEXT_SEQUENCE
                self.sequence_delay_counter = 0
                self.get_logger().info('Waiting 2 seconds before next sequence...')
            else:
                # 모든 시퀀스 완료
                self.complete_logistics_mission()
    
    def waiting_next_sequence_behavior(self):
        """다음 시퀀스 대기 행동"""
        self.stop_robot()
        
        self.sequence_delay_counter += 1
        
        if self.sequence_delay_counter >= self.sequence_delay_threshold:
            self.get_logger().info('Starting next sequence...')
            self.start_next_sequence()
        elif self.sequence_delay_counter % 10 == 0:
            remaining_time = (self.sequence_delay_threshold - self.sequence_delay_counter) / 10.0
            self.get_logger().info(f'Next sequence in {remaining_time:.1f}s...')
    
    def stop_robot(self):
        """로봇 정지"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LogisticsRobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()