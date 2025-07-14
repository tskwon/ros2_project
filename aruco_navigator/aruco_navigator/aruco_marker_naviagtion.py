#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
from enum import Enum

class RobotState(Enum):
    IDLE = 0        # 새로 추가: 완전히 정지 상태
    SEARCHING = 1
    APPROACHING = 2
    ALIGNING = 3
    FINISHED = 4

class ArucoRobotController(Node):
    def __init__(self):
        super().__init__('aruco_robot_controller')
        
        # 구독자들
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco/marker_pose',
            self.pose_callback,
            10)
        
        # 타겟 ID 구독
        self.target_id_sub = self.create_subscription(
            Int32,
            '/aruco/target_id',
            self.target_id_callback,
            10)
        
        # 타겟 거리 구독
        self.target_distance_sub = self.create_subscription(
            Int32,
            '/aruco/target_distance',
            self.target_distance_callback,
            10)
        
        # 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 제어 파라미터
        self.target_distance = None  
        self.distance_tolerance = 0.05 
        self.angle_tolerance = 0.017  
        self.lateral_tolerance = 0.05
        self.final_pitch_tolerance = 0.0349  # 2도 (최종 pitch 정렬을 위한 허용 오차)
        
        # 속도 제한
        self.max_linear_vel = 0.28
        self.max_angular_vel = 0.7
        
        # PI 제어 게인
        self.kp_distance = 0.4
        self.ki_distance = 0.001
        
        self.kp_lateral = 0.35
        self.ki_lateral = 0.01
        
        self.kp_angular = 0.4
        self.ki_angular = 0.1
        
        # 적분항 관리
        self.integral_distance = 0.0
        self.integral_lateral = 0.0
        self.integral_angular = 0.0
        
        # 적분항 제한 (windup 방지)
        self.integral_limit_distance = 0.5
        self.integral_limit_lateral = 0.3
        self.integral_limit_angular = 0.3
        
        # 이전 시간 저장 (dt 계산용)
        self.last_control_time = self.get_clock().now()
        
        # 상태 관리
        self.state = RobotState.IDLE  # 초기 상태를 IDLE로 변경
        self.last_pose_time = self.get_clock().now()
        self.pose_timeout = 2.0
        
        # 타겟 ID 관리
        self.target_id = None
        self.current_marker_id = None
        
        # 마커 정보
        self.marker_distance = None
        self.marker_lateral = None
        self.marker_pitch = None  # yaw 대신 pitch 사용
        self.current_pose = None
        
        # 제어 타이머
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # 상태 변경 디바운싱을 위한 카운터
        self.state_change_counter = 0
        self.state_change_threshold = 10
        
        # 미션 완료 플래그 추가
        self.mission_completed = False
        
        # 현재 미션 파라미터 추적 (변경 감지용)
        self.current_mission_id = None
        self.current_mission_distance = None
        
        # 최종 pitch 정렬 상태
        self.final_pitch_aligned = False
        
        self.get_logger().info('ArUco Robot Controller with PI Control Started')
        self.get_logger().info('Robot in IDLE state - completely stopped')
        self.get_logger().info('Set target_id and target_distance to start mission')
        
    def reset_integrals(self):
        """적분항 초기화"""
        self.integral_distance = 0.0
        self.integral_lateral = 0.0
        self.integral_angular = 0.0
        self.last_control_time = self.get_clock().now()
        self.get_logger().info('PI control integrals reset')
        
    def target_distance_callback(self, msg):
        """타겟 거리 수신 (cm 단위)"""
        new_distance = msg.data / 100.0  # cm를 m로 변환
        
        # 유효한 거리인지 확인
        if new_distance <= 0.1 or new_distance > 5.0:
            self.get_logger().warn(f'Invalid target distance: {new_distance:.2f}m. Ignoring...')
            return
        
        # 같은 거리가 반복적으로 오는 것을 방지
        if self.target_distance is not None and abs(self.target_distance - new_distance) < 0.01:
            return
            
        self.target_distance = new_distance
        self.get_logger().info(f'Target distance set to: {self.target_distance:.2f}m ({msg.data}cm)')
        
        # 새로운 미션 시작 조건 확인 (IDLE 또는 FINISHED 상태에서)
        if self.target_id is not None and self.state in [RobotState.IDLE, RobotState.FINISHED]:
            self.start_mission()
        elif self.target_id is not None:
            self.get_logger().info(f'Mission parameters updated: ID={self.target_id}, Distance={self.target_distance:.2f}m')
        else:
            self.get_logger().info('Waiting for target ID...')
        
        # 미션 진행 중 거리 변경 처리
        if self.state in [RobotState.APPROACHING, RobotState.ALIGNING]:
            # 현재 미션과 다른 파라미터인 경우 새로운 미션 시작
            if (self.current_mission_distance is None or 
                abs(self.current_mission_distance - self.target_distance) > 0.01):
                self.start_mission()
            else:
                self.state = RobotState.APPROACHING
                self.state_change_counter = 0
                self.reset_integrals()  # 적분항 초기화
                self.get_logger().info('Distance changed during mission, restarting approach...')
        
    def target_id_callback(self, msg):
        """타겟 ID 수신"""
        if msg.data < 0:
            self.get_logger().warn(f'Invalid target ID: {msg.data}. Ignoring...')
            return
            
        self.target_id = msg.data
        self.get_logger().info(f'Target ID set to: {self.target_id}')
        
        # 새로운 미션 시작 조건 확인 (IDLE 또는 FINISHED 상태에서)
        if self.target_distance is not None and self.state in [RobotState.IDLE, RobotState.FINISHED]:
            self.start_mission()
        elif self.target_distance is not None:
            self.get_logger().info(f'Mission parameters updated: ID={self.target_id}, Distance={self.target_distance:.2f}m')
        else:
            self.get_logger().info('Waiting for target distance...')
            
        # 미션 진행 중 ID 변경 처리
        if self.state in [RobotState.SEARCHING, RobotState.APPROACHING, RobotState.ALIGNING]:
            # 현재 미션과 다른 ID인 경우 새로운 미션 시작
            if self.current_mission_id != self.target_id:
                self.start_mission()

    def start_mission(self):
        """미션 시작"""
        self.state = RobotState.SEARCHING
        self.state_change_counter = 0
        self.mission_completed = False
        self.final_pitch_aligned = False  # 최종 pitch 정렬 초기화
        
        # 현재 미션 파라미터 저장
        self.current_mission_id = self.target_id
        self.current_mission_distance = self.target_distance
        
        # 적분항 초기화
        self.reset_integrals()
        
        self.get_logger().info(f'=== MISSION STARTED ===')
        self.get_logger().info(f'Target: ID={self.target_id}, Distance={self.target_distance:.2f}m')
        self.get_logger().info('Starting search for target marker...')

    def pose_callback(self, msg):
        """ArUco 마커 포즈 수신"""
        # IDLE 상태이거나 타겟 정보가 없으면 무시
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
        
        # 유효한 포즈 데이터인지 확인
        if z <= 0.05 or z > 10.0:
            self.get_logger().warn(f'Invalid pose data: distance={z:.3f}m. Ignoring...')
            return
        
        # 쿼터니언을 오일러 각도로 변환하여 pitch 계산
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # 회전 행렬 계산
        rot_matrix = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        
        # Pitch 각도 계산 (Y축 중심 회전)
        sy = math.sqrt(rot_matrix[0, 0]**2 + rot_matrix[1, 0]**2)
        if sy > 1e-6:
            self.marker_pitch = math.atan2(-rot_matrix[2, 0], sy)
        else:
            self.marker_pitch = math.atan2(-rot_matrix[2, 0], sy)
        
        self.marker_distance = z
        self.marker_lateral = x
        
        # 상태 업데이트
        if self.state == RobotState.SEARCHING:
            if (self.marker_distance is not None and self.marker_lateral is not None and 
                self.marker_pitch is not None and self.target_distance is not None):
                self.state = RobotState.APPROACHING
                self.state_change_counter = 0
                self.reset_integrals()  # 적분항 초기화
                self.get_logger().info(f'Target marker {self.target_id} detected! Starting approach...')

    def control_loop(self):
        """메인 제어 루프"""
        # IDLE 상태에서는 완전히 정지
        if self.state == RobotState.IDLE:
            self.stop_robot()
            return
            
        # 타겟 정보가 없으면 IDLE 상태로 복귀
        if self.target_id is None or self.target_distance is None:
            if self.state != RobotState.IDLE:
                self.get_logger().info('Missing target information, returning to IDLE state')
                self.state = RobotState.IDLE
                self.state_change_counter = 0
                self.reset_integrals()
            self.stop_robot()
            return
            
        current_time = self.get_clock().now()
        
        # 마커 검출 시간 체크 (FINISHED 상태가 아닐 때만)
        if self.state != RobotState.FINISHED:
            if (current_time - self.last_pose_time).nanoseconds / 1e9 > self.pose_timeout:
                if self.state != RobotState.SEARCHING:
                    self.get_logger().warn(f'Target marker {self.target_id} lost! Searching again...')
                    self.state = RobotState.SEARCHING
                    self.state_change_counter = 0
                    self.reset_integrals()  # 적분항 초기화
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

    def search_behavior(self):
        """마커 검색 행동"""
        if self.target_id is not None and self.target_distance is not None:
            twist = Twist()
            twist.angular.z = 0.2  # 천천히 회전
            self.cmd_vel_pub.publish(twist)
        else:
            self.stop_robot()

    def approach_behavior(self):
        """마커 접근 행동 - PI 제어 적용"""
        if self.current_pose is None or self.marker_distance is None or self.target_distance is None:
            self.stop_robot()
            return
            
        # 시간 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_time
        
        # dt가 너무 크면 적분항 초기화 (긴 시간 간격 후 첫 제어)
        if dt > 0.5:
            self.reset_integrals()
            return
        
        twist = Twist()
        
        # 거리 제어 (PI 제어)
        distance_error = self.marker_distance - self.target_distance
        
        # 거리 적분항 계산
        self.integral_distance += distance_error * dt
        # 적분항 제한 (windup 방지)
        self.integral_distance = max(-self.integral_limit_distance, 
                                   min(self.integral_limit_distance, self.integral_distance))
        
        # 목표 거리 도달 체크
        if abs(distance_error) < self.distance_tolerance:
            self.state_change_counter += 1
            if self.state_change_counter >= self.state_change_threshold:
                self.state = RobotState.ALIGNING
                self.state_change_counter = 0
                self.reset_integrals()  # 정렬 단계로 전환 시 적분항 초기화
                self.get_logger().info('Target distance reached! Starting alignment...')
                return
        else:
            self.state_change_counter = 0
        
        # PI 제어로 선속도 계산
        linear_vel = (self.kp_distance * distance_error + 
                     self.ki_distance * self.integral_distance)
        
        # 속도 제한 및 최소 속도 설정
        if abs(linear_vel) < 0.02:
            linear_vel = 0.02 if linear_vel > 0 else -0.02
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        
        # 좌우 정렬을 위한 PI 제어
        lateral_error = self.marker_lateral
        
        # 좌우 적분항 계산
        self.integral_lateral += lateral_error * dt
        # 적분항 제한 (windup 방지)
        self.integral_lateral = max(-self.integral_limit_lateral, 
                                  min(self.integral_limit_lateral, self.integral_lateral))
        
        # PI 제어로 각속도 계산
        angular_vel = -(self.kp_lateral * lateral_error + 
                       self.ki_lateral * self.integral_lateral)
        
        # 각속도 제한
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # 너무 가까우면 후진만
        if self.marker_distance < 0.15:
            twist.linear.x = -0.05
            twist.angular.z = angular_vel * 0.5
        else:
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
        
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(f'Approaching (PI) - Distance: {self.marker_distance:.3f}m, '
                             f'Lateral: {self.marker_lateral:.3f}m, '
                             f'Pitch: {self.marker_pitch:.3f}rad ({math.degrees(self.marker_pitch):.1f}°), '
                             f'Linear: {twist.linear.x:.3f}, Angular: {twist.angular.z:.3f}, '
                             f'I_dist: {self.integral_distance:.3f}, I_lat: {self.integral_lateral:.3f}')

    def align_behavior(self):
        """마커 방향 정렬 행동 - PI 제어 적용"""
        if (self.current_pose is None or self.marker_distance is None or 
            self.marker_lateral is None or self.marker_pitch is None or 
            self.target_distance is None):
            self.stop_robot()
            return
            
        # 시간 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_time
        
        # dt가 너무 크면 적분항 초기화
        if dt > 0.5:
            self.reset_integrals()
            return
            
        twist = Twist()
        
        # 미세 거리 조정
        distance_error = self.marker_distance - self.target_distance
        if abs(distance_error) > self.distance_tolerance * 3:
            self.state = RobotState.APPROACHING
            self.state_change_counter = 0
            self.reset_integrals()  # 접근 단계로 전환 시 적분항 초기화
            self.get_logger().info('Distance too far, returning to approach mode...')
            return
        
        # 정렬 상태 확인
        lateral_error = self.marker_lateral
        angle_error = self.marker_pitch  # pitch 사용
        
        lateral_aligned = abs(lateral_error) <= self.lateral_tolerance
        angle_aligned = abs(angle_error) <= self.angle_tolerance or abs(angle_error - 2*math.pi) <= self.angle_tolerance or abs(angle_error + 2*math.pi) <= self.angle_tolerance
        distance_ok = abs(distance_error) <= self.distance_tolerance
        
        if lateral_aligned and angle_aligned and distance_ok:
            self.state_change_counter += 1
            self.get_logger().info(f'Alignment progress: {self.state_change_counter}/{self.state_change_threshold}')
            if self.state_change_counter >= self.state_change_threshold:
                self.state = RobotState.FINISHED
                self.state_change_counter = 0
                self.reset_integrals()  # 완료 단계로 전환 시 적분항 초기화
                self.get_logger().info('Alignment completed! Starting final pitch alignment...')
                return
        else:
            self.state_change_counter = 0
            
            # PI 제어로 정렬 수행
            if not lateral_aligned:
                # 좌우 적분항 계산
                self.integral_lateral += lateral_error * dt
                self.integral_lateral = max(-self.integral_limit_lateral, 
                                          min(self.integral_limit_lateral, self.integral_lateral))
                
                angular_vel = -(self.kp_lateral * lateral_error * 0.3 + 
                               self.ki_lateral * self.integral_lateral * 0.3)
                angular_vel = max(-self.max_angular_vel * 0.3, min(self.max_angular_vel * 0.3, angular_vel))
                twist.angular.z = angular_vel
                
            elif not angle_aligned:
                normalized_angle = angle_error
                if normalized_angle > math.pi:
                    normalized_angle -= 2 * math.pi
                elif normalized_angle < -math.pi:
                    normalized_angle += 2 * math.pi
                
                # 각도 적분항 계산
                self.integral_angular += normalized_angle * dt
                self.integral_angular = max(-self.integral_limit_angular, 
                                          min(self.integral_limit_angular, self.integral_angular))
                
                angular_vel = -(self.kp_angular * normalized_angle * 1.1 + 
                               self.ki_angular * self.integral_angular * 1.1)
                angular_vel = max(-self.max_angular_vel * 0.3, min(self.max_angular_vel * 0.3, angular_vel))
                twist.angular.z = angular_vel
        
        # 미세 거리 조정 (PI 제어)
        if abs(distance_error) > self.distance_tolerance * 0.5:
            self.integral_distance += distance_error * dt
            self.integral_distance = max(-self.integral_limit_distance * 0.1, 
                                       min(self.integral_limit_distance * 0.1, self.integral_distance))
            
            linear_vel = (self.kp_distance * distance_error * 1.1 + 
                         self.ki_distance * self.integral_distance * 1.1)
            linear_vel = max(-0.03, min(0.03, linear_vel))
            twist.linear.x = linear_vel
        
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(f'Aligning (PI) - Lateral: {self.marker_lateral:.3f}m, '
                             f'Pitch: {self.marker_pitch:.3f}rad ({math.degrees(self.marker_pitch):.1f}°), '
                             f'Distance: {self.marker_distance:.3f}m, '
                             f'L_OK: {lateral_aligned}, A_OK: {angle_aligned}, D_OK: {distance_ok}, '
                             f'I_lat: {self.integral_lateral:.3f}, I_ang: {self.integral_angular:.3f}')

    def finish_behavior(self):
        """미션 완료 후 행동 - 최종 pitch 정렬 수행 (PI 제어)"""
        # 마커 검출 중단 시 정지
        current_time = self.get_clock().now()
        if (current_time - self.last_pose_time).nanoseconds / 1e9 > self.pose_timeout:
            self.stop_robot()
            if not self.mission_completed:
                self.mission_completed = True
                self.get_logger().info('=== MISSION COMPLETED ===')
                self.get_logger().info(f'Successfully reached marker {self.target_id} at {self.target_distance:.2f}m')
                self.get_logger().info('Marker lost. Robot stopped.')
            return
        
        # 최종 pitch 정렬 수행 (PI 제어)
        if self.marker_pitch is not None and not self.final_pitch_aligned:
            # 시간 계산
            dt = (current_time - self.last_control_time).nanoseconds / 1e9
            self.last_control_time = current_time
            
            # dt가 너무 크면 적분항 초기화
            if dt > 0.5:
                self.reset_integrals()
                return
            
            twist = Twist()
            
            # pitch 각도 정규화
            normalized_pitch = self.marker_pitch
            if normalized_pitch > math.pi:
                normalized_pitch -= 2 * math.pi
            elif normalized_pitch < -math.pi:
                normalized_pitch += 2 * math.pi
            
            # 최종 pitch 정렬 확인
            if abs(normalized_pitch) <= self.final_pitch_tolerance:
                self.state_change_counter += 1
                if self.state_change_counter >= self.state_change_threshold:
                    self.final_pitch_aligned = True
                    self.state_change_counter = 0
                    self.get_logger().info('Final pitch alignment completed!')
            else:
                self.state_change_counter = 0
                
                # 최종 pitch 정렬을 위한 PI 제어
                self.integral_angular += normalized_pitch * dt
                self.integral_angular = max(-self.integral_limit_angular * 0.1, 
                                          min(self.integral_limit_angular * 0.1, self.integral_angular))
                
                angular_vel = -(self.kp_angular * normalized_pitch * 0.2 + 
                               self.ki_angular * self.integral_angular * 0.2)
                angular_vel = max(-self.max_angular_vel * 0.2, min(self.max_angular_vel * 0.2, angular_vel))
                twist.angular.z = angular_vel
                
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'Final pitch alignment (PI) - Pitch: {self.marker_pitch:.3f}rad ({math.degrees(self.marker_pitch):.1f}°), '
                                     f'I_ang: {self.integral_angular:.3f}')
                return
        
        # 완전히 정지
        self.stop_robot()
        
        # 미션 완료 로그 (한 번만)
        if not self.mission_completed:
            self.mission_completed = True
            self.get_logger().info('=== MISSION COMPLETED ===')
            self.get_logger().info(f'Successfully reached marker {self.target_id} at {self.target_distance:.2f}m')
            self.get_logger().info('Final pitch alignment completed! Robot stopped.')
            self.get_logger().info('Send new target_id or target_distance to start new mission.')

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
    node = ArucoRobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()