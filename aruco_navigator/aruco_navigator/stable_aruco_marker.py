#!/usr/bin/env python3
"""
물류 로봇용 안정적인 ArUco 마커 네비게이션
노이즈 필터링과 안정적인 목표점 생성
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32, Bool
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import numpy as np
from collections import deque
import math

class StableArucoNavigator(Node):
    def __init__(self):
        super().__init__('stable_aruco_detection_node')
        
        # TF2 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco/marker_pose',
            self.pose_callback,
            10
        )
        
        # Publishers
        self.stable_pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco/stable_marker_pose',
            10
        )
        
        self.goal_point_pub = self.create_publisher(
            Point,
            '/aruco/goal_point',
            10
        )
        
        self.detection_status_pub = self.create_publisher(
            Bool,
            '/aruco/detection_stable',
            10
        )
        
        # 안정화 필터 설정
        self.buffer_size = 15  # 더 큰 버퍼로 안정성 향상
        self.position_buffer = deque(maxlen=self.buffer_size)
        self.orientation_buffer = deque(maxlen=self.buffer_size)
        
        # 노이즈 필터링 파라미터
        self.position_threshold = 0.005  # 5mm 이하 변화는 무시
        self.min_samples = 8  # 최소 8개 샘플로 안정성 확보
        self.max_std_dev = 0.01  # 표준편차 1cm 이하일 때만 안정으로 판단
        
        # 마커까지의 거리에 따른 접근점 계산
        self.approach_distance = 0.5  # 마커 앞 50cm 지점으로 이동
        
        # 상태 관리
        self.current_stable_pose = None
        self.last_detection_time = self.get_clock().now()
        self.detection_timeout = 2.0  # 2초간 미검출시 타임아웃
        self.is_stable = False
        
        # 타이머 - 주기적으로 상태 체크
        self.timer = self.create_timer(0.1, self.check_detection_status)
        
        self.get_logger().info('Stable ArUco Navigator for Logistics Robot started')
    
    def pose_callback(self, msg):
        """ArUco 포즈를 받아서 필터링하고 안정적인 목표점 생성"""
        self.last_detection_time = self.get_clock().now()
        
        try:
            # 카메라 프레임에서 odom 프레임으로 변환
            transform = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # 포즈 변환
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(msg, transform)
            
            # 위치 데이터 추출
            current_pos = np.array([
                transformed_pose.pose.position.x,
                transformed_pose.pose.position.y,
                transformed_pose.pose.position.z
            ])
            
            current_ori = np.array([
                transformed_pose.pose.orientation.w,
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z
            ])
            
            # 버퍼에 추가
            self.position_buffer.append(current_pos)
            self.orientation_buffer.append(current_ori)
            
            # 충분한 샘플이 모이면 안정성 검사
            if len(self.position_buffer) >= self.min_samples:
                self.process_stable_detection(transformed_pose.header)
                
        except TransformException as e:
            self.get_logger().warn(f'Transform failed: {e}', throttle_duration_sec=1.0)
    
    def process_stable_detection(self, header):
        """안정적인 검출 여부 판단하고 목표점 생성"""
        
        # 위치 통계 계산
        positions = np.array(self.position_buffer)
        mean_position = np.mean(positions, axis=0)
        std_position = np.std(positions, axis=0)
        max_std = np.max(std_position)
        
        # 안정성 판단
        if max_std < self.max_std_dev:
            if not self.is_stable:
                self.get_logger().info(
                    f'Marker position stabilized at: '
                    f'({mean_position[0]:.3f}, {mean_position[1]:.3f}, {mean_position[2]:.3f})'
                )
                self.is_stable = True
            
            # 안정적인 포즈 계산
            mean_orientation = np.mean(self.orientation_buffer, axis=0)
            mean_orientation = mean_orientation / np.linalg.norm(mean_orientation)
            
            # 안정적인 포즈 퍼블리시
            stable_pose = self.create_pose_stamped(header, mean_position, mean_orientation)
            self.stable_pose_pub.publish(stable_pose)
            self.current_stable_pose = stable_pose
            
            # 로봇 접근 목표점 계산
            goal_point = self.calculate_approach_point(mean_position, mean_orientation)
            self.goal_point_pub.publish(goal_point)
            
            # 안정 상태 퍼블리시
            status_msg = Bool()
            status_msg.data = True
            self.detection_status_pub.publish(status_msg)
            
        else:
            self.is_stable = False
            # 불안정 상태 퍼블리시
            status_msg = Bool()
            status_msg.data = False
            self.detection_status_pub.publish(status_msg)
    
    def calculate_approach_point(self, marker_position, marker_orientation):
        """마커 위치를 기반으로 로봇이 접근할 목표점 계산"""
        
        # 마커의 회전을 오일러 각으로 변환
        qw, qx, qy, qz = marker_orientation
        
        # Z축 회전 (yaw) 계산
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # 마커 앞쪽 방향으로 접근 거리만큼 떨어진 지점 계산
        approach_x = marker_position[0] - self.approach_distance * math.cos(yaw)
        approach_y = marker_position[1] - self.approach_distance * math.sin(yaw)
        approach_z = 0.0  # 지면 레벨
        
        goal_point = Point()
        goal_point.x = approach_x
        goal_point.y = approach_y
        goal_point.z = approach_z
        
        return goal_point
    
    def create_pose_stamped(self, header, position, orientation):
        """PoseStamped 메시지 생성"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = 'odom'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        
        pose_msg.pose.orientation.w = float(orientation[0])
        pose_msg.pose.orientation.x = float(orientation[1])
        pose_msg.pose.orientation.y = float(orientation[2])
        pose_msg.pose.orientation.z = float(orientation[3])
        
        return pose_msg
    
    def check_detection_status(self):
        """주기적으로 검출 상태 확인"""
        current_time = self.get_clock().now()
        time_since_detection = (current_time - self.last_detection_time).nanoseconds / 1e9
        
        if time_since_detection > self.detection_timeout:
            if self.is_stable:
                self.get_logger().info('Marker detection lost - maintaining last stable position')
                self.is_stable = False
                
                # 검출 중단 상태 퍼블리시
                status_msg = Bool()
                status_msg.data = False
                self.detection_status_pub.publish(status_msg)

def main():
    rclpy.init()
    node = StableArucoNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()