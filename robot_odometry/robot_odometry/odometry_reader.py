#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Quaternion
import math
import time

class OdometryReader(Node):
    def __init__(self):
        super().__init__('odometry_reader')
        
        # 구독자 생성
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
        # EKF 필터링된 위치 저장
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_yaw = 0.0
        self.filtered_yaw_deg = 0.0
        
        # IMU 직접 계산된 각도 저장
        self.imu_yaw = 0.0
        self.imu_yaw_deg = 0.0
        
        # 이전 위치 저장 (이동 거리 계산용)
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        
        # 누적 거리
        self.total_distance = 0.0
        self.total_rotation = 0.0
        
        # 통계 정보
        self.odometry_count = 0
        self.imu_count = 0
        self.start_time = time.time()
        
        # 데이터 수신 플래그
        self.odometry_received = False
        self.imu_received = False
        
        self.get_logger().info('Odometry Reader 노드가 시작되었습니다.')
        self.get_logger().info('토픽: /odometry/filtered, /imu/data 를 구독하고 있습니다.')

    def odometry_callback(self, msg):
        """EKF 필터링된 오도메트리 데이터 콜백 함수"""
        try:
            # 위치 정보 추출
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # 쿼터니언 추출
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            # 쿼터니언에서 yaw 각도 계산
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                           1.0 - 2.0 * (qy * qy + qz * qz))
            yaw_deg = math.degrees(yaw)
            
            # 이동 거리 계산
            if self.odometry_count > 0:
                dx = x - self.prev_x
                dy = y - self.prev_y
                distance = math.sqrt(dx*dx + dy*dy)
                self.total_distance += distance
                
                # 회전 변화량 계산
                dyaw = abs(yaw - self.prev_yaw)
                # 각도 wrapping 처리
                if dyaw > math.pi:
                    dyaw = 2*math.pi - dyaw
                self.total_rotation += dyaw
            
            # 현재 값 저장
            self.filtered_x = x
            self.filtered_y = y
            self.filtered_yaw = yaw
            self.filtered_yaw_deg = yaw_deg
            
            # 이전 값 업데이트
            self.prev_x = x
            self.prev_y = y
            self.prev_yaw = yaw
            
            self.odometry_count += 1
            self.odometry_received = True
            
            # 두 데이터가 모두 수신되면 비교 출력
            if self.imu_received:
                self.print_comparison()
            
        except Exception as e:
            self.get_logger().error(f'오도메트리 데이터 처리 중 오류: {e}')

    def imu_callback(self, msg):
        """IMU 데이터 콜백 함수"""
        try:
            # 쿼터니언 추출
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            
            # 쿼터니언에서 yaw 각도 계산
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                           1.0 - 2.0 * (qy * qy + qz * qz))
            yaw_deg = math.degrees(yaw)
            
            # IMU 값 저장
            self.imu_yaw = yaw
            self.imu_yaw_deg = yaw_deg
            
            self.imu_count += 1
            self.imu_received = True
            
        except Exception as e:
            self.get_logger().error(f'IMU 데이터 처리 중 오류: {e}')

    def print_comparison(self):
        """EKF와 IMU 데이터 비교 출력"""
        # 각도 차이 계산
        angle_diff = abs(self.filtered_yaw - self.imu_yaw)
        
        # 각도 wrapping 처리 (-π to π)
        if angle_diff > math.pi:
            angle_diff = 2*math.pi - angle_diff
        
        angle_diff_deg = math.degrees(angle_diff)
        
        # 비교 출력
        self.get_logger().info(
            f'위치 -> x: {self.filtered_x:7.3f}m, y: {self.filtered_y:7.3f}m'
        )
        self.get_logger().info(
            f'EKF Yaw : {self.filtered_yaw:6.3f}rad ({self.filtered_yaw_deg:6.1f}°)'
        )
        self.get_logger().info(
            f'IMU Yaw : {self.imu_yaw:6.3f}rad ({self.imu_yaw_deg:6.1f}°)'
        )
        self.get_logger().info(
            f'각도차이: {angle_diff:6.3f}rad ({angle_diff_deg:6.1f}°)'
        )
        self.get_logger().info('─' * 60)

    def get_current_pose(self):
        """현재 위치 반환 (다른 노드에서 사용 가능)"""
        return {
            'filtered_x': self.filtered_x,
            'filtered_y': self.filtered_y,
            'filtered_yaw': self.filtered_yaw,
            'filtered_yaw_deg': self.filtered_yaw_deg,
            'imu_yaw': self.imu_yaw,
            'imu_yaw_deg': self.imu_yaw_deg,
            'angle_difference': abs(self.filtered_yaw - self.imu_yaw)
        }

    def reset_odometry(self):
        """오도메트리 리셋"""
        self.total_distance = 0.0
        self.total_rotation = 0.0
        self.odometry_count = 0
        self.imu_count = 0
        self.start_time = time.time()
        self.odometry_received = False
        self.imu_received = False
        self.get_logger().info('오도메트리가 리셋되었습니다.')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        odometry_reader = OdometryReader()
        rclpy.spin(odometry_reader)
    except KeyboardInterrupt:
        print('\n키보드 인터럽트 감지됨. 노드를 종료합니다.')
    except Exception as e:
        print(f'예상치 못한 오류: {e}')
    finally:
        if 'odometry_reader' in locals():
            odometry_reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
