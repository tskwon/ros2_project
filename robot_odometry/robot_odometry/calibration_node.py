#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray  # 변경된 import
from geometry_msgs.msg import Twist

class OdometryCalibration(Node):
    """MDH250H 모터 캘리브레이션 노드"""
    
    def __init__(self):
        super().__init__('odometry_calibration')
        
        self.encoder_resolution = 4096  # MDH250H: 4,096PPR
        self.wheel_radius = 0.033       # 초기 추정값
        self.wheel_base = 0.16          # 초기 추정값
        
        self.initial_left = None
        self.initial_right = None
        self.test_distance = 1.0        # 테스트 거리 (m)
        self.test_angle = math.pi       # 테스트 각도 (180도)
        
        # Int32MultiArray 구독자로 변경
        self.encoder_sub = self.create_subscription(
            Int32MultiArray, 'encoder_values', self.encoder_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('=== MDH250H 모터 캘리브레이션 시작 ===')
        self.get_logger().info('엔코더 토픽: /encoder_values (Int32MultiArray)')
        self.get_logger().info('1. 직진 테스트: 로봇을 정확히 1미터 직진시키세요')
        self.get_logger().info('2. 회전 테스트: 로봇을 180도 회전시키세요')

    def encoder_callback(self, msg):
        """Int32MultiArray 형태의 엔코더 데이터 처리"""
        if len(msg.data) < 2:
            self.get_logger().warn(f'엔코더 데이터 부족: {len(msg.data)}개 (필요: 2개)')
            return
        
        current_left = int(msg.data[0])   # 왼쪽 바퀴 엔코더
        current_right = int(msg.data[1])  # 오른쪽 바퀴 엔코더
        
        if self.initial_left is None:
            self.initial_left = current_left
            self.initial_right = current_right
            self.get_logger().info(f'초기 엔코더 값 저장 - L: {current_left}, R: {current_right}')
            return
        
        # 이동한 엔코더 틱 수
        delta_left = abs(current_left - self.initial_left)
        delta_right = abs(current_right - self.initial_right)
        avg_ticks = (delta_left + delta_right) / 2
        
        if avg_ticks > 100:  # 충분히 이동했을 때
            # 계산된 이동 거리
            calculated_distance = (avg_ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
            
            self.get_logger().info(f'=== 캘리브레이션 결과 ===')
            self.get_logger().info(f'왼쪽 바퀴 틱 변화: {delta_left}')
            self.get_logger().info(f'오른쪽 바퀴 틱 변화: {delta_right}')
            self.get_logger().info(f'평균 엔코더 틱: {avg_ticks:.1f}')
            self.get_logger().info(f'계산된 이동거리: {calculated_distance:.3f}m')
            
            if calculated_distance > 0.1:
                # 바퀴 반지름 보정값 계산
                corrected_radius = self.wheel_radius * (self.test_distance / calculated_distance)
                self.get_logger().info(f'보정된 바퀴 반지름: {corrected_radius:.6f}m')
                
                # 회전 테스트의 경우
                if abs(delta_left - delta_right) > avg_ticks * 0.1:  # 좌우 차이가 큰 경우 (회전)
                    angle_ticks = abs(delta_left - delta_right)
                    calculated_angle = (angle_ticks / self.encoder_resolution) * 2 * math.pi * self.wheel_radius / self.wheel_base
                    
                    self.get_logger().info(f'회전 틱 차이: {angle_ticks}')
                    self.get_logger().info(f'계산된 회전각: {math.degrees(calculated_angle):.1f}도')
                    
                    if calculated_angle > 0.1:
                        corrected_wheel_base = self.wheel_base * (calculated_angle / self.test_angle)
                        self.get_logger().info(f'보정된 휠베이스: {corrected_wheel_base:.6f}m')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
