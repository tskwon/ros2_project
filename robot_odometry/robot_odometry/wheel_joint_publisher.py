#!/usr/bin/env python3
"""
차동구동 로봇 바퀴 조인트 퍼블리셔
cmd_vel 토픽을 구독하여 바퀴 회전 정보를 joint_states로 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import math

class WheelJointPublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_publisher')
        
        # 파라미터 선언 및 기본값 설정
        self.declare_parameter('wheel_radius', 0.103)
        self.declare_parameter('wheel_base', 0.503)
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')
        self.declare_parameter('publish_rate', 30.0)
        
        # 파라미터 값 가져오기
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.left_joint_name = self.get_parameter('left_wheel_joint').get_parameter_value().string_value
        self.right_joint_name = self.get_parameter('right_wheel_joint').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # 퍼블리셔와 구독자
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 타이머 설정
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_joint_states)
        
        # 바퀴 각도 및 속도 저장
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        
        # 현재 cmd_vel 값
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # 이전 시간 저장
        self.last_time = self.get_clock().now()
        
        self.get_logger().info(f'Wheel Joint Publisher initialized')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m')
        self.get_logger().info(f'Publish rate: {self.publish_rate}Hz')
        
    def cmd_vel_callback(self, msg):
        """cmd_vel 토픽 콜백"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        # 차동구동 키네마틱스 계산
        # 왼쪽 바퀴 속도 = (선속도 - 각속도 * 바퀴간격/2) / 바퀴반지름
        # 오른쪽 바퀴 속도 = (선속도 + 각속도 * 바퀴간격/2) / 바퀴반지름
        self.left_wheel_velocity = (self.linear_vel - self.angular_vel * self.wheel_base / 2.0) / self.wheel_radius
        self.right_wheel_velocity = (self.linear_vel + self.angular_vel * self.wheel_base / 2.0) / self.wheel_radius
        
        # 디버그 출력
        if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
            self.get_logger().info(f'Linear: {self.linear_vel:.3f}, Angular: {self.angular_vel:.3f}')
            self.get_logger().info(f'Left wheel vel: {self.left_wheel_velocity:.3f}, Right wheel vel: {self.right_wheel_velocity:.3f}')
        
    def publish_joint_states(self):
        """조인트 상태 발행"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        
        # 바퀴 각도 업데이트 (적분)
        self.left_wheel_angle += self.left_wheel_velocity * dt
        self.right_wheel_angle += self.right_wheel_velocity * dt
        
        # 각도를 -π ~ π 범위로 정규화
        self.left_wheel_angle = self.normalize_angle(self.left_wheel_angle)
        self.right_wheel_angle = self.normalize_angle(self.right_wheel_angle)
        
        # JointState 메시지 생성
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.header.frame_id = ""
        
        joint_state.name = [self.left_joint_name, self.right_joint_name]
        joint_state.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_state.velocity = [self.left_wheel_velocity, self.right_wheel_velocity]
        joint_state.effort = []
        
        self.joint_pub.publish(joint_state)
        
    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    
    try:
        wheel_joint_publisher = WheelJointPublisher()
        rclpy.spin(wheel_joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'wheel_joint_publisher' in locals():
            wheel_joint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()