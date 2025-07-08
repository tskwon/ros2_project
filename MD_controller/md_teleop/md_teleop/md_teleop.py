#!/usr/bin/env python3
import rclpy
import sys
import signal
import os
import tty
import termios
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 선속도와 각속도 설정
LINEAR_STEP_SIZE = 0.1  # 0.1 m/s 씩 증가
ANGULAR_STEP_SIZE = 0.1  # 0.1 rad/s 씩 증가
MAX_LINEAR = 0.5  # 최대 선속도 (m/s)
MAX_ANGULAR = 1.0  # 최대 각속도 (rad/s)

MSG = """
-------------------------------------------------
W: Forward  S: Backward    
A: Turn Left  D: Turn Right
    SPACE: Stop
CTRL-C to quit
-------------------------------------------------
"""

# getkey 대신 getch 함수 구현
def getch():
    """터미널에서 키 입력을 받는 함수"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class TeleopKey(Node):
    def __init__(self):
        super().__init__("md_teleop_key_node")
        qos_profile = rclpy.qos.QoSProfile(depth=2, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        
        # cmd_vel 토픽으로 변경 (Twist 메시지 사용)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile)
        
        # 선속도와 각속도 초기화
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        signal.signal(signal.SIGINT, self.signal_handler)
        os.system('clear')
        print(MSG)
        print(f"Linear: {self.linear_vel} m/s, Angular: {self.angular_vel} rad/s")
    
    def run(self):
        """키 입력을 블록킹 방식으로 처리"""
        while rclpy.ok():
            key = getch().lower()  # getkey 대신 getch 함수 사용
            
            # Ctrl+C 처리
            if key == '\x03':
                self.signal_handler(signal.SIGINT, None)
                break
            
            if key == 'w':
                self.linear_vel = self.check_limit(self.linear_vel + LINEAR_STEP_SIZE, MAX_LINEAR)
            elif key == 's':
                self.linear_vel = self.check_limit(self.linear_vel - LINEAR_STEP_SIZE, MAX_LINEAR)
            elif key == 'a':
                self.angular_vel = self.check_limit(self.angular_vel + ANGULAR_STEP_SIZE, MAX_ANGULAR)
            elif key == 'd':
                self.angular_vel = self.check_limit(self.angular_vel - ANGULAR_STEP_SIZE, MAX_ANGULAR)
            elif key == ' ':
                self.linear_vel = 0.0
                self.angular_vel = 0.0
            
            # Twist 메시지 생성 및 발행
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_vel
            twist_msg.angular.z = self.angular_vel
            self.cmd_vel_pub.publish(twist_msg)
            
            os.system('clear')
            print(MSG)
            print(f"Linear: {self.linear_vel:.2f} m/s, Angular: {self.angular_vel:.2f} rad/s")
    
    def signal_handler(self, sig, frame):
        # 정지 명령 전송
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Linear: {twist_msg.linear.x} m/s, Angular: {twist_msg.angular.z} rad/s - Node shutting down")
        rclpy.shutdown()
    
    def check_limit(self, value, max_value):
        """값이 최대/최소 범위를 벗어나지 않도록 제한"""
        if value > max_value:
            return max_value
        elif value < -max_value:
            return -max_value
        return value

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    try:
        node.run()  # 블록킹 루프 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()