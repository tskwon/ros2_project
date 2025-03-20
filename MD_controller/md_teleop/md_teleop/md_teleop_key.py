#!/usr/bin/env python3
import rclpy
import sys
import signal
import os
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from getkey import getkey

MAX_VEL = 1000
VEL_STEP_SIZE = 10

MSG = """
-------------------------------------------------
W: Forward  S: Backward  
A: Turn Left  D: Turn Right  

CTRL-C to quit
-------------------------------------------------
"""

class TeleopKey(Node):
    def __init__(self):
        super().__init__("md_teleop_key_node")
        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self.cmd_rpm_pub = self.create_publisher(Int32MultiArray, "/cmd_rpm", qos_profile)
        self.left_rpm = 0
        self.right_rpm = 0
        signal.signal(signal.SIGINT, self.signal_handler)
        os.system('clear')
        print(MSG)
        print(f"Left RPM: {self.left_rpm}, Right RPM: {self.right_rpm}")

    def run(self):
        """키 입력을 블록킹 방식으로 처리"""
        while rclpy.ok():
            key = getkey(blocking=True).lower()  # 블록킹 모드로 키 입력 대기
            if key == 'w':
                self.left_rpm = self.check_rpm_limit(self.left_rpm + VEL_STEP_SIZE)
                self.right_rpm = self.check_rpm_limit(self.right_rpm + VEL_STEP_SIZE)
            elif key == 's':
                self.left_rpm = self.check_rpm_limit(self.left_rpm - VEL_STEP_SIZE)
                self.right_rpm = self.check_rpm_limit(self.right_rpm - VEL_STEP_SIZE)
            elif key == 'a':
                self.left_rpm = self.check_rpm_limit(self.left_rpm - VEL_STEP_SIZE)
                self.right_rpm = self.check_rpm_limit(self.right_rpm + VEL_STEP_SIZE)
            elif key == 'd':
                self.left_rpm = self.check_rpm_limit(self.left_rpm + VEL_STEP_SIZE)
                self.right_rpm = self.check_rpm_limit(self.right_rpm - VEL_STEP_SIZE)

            rpm_msg = Int32MultiArray()
            rpm_msg.data = [-self.left_rpm, self.right_rpm]
            self.cmd_rpm_pub.publish(rpm_msg)

            os.system('clear')
            print(MSG)
            print(f"Left RPM: {self.left_rpm}, Right RPM: {self.right_rpm}")

    def signal_handler(self, sig, frame):
        rpm_msg = Int32MultiArray()
        rpm_msg.data = [0, 0]
        self.cmd_rpm_pub.publish(rpm_msg)
        self.get_logger().info(f"Left RPM: {rpm_msg.data[0]}, Right RPM: {rpm_msg.data[1]} - Node shutting down")
        rclpy.shutdown()

    def check_rpm_limit(self, vel):
        if vel <= -MAX_VEL:
            return -MAX_VEL
        elif vel >= MAX_VEL:
            return MAX_VEL
        return vel

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