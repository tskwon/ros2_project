#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys

class SimpleTest(Node):
    def __init__(self):
        super().__init__('simple_test_node')
        
        # 테스트 명령 발행자
        self.command_pub = self.create_publisher(String, '/test/command', 10)
        
        # 상태 구독자
        self.status_sub = self.create_subscription(
            String,
            '/test/status',
            self.status_callback,
            10
        )
        
        self.get_logger().info('🧪 간단한 테스트 노드가 시작되었습니다.')
        
        # 타이머로 자동 테스트 실행
        self.test_timer = self.create_timer(3.0, self.run_auto_test)
        self.test_step = 0
        self.test_steps = [
            ('reset', None),
            ('start', 'apple'),
            ('wait', 20),  # 20초 대기
            ('reset', None),
            ('start', 'bottle'),
            ('wait', 20),
            ('home', None)
        ]

    def status_callback(self, msg):
        """상태 메시지 수신"""
        self.get_logger().info(f'📊 [상태] {msg.data}')

    def send_command(self, command, target=None):
        """명령 전송"""
        command_data = {'command': command}
        if target:
            command_data['target'] = target
            
        msg = String()
        msg.data = json.dumps(command_data)
        self.command_pub.publish(msg)
        
        self.get_logger().info(f'📤 [명령 전송] {command_data}')

    def run_auto_test(self):
        """자동 테스트 실행"""
        if self.test_step >= len(self.test_steps):
            self.get_logger().info('🎉 모든 자동 테스트 완료!')
            self.test_timer.cancel()
            return
            
        current_step = self.test_steps[self.test_step]
        command = current_step[0]
        target = current_step[1]
        
        self.get_logger().info(f'🔄 테스트 단계 {self.test_step + 1}/{len(self.test_steps)}: {command}')
        
        if command == 'wait':
            self.get_logger().info(f'⏳ {target}초 대기 중...')
            # 대기 시간을 위해 타이머 조정
            self.test_timer.cancel()
            self.test_timer = self.create_timer(float(target), self.run_auto_test)
        else:
            self.send_command(command, target)
            
        self.test_step += 1

def main(args=None):
    rclpy.init(args=args)
    
    # 명령행 인자 확인
    if len(sys.argv) > 1:
        target = sys.argv[1]
        
        # 단일 명령 실행 모드
        node = SimpleTest()
        time.sleep(1)  # 노드 초기화 대기
        
        if target in ['apple', 'bottle', 'cup', 'banana']:
            node.get_logger().info(f'🎯 단일 테스트 실행: {target}')
            node.send_command('reset')
            time.sleep(2)
            node.send_command('start', target)
        elif target == 'reset':
            node.send_command('reset')
        elif target == 'home':
            node.send_command('home')
        else:
            node.get_logger().error(f'❌ 알 수 없는 대상: {target}')
            node.get_logger().info('💡 사용법: ros2 run robot_arm_test simple_test [apple|bottle|cup|banana|reset|home]')
            
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('🛑 단일 테스트 종료')
    else:
        # 자동 테스트 모드
        node = SimpleTest()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('🛑 자동 테스트 종료')
        finally:
          if rclpy.ok():
              node.destroy_node()
              rclpy.shutdown()

if __name__ == '__main__':
    main()