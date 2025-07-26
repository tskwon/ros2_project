#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class TestCommander(Node):
    def __init__(self):
        super().__init__('test_commander_node')
        
        # 테스트 명령 발행자
        self.command_pub = self.create_publisher(String, '/test/command', 10)
        
        # 상태 구독자
        self.status_sub = self.create_subscription(
            String,
            '/test/status',
            self.status_callback,
            10
        )
        
        self.get_logger().info('🎮 테스트 명령 노드가 시작되었습니다.')
        self.get_logger().info('📋 사용 가능한 명령어:')
        self.print_help()
        
        # 사용자 입력을 위한 타이머
        self.input_timer = self.create_timer(1.0, self.check_user_input)
        self.input_active = True

    def print_help(self):
        """도움말 출력"""
        help_text = [
            '  1. start [target] [quantity] - 테스트 시작 (기본: apple 1개)',
            '  2. reset - 테스트 리셋',
            '  3. home - 로봇팔 홈으로 이동',
            '  4. help - 도움말 출력',
            '  5. quit - 종료',
            '',
            '예시:',
            '  start apple 3      # 사과 3개 집기',
            '  start bottle 2     # 병 2개 집기',
            '  start cup          # 컵 1개 집기 (기본)',
            '  reset',
            '  home'
        ]
        for line in help_text:
            self.get_logger().info(line)

    def status_callback(self, msg):
        """상태 메시지 수신"""
        self.get_logger().info(f'📊 상태 업데이트: {msg.data}')

    def send_command(self, command, target=None, quantity=None):
        """명령 전송"""
        command_data = {'command': command}
        if target:
            command_data['target'] = target
        if quantity:
            command_data['quantity'] = quantity
            
        msg = String()
        msg.data = json.dumps(command_data)
        self.command_pub.publish(msg)
        
        self.get_logger().info(f'📤 명령 전송: {command_data}')

    def check_user_input(self):
        """사용자 입력 확인 (비블로킹)"""
        if not self.input_active:
            return
            
        try:
            import select
            import sys
            
            # Linux/Unix 시스템에서만 작동
            if select.select([sys.stdin], [], [], 0.0)[0]:
                user_input = sys.stdin.readline().strip()
                self.process_user_input(user_input)
        except:
            # Windows나 다른 환경에서는 간단한 메뉴 제공
            pass

    def process_user_input(self, user_input):
        """사용자 입력 처리"""
        if not user_input:
            return
            
        parts = user_input.split()
        command = parts[0].lower()
        
        if command == 'start':
            target = parts[1] if len(parts) > 1 else 'apple'
            quantity = 1  # 기본값
            
            # 수량 파라미터 처리
            if len(parts) > 2:
                try:
                    quantity = int(parts[2])
                    if quantity <= 0:
                        self.get_logger().warning('⚠️ 수량은 1 이상이어야 합니다. 기본값 1을 사용합니다.')
                        quantity = 1
                    elif quantity > 10:
                        self.get_logger().warning('⚠️ 수량이 너무 큽니다. 최대 10개로 제한합니다.')
                        quantity = 10
                except ValueError:
                    self.get_logger().warning(f'⚠️ 잘못된 수량 형식: {parts[2]}. 기본값 1을 사용합니다.')
                    quantity = 1
            
            self.get_logger().info(f'🎯 테스트 시작: {target} {quantity}개')
            self.send_command('start', target, quantity)
            
        elif command == 'reset':
            self.send_command('reset')
        elif command == 'home':
            self.send_command('home')
        elif command == 'help':
            self.print_help()
        elif command == 'quit' or command == 'exit':
            self.get_logger().info('👋 테스트 명령 노드를 종료합니다.')
            rclpy.shutdown()
        else:
            self.get_logger().warning(f'⚠️ 알 수 없는 명령: {command}')
            self.get_logger().info('💡 "help"를 입력하여 사용법을 확인하세요.')

    def run_interactive_mode(self):
        """대화형 모드 실행"""
        self.get_logger().info('🎮 대화형 모드 시작 (명령어 입력 후 Enter)')
        
        try:
            while rclpy.ok():
                user_input = input('테스트 명령 입력> ').strip()
                if user_input:
                    self.process_user_input(user_input)
                    if user_input.lower() in ['quit', 'exit']:
                        break
        except KeyboardInterrupt:
            self.get_logger().info('🛑 사용자가 종료했습니다.')
        except EOFError:
            self.get_logger().info('🛑 입력 종료')

def main(args=None):
    rclpy.init(args=args)
    node = TestCommander()
    
    try:
        # 별도 스레드에서 ROS 스핀
        import threading
        spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
        spin_thread.start()
        
        # 메인 스레드에서 대화형 모드 실행
        node.run_interactive_mode()
        
    except KeyboardInterrupt:
        node.get_logger().info('🛑 노드가 종료됩니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()