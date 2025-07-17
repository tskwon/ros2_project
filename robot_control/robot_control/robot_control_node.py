import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
from enum import Enum

class MissionState(Enum):
    IDLE = 0
    ROBOT_MOVING = 1
    WAITING_ROBOT_COMPLETE = 2
    LIFT_OPERATING = 3
    WAITING_LIFT_COMPLETE = 4
    MISSION_COMPLETE = 5

class RobotControl(Node):
    def __init__(self):
        super().__init__('order_parser_node')
        
        # 상태 관리
        self.current_state = MissionState.IDLE
        self.current_mission = None
        self.mission_queue = []
        self.current_step = 0
        
        # Publishers
        self.robot_command_pub = self.create_publisher(Int32, '/logistics/command', 10)
        self.lift_command_pub = self.create_publisher(Int32, '/lift/floor', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        
        # Subscribers
        self.order_sub = self.create_subscription(
            String,
            'order',
            self.order_callback,
            10
        )
        
        self.robot_complete_sub = self.create_subscription(
            Int32,
            '/mission_complete',
            self.robot_complete_callback,
            10
        )
        
        self.lift_complete_sub = self.create_subscription(
            Int32,
            '/lift_complete',
            self.lift_complete_callback,
            10
        )
        
        # 상태 모니터링 타이머
        self.status_timer = self.create_timer(2.0, self.status_monitor)
        
        self.get_logger().info('🤖 순차 실행 로봇 제어 시스템이 시작되었습니다.')
        self.message_count = 0

    def order_callback(self, msg):
        """주문 수신 및 처리"""
        self.message_count += 1
        
        try:
            # JSON 파싱
            order_data = json.loads(msg.data)
            self.get_logger().info('✅ JSON 파싱 성공')
            self.get_logger().info(f'📊 데이터 타입: {type(order_data)}')
            
            # 주문 데이터 처리
            if 'order' in order_data:
                orders = order_data['order']
                self.get_logger().info(f'📦 주문 개수: {len(orders)}개')
                
                for i, item in enumerate(orders, 1):
                    self.get_logger().info(f'주문 {i}:')
                    self.get_logger().info(f'  - 상품명: {item.get("name", "없음")}')
                    self.get_logger().info(f'  - 수량: {item.get("quantity", "없음")}')
                    self.get_logger().info(f'  - 위치: {item.get("position", "없음")}')
                    self.get_logger().info(f'  - 층: {item.get("floor", "없음")}')
                    
                    # 미션 큐에 추가
                    self.add_mission_to_queue(item)
                    
                # 첫 번째 미션 실행
                if self.current_state == MissionState.IDLE:
                    self.execute_next_mission()
            else:
                self.get_logger().warning('⚠️ "order" 키가 없습니다.')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ JSON 파싱 오류: {e}')
            self.get_logger().error(f'🔍 원본 데이터: "{msg.data}"')
        except Exception as e:
            self.get_logger().error(f'❌ 메시지 처리 중 오류: {e}')

    def add_mission_to_queue(self, item):
        """미션을 큐에 추가"""
        # 문자열을 정수로 안전하게 변환
        try:
            position = int(item.get("position", 1))
        except (ValueError, TypeError):
            position = 1
            
        try:
            floor = int(item.get("floor", 1))
        except (ValueError, TypeError):
            floor = 1
            
        try:
            quantity = int(item.get("quantity", 1))
        except (ValueError, TypeError):
            quantity = 1
        
        mission = {
            'name': item.get("name", "unknown"),
            'quantity': quantity,
            'position': position,
            'floor': floor,
            'steps': [
                {'type': 'robot', 'action': 'move', 'position': position},
                {'type': 'lift', 'floor': floor}
            ]
        }
        
        self.mission_queue.append(mission)
        self.get_logger().info(f'📋 미션 큐에 추가: {mission["name"]} (큐 크기: {len(self.mission_queue)})')

    def execute_next_mission(self):
        """큐에서 다음 미션 실행"""
        if not self.mission_queue:
            self.get_logger().info('📭 미션 큐가 비어있습니다.')
            return
        
        self.current_mission = self.mission_queue.pop(0)
        self.current_step = 0
        self.get_logger().info(f'🎯 새 미션 시작: {self.current_mission["name"]}')
        
        self.execute_current_step()

    def execute_current_step(self):
        """현재 단계 실행"""
        if not self.current_mission or self.current_step >= len(self.current_mission['steps']):
            self.complete_mission()
            return
        
        step = self.current_mission['steps'][self.current_step]
        
        if step['type'] == 'robot':
            self.execute_robot_step(step)
        elif step['type'] == 'lift':
            self.execute_lift_step(step)

    def execute_robot_step(self, step):
        """로봇 단계 실행"""
        action = step['action']
        position = step['position']
        
        self.get_logger().info(f'🤖 로봇 동작 시작: {action} at position {position}')
        self.current_state = MissionState.ROBOT_MOVING
        
        # 로봇에게 명령 전송
        cmd_msg = Int32()
        cmd_msg.data = position
        self.robot_command_pub.publish(cmd_msg)
        
        # 상태 업데이트
        self.current_state = MissionState.WAITING_ROBOT_COMPLETE
        
        # 상태 발행
        status_msg = String()
        status_msg.data = f"로봇 {action} 동작 중 (위치: {position})"
        self.status_pub.publish(status_msg)

    def execute_lift_step(self, step):
        """리프트 단계 실행"""
        floor = step['floor']
        
        self.get_logger().info(f'🛗 리프트 동작 시작: {floor}층으로 이동')
        self.current_state = MissionState.LIFT_OPERATING
        
        # 리프트에게 명령 전송
        lift_msg = Int32()
        lift_msg.data = floor
        self.lift_command_pub.publish(lift_msg)
        
        # 상태 업데이트
        self.current_state = MissionState.WAITING_LIFT_COMPLETE
        
        # 상태 발행
        status_msg = String()
        status_msg.data = f"리프트 {floor}층 이동 중"
        self.status_pub.publish(status_msg)

    def robot_complete_callback(self, msg):
        """로봇 완료 신호 수신"""
        if self.current_state == MissionState.WAITING_ROBOT_COMPLETE:
            self.get_logger().info(f'✅ 로봇 동작 완료 (응답: {msg.data})')
            self.current_step += 1
            self.execute_current_step()
        else:
            self.get_logger().warning(f'⚠️ 예상치 못한 로봇 완료 신호 (현재 상태: {self.current_state})')

    def lift_complete_callback(self, msg):
        """리프트 완료 신호 수신"""
        if self.current_state == MissionState.WAITING_LIFT_COMPLETE:
            self.get_logger().info(f'✅ 리프트 동작 완료 (응답: {msg.data})')
            self.current_step += 1
            self.execute_current_step()
        else:
            self.get_logger().warning(f'⚠️ 예상치 못한 리프트 완료 신호 (현재 상태: {self.current_state})')

    def complete_mission(self):
        """미션 완료 처리"""
        if self.current_mission:
            self.get_logger().info(f'🎉 미션 완료: {self.current_mission["name"]}')
            
            # 완료 상태 발행
            status_msg = String()
            status_msg.data = f"미션 완료: {self.current_mission['name']}"
            self.status_pub.publish(status_msg)
        
        self.current_mission = None
        self.current_step = 0
        self.current_state = MissionState.IDLE
        
        # 다음 미션 실행
        if self.mission_queue:
            self.get_logger().info(f'📋 다음 미션 실행 (남은 미션: {len(self.mission_queue)}개)')
            self.execute_next_mission()
        else:
            self.get_logger().info('🏁 모든 미션 완료!')

    def status_monitor(self):
        """상태 모니터링"""
        if self.current_state != MissionState.IDLE:
            current_mission_name = self.current_mission['name'] if self.current_mission else 'None'
            self.get_logger().info(f'📊 상태: {self.current_state.name} | 미션: {current_mission_name} | 단계: {self.current_step + 1}/{len(self.current_mission["steps"]) if self.current_mission else 0} | 큐: {len(self.mission_queue)}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 노드가 종료됩니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()