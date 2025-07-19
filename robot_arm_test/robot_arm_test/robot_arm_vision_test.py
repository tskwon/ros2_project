#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import Point
import json
import time
import math
from enum import Enum

class TestState(Enum):
    IDLE = 0
    YOLO_DETECTING = 1
    WAITING_YOLO_COMPLETE = 2
    ROBOT_ARM_MOVING = 3
    WAITING_ROBOT_ARM_COMPLETE = 4
    ROBOT_ARM_DELIVERY = 5
    WAITING_ROBOT_ARM_DELIVERY_COMPLETE = 6
    TEST_COMPLETE = 7

class RobotArmVisionTest(Node):
    def __init__(self):
        super().__init__('robot_arm_vision_test_node')
        
        # 상태 관리
        self.current_state = TestState.IDLE
        self.detected_objects = []
        self.current_object_index = 0
        self.test_target = "apple"  # 기본 테스트 대상
        
        # Publishers
        self.robot_arm_position_pub = self.create_publisher(Point, '/robot_arm/target_position', 10)
        self.yolo_trigger_pub = self.create_publisher(String, '/yolo/detection_trigger', 10)
        self.gripper_control_pub = self.create_publisher(Bool, '/robot_arm/gripper_control', 10)
        self.status_pub = self.create_publisher(String, '/test/status', 10)
        
        # Subscribers
        self.robot_arm_complete_sub = self.create_subscription(
            Int32,
            '/robot_arm/mission_complete',
            self.robot_arm_complete_callback,
            10
        )
        
        self.yolo_result_sub = self.create_subscription(
            String,
            '/yolo/detection_result',
            self.yolo_result_callback,
            10
        )
        
        # 테스트 시작 명령 구독
        self.test_command_sub = self.create_subscription(
            String,
            '/test/command',
            self.test_command_callback,
            10
        )
        
        # 상태 모니터링 타이머
        self.status_timer = self.create_timer(2.0, self.status_monitor)
        
        self.get_logger().info('🤖 로봇팔 카메라 동작 테스트 노드가 시작되었습니다.')
        self.get_logger().info('📋 테스트 명령어:')
        self.get_logger().info('  - {"command": "start", "target": "apple", "quantity": 3} : 테스트 시작')
        self.get_logger().info('  - {"command": "reset"} : 테스트 리셋')
        self.get_logger().info('  - {"command": "home"} : 로봇팔 홈 위치로 이동')

    def test_command_callback(self, msg):
        """테스트 명령 수신"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')
            
            if command == 'start':
                target = command_data.get('target', 'apple')
                quantity = command_data.get('quantity', 1)  # 기본 수량 1개
                self.start_vision_test(target, quantity)
            elif command == 'reset':
                self.reset_test()
            elif command == 'home':
                self.move_arm_to_home()
            else:
                self.get_logger().warning(f'⚠️ 알 수 없는 명령: {command}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ 명령 파싱 오류: {e}')

    def start_vision_test(self, target_name, quantity=1):
        """비전 테스트 시작"""
        if self.current_state != TestState.IDLE:
            self.get_logger().warning('⚠️ 이미 테스트가 진행 중입니다.')
            return
            
        self.test_target = target_name
        self.detected_objects = []
        self.current_object_index = 0
        self.target_quantity = quantity  # 목표 수량 추가
        self.processed_quantity = 0      # 처리된 수량 추가
        
        self.get_logger().info(f'🎯 비전 테스트 시작: {target_name} 검출 (목표 수량: {quantity})')
        self.publish_status(f"비전 테스트 시작: {target_name} (목표: {quantity}개)")
        
        # YOLO 검출 시작
        self.execute_yolo_detection()

    def execute_yolo_detection(self):
        """YOLO 물체 인식 실행"""
        self.get_logger().info(f'📷 YOLO 물체 인식 시작: {self.test_target}')
        self.current_state = TestState.YOLO_DETECTING
        
        # YOLO 트리거 전송
        trigger_msg = String()
        trigger_data = {
            'target': self.test_target,
            'action': 'detect',
            'timestamp': time.time()
        }
        trigger_msg.data = json.dumps(trigger_data)
        self.yolo_trigger_pub.publish(trigger_msg)
        
        self.current_state = TestState.WAITING_YOLO_COMPLETE
        self.publish_status(f"YOLO 검출 중: {self.test_target}")
        
        self.get_logger().info(f'🔍 YOLO 검출 트리거 전송: {self.test_target}')

    def camera_to_robot_coordinates(self, pixel_x, pixel_y, angle_deg):
        # Bilinear 보간
        robot_x = (1 / 15) * pixel_y - (113 / 15)
        robot_y = (-19 / 286) * pixel_x + (12697 / 286)
        
        return robot_x, robot_y, angle_deg - 180.0

    def yolo_result_callback(self, msg):
        """YOLO 검출 결과 수신"""
        if self.current_state == TestState.WAITING_YOLO_COMPLETE:
            try:
                result_data = json.loads(msg.data)
                self.get_logger().info(f'✅ YOLO 검출 완료: {result_data.get("target", "unknown")}')
                
                if 'objects' in result_data and result_data['objects']:
                    objects = result_data['objects']
                    detected_objects_with_distance = []
                    
                    # 카메라 중앙 좌표 (640x480 해상도 기준)
                    camera_center_x = 320.0
                    camera_center_y = 240.0
                    
                    for i, obj in enumerate(objects):
                        center_x = obj['pixel_x']
                        center_y = obj['pixel_y']
                        angle = obj['angle']
                        depth_mm = obj.get('depth_mm', None)
                        
                        # 카메라 좌표를 로봇 좌표로 변환
                        robot_x, robot_y, robot_angle = self.camera_to_robot_coordinates(center_x, center_y, angle)
                        
                        
                        # 카메라 중앙으로부터의 거리 계산
                        distance_from_center = math.sqrt(
                            (center_x - camera_center_x)**2 + 
                            (center_y - camera_center_y)**2
                        )
                        
                        detected_object = {
                            'id': i,
                            'pixel_x': center_x,
                            'pixel_y': center_y,
                            'pixel_angle': angle,
                            'depth_mm': depth_mm,
                            'robot_x': robot_x,
                            'robot_y': robot_y,
                            'robot_angle': robot_angle,
                            'distance_from_center': distance_from_center
                        }
                        
                        detected_objects_with_distance.append(detected_object)
                        
                        depth_info = f", depth={depth_mm:.0f}mm" if depth_mm is not None else ""
                        self.get_logger().info(f'🎯 물체 #{i+1}: 픽셀({center_x:.0f}, {center_y:.0f}, {angle:.0f}°{depth_info}) → 로봇({robot_x:.2f}, {robot_y:.2f}, {robot_angle:.1f}°) ✅')
                    
                    # 카메라 중앙으로부터의 거리 기준으로 정렬 (가까운 순)
                    detected_objects_with_distance.sort(key=lambda x: x['distance_from_center'])
                    self.detected_objects = detected_objects_with_distance
                    
                    total_detected = len(objects)
                    safe_objects = len(detected_objects_with_distance)
                    filtered_out = total_detected - safe_objects
                    
                    self.get_logger().info(f'✅ 총 {total_detected}개 검출, {safe_objects}개 안전 범위 내, {filtered_out}개 필터링됨')
                    
                    if self.detected_objects:
                        # 첫 번째 안전한 객체로 로봇 팔 이동
                        self.current_object_index = 0
                        detected_object = self.detected_objects[0]
                        self.get_logger().info(f'🎯 첫 번째 안전한 물체 선택: ({detected_object["robot_x"]:.2f}, {detected_object["robot_y"]:.2f})')
                        self.execute_robot_arm_pickup(detected_object)
                    else:
                        self.get_logger().warning('⚠️ 안전 범위 내에 검출된 물체가 없습니다.')
                        self.complete_test()
                else:
                    self.get_logger().warning('⚠️ 검출된 물체가 없습니다.')
                    self.complete_test()
                    
            except json.JSONDecodeError as e:
                self.get_logger().error(f'❌ YOLO 결과 파싱 오류: {e}')
                self.complete_test()
            except Exception as e:
                self.get_logger().error(f'❌ YOLO 결과 처리 오류: {e}')
                self.complete_test()

    def execute_robot_arm_pickup(self, detected_object):
        """로봇 팔 물체 집기 실행"""
        x = detected_object['robot_x']
        y = detected_object['robot_y']
        z = detected_object['robot_angle']
        
        self.get_logger().info(f'🦾 로봇 팔 물체 집기 시작: ({x:.2f}, {y:.2f}, {z:.1f}°)')
        self.current_state = TestState.ROBOT_ARM_MOVING
        
        # 1. 그리퍼 열기
        self.control_gripper(False)  # False = 열기
        time.sleep(0.5)  # 그리퍼 동작 대기
        
        # 2. 물체 위치로 이동
        arm_msg = Point()
        arm_msg.x = float(x)
        arm_msg.y = float(y)
        arm_msg.z = float(z)
        self.robot_arm_position_pub.publish(arm_msg)
        
        self.current_state = TestState.WAITING_ROBOT_ARM_COMPLETE
        self.publish_status(f"로봇 팔 물체 집기 중: ({x:.2f}, {y:.2f}, {z:.1f}°)")

    def execute_robot_arm_delivery(self):
        """로봇 팔 배송 위치로 이동"""
        delivery_x = -10.0
        delivery_y = 15.0
        delivery_z = 0.0
        
        self.get_logger().info(f'🚚 로봇 팔 배송 위치로 이동: ({delivery_x}, {delivery_y})')
        self.current_state = TestState.ROBOT_ARM_DELIVERY
        
        arm_msg = Point()
        arm_msg.x = delivery_x
        arm_msg.y = delivery_y
        arm_msg.z = delivery_z
        self.robot_arm_position_pub.publish(arm_msg)
        
        self.current_state = TestState.WAITING_ROBOT_ARM_DELIVERY_COMPLETE
        self.publish_status(f"로봇 팔 배송 위치로 이동 중: ({delivery_x}, {delivery_y})")

    def control_gripper(self, close):
        """그리퍼 제어 (열기/닫기)"""
        gripper_msg = Bool()
        gripper_msg.data = close  # True = 닫기, False = 열기
        self.gripper_control_pub.publish(gripper_msg)
        action = "닫기" if close else "열기"
        self.get_logger().info(f'✋ 그리퍼 {action}')

    def robot_arm_complete_callback(self, msg):
        """로봇 팔 완료 신호 수신"""
        self.get_logger().info(f'✅ 로봇 팔 동작 완료 (응답: {msg.data}), 현재 상태: {self.current_state.name}')
        
        if self.current_state == TestState.WAITING_ROBOT_ARM_COMPLETE:
            self.get_logger().info('🔒 물체 집기 완료 - 그리퍼 닫기')
            # 그리퍼 닫기
            self.control_gripper(True)  # True = 닫기
            time.sleep(1.5)  # 그리퍼 동작 대기
            
            # 배송 위치로 이동
            self.execute_robot_arm_delivery()
            
        elif self.current_state == TestState.WAITING_ROBOT_ARM_DELIVERY_COMPLETE:
            self.get_logger().info('🎉 배송 위치 도달 완료')
            time.sleep(1.5)  # 안정화 대기
            
            # 그리퍼 열기 (물체 놓기)
            self.control_gripper(False)  # False = 열기
            time.sleep(0.5)
            
            # 처리된 수량 증가
            self.processed_quantity += 1
            self.current_object_index += 1  # 다음 객체 인덱스로 이동
            
            self.get_logger().info(f'📦 처리 완료: {self.processed_quantity}/{self.target_quantity} (객체 #{self.current_object_index})')
            
            # 목표 수량 달성 체크
            if self.processed_quantity < self.target_quantity and self.current_object_index < len(self.detected_objects):
                # 다음 객체 처리 (이미 검출된 좌표 사용)
                next_object = self.detected_objects[self.current_object_index]
                self.get_logger().info(f'🔄 다음 물체 처리: 인덱스 {self.current_object_index} → 로봇 좌표 ({next_object["robot_x"]:.2f}, {next_object["robot_y"]:.2f})')
                self.execute_robot_arm_pickup(next_object)
            elif self.processed_quantity >= self.target_quantity:
                self.get_logger().info('✅ 목표 수량 달성! 테스트 완료')
                self.complete_test()
            else:
                self.get_logger().warning(f'⚠️ 처리 가능한 객체 부족: 검출됨 {len(self.detected_objects)}개, 필요 {self.target_quantity}개')
                self.complete_test()

    def move_arm_to_home(self):
        """로봇 팔을 홈 위치로 이동"""
        self.get_logger().info('🏠 로봇 팔 홈 위치로 이동')
        
        arm_msg = Point()
        arm_msg.x = 4.0
        arm_msg.y = 0.0
        arm_msg.z = 0.0
        self.robot_arm_position_pub.publish(arm_msg)
        
        # 그리퍼도 열어둠
        self.control_gripper(False)
        
        self.publish_status("로봇 팔 홈 위치로 이동")

    def complete_test(self):
        """테스트 완료 처리"""
        self.get_logger().info('🎉 비전 테스트 완료!')
        
        # 로봇 팔 홈 위치로 이동
        time.sleep(1.5)
        self.move_arm_to_home()
        
        self.current_state = TestState.IDLE
        self.detected_objects = []
        self.current_object_index = 0
        
        self.publish_status("테스트 완료 - 대기 중")

    def reset_test(self):
        """테스트 리셋"""
        self.get_logger().info('🔄 테스트 리셋')
        
        self.current_state = TestState.IDLE
        self.detected_objects = []
        self.current_object_index = 0
        
        # 로봇 팔 홈 위치로 이동
        self.move_arm_to_home()
        
        self.publish_status("테스트 리셋 완료 - 대기 중")

    def publish_status(self, message):
        """상태 메시지 발행"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)

    def status_monitor(self):
        """상태 모니터링"""
        if self.current_state != TestState.IDLE:
            progress = f"{getattr(self, 'processed_quantity', 0)}/{getattr(self, 'target_quantity', 1)}"
            current_idx = getattr(self, 'current_object_index', 0)
            total_detected = len(getattr(self, 'detected_objects', []))
            
            self.get_logger().info(f'📊 상태: {self.current_state.name} | 대상: {self.test_target} | 진행: {progress} | 객체: {current_idx}/{total_detected}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotArmVisionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 테스트 노드가 종료됩니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()