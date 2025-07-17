import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import Point
import json
import time
import math
import numpy as np
from enum import Enum

class MissionState(Enum):
    IDLE = 0
    ROBOT_MOVING = 1
    WAITING_ROBOT_COMPLETE = 2
    LIFT_OPERATING = 3
    WAITING_LIFT_COMPLETE = 4
    YOLO_DETECTING = 5
    WAITING_YOLO_COMPLETE = 6
    ROBOT_ARM_MOVING = 7
    WAITING_ROBOT_ARM_COMPLETE = 8
    LIFT_UP_OPERATING = 9
    WAITING_LIFT_UP_COMPLETE = 10
    LIFT_DOWN_OPERATING = 11
    WAITING_LIFT_DOWN_COMPLETE = 12
    ROBOT_ARM_DELIVERY = 13
    WAITING_ROBOT_ARM_DELIVERY_COMPLETE = 14
    MISSION_COMPLETE = 15

class RobotControl(Node):
    def __init__(self):
        super().__init__('order_parser_node')
        
        # 상태 관리
        self.current_state = MissionState.IDLE
        self.current_mission = None
        self.mission_queue = []
        self.current_step = 0
        self.current_quantity_processed = 0
        self.detected_objects = []
        
        # 카메라 좌표 → 로봇 팔 좌표 변환 파라미터 (임의 설정)
        self.camera_to_robot_params = {
            'scale_x': 0.05,    # 픽셀당 cm 변환 비율
            'scale_y': 0.05,
            'offset_x': -32.0,  # 카메라 중심(640/2=320)을 로봇 좌표 0으로 변환
            'offset_y': -24.0,  # 카메라 중심(480/2=240)을 로봇 좌표 0으로 변환
            'base_x': 0.0,      # 로봇 팔 베이스 좌표
            'base_y': 30.0      # 로봇 팔 베이스 좌표
        }
        
        # Publishers
        self.robot_command_pub = self.create_publisher(Int32, '/logistics/command', 10)
        self.lift_command_pub = self.create_publisher(Int32, '/lift/floor', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.robot_arm_position_pub = self.create_publisher(Point, '/robot_arm/target_position', 10)
        self.yolo_trigger_pub = self.create_publisher(String, '/yolo/detection_trigger', 10)
        self.gripper_control_pub = self.create_publisher(Bool, '/robot_arm/gripper_control', 10)
        
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
        
        self.robot_arm_complete_sub = self.create_subscription(
            Int32,
            '/robot_arm/mission_complete',  # 로봇팔 완료 신호
            self.robot_arm_complete_callback,
            10
        )
        
        # YOLO 검출 결과 구독
        self.yolo_result_sub = self.create_subscription(
            String,
            '/yolo/detection_result',
            self.yolo_result_callback,
            10
        )
        
        # 상태 모니터링 타이머
        self.status_timer = self.create_timer(2.0, self.status_monitor)
        
        self.get_logger().info('🤖 YOLO 물체 인식 및 로봇 팔 제어 시스템이 시작되었습니다.')
        self.message_count = 0

    def order_callback(self, msg):
        """주문 수신 및 처리"""
        # 이미 처리 중인 상태면 새로운 주문 무시
        if self.current_state != MissionState.IDLE:
            self.get_logger().warning('⚠️ 현재 처리 중인 미션이 있어 새로운 주문을 무시합니다.')
            return
            
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
            'target_name': item.get("name", "unknown")  # YOLO 검출 대상
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
        self.current_quantity_processed = 0
        self.detected_objects = []
        
        self.get_logger().info(f'🎯 새 미션 시작: {self.current_mission["name"]} (수량: {self.current_mission["quantity"]})')
        
        # 1단계: 로봇 이동
        # self.execute_robot_movement()
        self.execute_lift_operation()

    def execute_robot_movement(self):
        """로봇 이동 단계"""
        position = self.current_mission['position']
        
        self.get_logger().info(f'🤖 로봇 이동 시작: position {position}')
        self.current_state = MissionState.ROBOT_MOVING
        
        # 로봇에게 명령 전송
        cmd_msg = Int32()
        cmd_msg.data = position
        self.robot_command_pub.publish(cmd_msg)
        
        # 상태 업데이트
        self.current_state = MissionState.WAITING_ROBOT_COMPLETE
        
        # 상태 발행
        status_msg = String()
        status_msg.data = f"로봇 이동 중 (위치: {position})"
        self.status_pub.publish(status_msg)

    def execute_lift_operation(self):
        """리프트 동작 단계"""
        floor = self.current_mission['floor']
        
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

    def camera_to_robot_coordinates(self, pixel_x, pixel_y, angle_deg):
        """카메라 픽셀 좌표를 로봇 팔 좌표로 변환"""
        
        robot_angle = angle_deg
        
        robot_y = 0.0650 * pixel_x - 0.0425 * pixel_y + 12.90
        robot_x = -0.0086 * pixel_x + 0.0707 * pixel_y - 6.63

        self.get_logger().info(f'좌표 변환: 픽셀({pixel_x}, {pixel_y}, {angle_deg}°) → 로봇({robot_x:.1f}, {robot_y:.1f}, {robot_angle:.1f}°)')
        
        return robot_x, robot_y, robot_angle

    def detect_objects_with_yolo(self, target_name):
        """YOLO를 사용한 실제 물체 검출 (트리거 전송)"""
        try:
            self.get_logger().info(f'YOLO 검출 시작: {target_name}')
            
            # YOLO 트리거 전송
            trigger_msg = String()
            trigger_data = {
                'target': target_name,
                'action': 'detect',
                'timestamp': time.time()
            }
            trigger_msg.data = json.dumps(trigger_data)
            self.yolo_trigger_pub.publish(trigger_msg)
            
            self.get_logger().info(f'🔍 YOLO 검출 트리거 전송: {target_name}')
            
            # 결과는 yolo_result_callback에서 처리됨
            return None
            
        except Exception as e:
            self.get_logger().error(f'YOLO 검출 오류: {str(e)}')
            return []

    def execute_yolo_detection(self):
        """YOLO 물체 인식 단계"""
        target_name = self.current_mission['target_name']
        
        self.get_logger().info(f'📷 YOLO 물체 인식 시작: {target_name} 검출')
        self.current_state = MissionState.YOLO_DETECTING
        
        # 상태 발행
        status_msg = String()
        status_msg.data = f"YOLO 물체 인식 중: {target_name}"
        self.status_pub.publish(status_msg)
        
        # YOLO 검출 트리거 전송
        trigger_msg = String()
        trigger_data = {
            'target': target_name,
            'action': 'detect',
            'timestamp': time.time()
        }
        trigger_msg.data = json.dumps(trigger_data)
        self.yolo_trigger_pub.publish(trigger_msg)
        
        # 상태 업데이트
        self.current_state = MissionState.WAITING_YOLO_COMPLETE
        
        self.get_logger().info(f'🔍 YOLO 검출 트리거 전송: {target_name}')

    def execute_robot_arm_pickup(self, detected_object):
        """로봇 팔 물체 집기 단계"""
        x = detected_object['robot_x']
        y = detected_object['robot_y']
        z = detected_object['robot_angle']
        
        self.get_logger().info(f'🦾 로봇 팔 물체 집기: ({x:.1f}, {y:.1f}, {z:.1f}°)')
        self.current_state = MissionState.ROBOT_ARM_MOVING
        
        # 1. 그리퍼 열기
        self.control_gripper(False)  # False = 열기
        
        # 2. 물체 위치로 이동 (위치 + 회전)
        arm_msg = Point()
        arm_msg.x = float(x)
        arm_msg.y = float(y)
        arm_msg.z = float(z)
        self.robot_arm_position_pub.publish(arm_msg)
        
        
        # 상태 업데이트
        self.current_state = MissionState.WAITING_ROBOT_ARM_COMPLETE
        
        # 상태 발행
        status_msg = String()
        status_msg.data = f"로봇 팔 물체 집기 중: ({x:.1f}, {y:.1f}, {z:.1f}°)"
        self.status_pub.publish(status_msg)
        
    def control_gripper(self, close):
        """그리퍼 제어 (열기/닫기)"""
        gripper_msg = Bool()
        gripper_msg.data = close  # True = 닫기, False = 열기
        self.gripper_control_pub.publish(gripper_msg)
        action = "닫기" if close else "열기"        
    

        
    def open_gripper_for_delivery(self):
        """배송 위치에서 그리퍼 열기 (물체 놓기)"""
        self.control_gripper(False)  # False = 열기
        self.get_logger().info('✋ 배송 완료 - 그리퍼 열기')

    def execute_lift_up(self):
        """리프트 위로 동작 (8번 전송)"""
        self.get_logger().info('🛗 리프트 위로 동작 시작 (8번)')
        self.current_state = MissionState.LIFT_UP_OPERATING
        
        # 리프트에게 8번 명령 전송
        lift_msg = Int32()
        lift_msg.data = 8
        self.lift_command_pub.publish(lift_msg)
        
        # 상태 업데이트
        self.current_state = MissionState.WAITING_LIFT_UP_COMPLETE
        
        # 상태 발행
        status_msg = String()
        status_msg.data = "리프트 위로 동작 중 (8번)"
        self.status_pub.publish(status_msg)

    def execute_lift_down(self):
        """리프트 아래로 동작 (9번 전송)"""
        self.get_logger().info('🛗 리프트 아래로 동작 시작 (9번)')
        self.current_state = MissionState.LIFT_DOWN_OPERATING
        
        # 리프트에게 9번 명령 전송
        lift_msg = Int32()
        lift_msg.data = 9
        self.lift_command_pub.publish(lift_msg)
        
        # 상태 업데이트
        self.current_state = MissionState.WAITING_LIFT_DOWN_COMPLETE
        
        # 상태 발행
        status_msg = String()
        status_msg.data = "리프트 아래로 동작 중 (9번)"
        self.status_pub.publish(status_msg)

    def execute_robot_arm_delivery(self):
        """로봇 팔 배송 위치로 이동 (15, -15)"""
        delivery_x = -15.0
        delivery_y = 20.0
        
        self.get_logger().info(f'🦾 로봇 팔 배송 위치로 이동: ({delivery_x}, {delivery_y})')
        self.current_state = MissionState.ROBOT_ARM_DELIVERY
        
        # 로봇 팔에게 배송 위치 명령 전송
        arm_msg = Point()
        arm_msg.x = delivery_x
        arm_msg.y = delivery_y
        arm_msg.z = 0.0  # 배송 시에는 회전 없음
        self.robot_arm_position_pub.publish(arm_msg)
        
        # 상태 업데이트
        self.current_state = MissionState.WAITING_ROBOT_ARM_DELIVERY_COMPLETE
        
        # 상태 발행
        status_msg = String()
        status_msg.data = f"로봇 팔 배송 위치로 이동 중: ({delivery_x}, {delivery_y})"
        self.status_pub.publish(status_msg)
        
            
    def open_gripper_for_delivery(self):
        """배송 위치에서 그리퍼 열기 (물체 놓기)"""
        self.control_gripper(False)  # False = 열기
        self.get_logger().info('✋ 배송 완료 - 그리퍼 열기')

    def robot_complete_callback(self, msg):
        """로봇 완료 신호 수신"""
        if self.current_state == MissionState.WAITING_ROBOT_COMPLETE:
            self.get_logger().info(f'✅ 로봇 이동 완료 (응답: {msg.data})')
            # 다음 단계: 리프트 동작
            self.execute_lift_operation()
        else:
            self.get_logger().debug(f'로봇 완료 신호 수신 (현재 상태: {self.current_state})')

    def lift_complete_callback(self, msg):
        """리프트 완료 신호 수신"""
        self.get_logger().info(f'🛗 리프트 완료 신호 수신: {msg.data}, 현재 상태: {self.current_state}')
        
        if self.current_state == MissionState.WAITING_LIFT_COMPLETE:
            self.get_logger().info(f'✅ 리프트 동작 완료 (응답: {msg.data})')
            # 다음 단계: YOLO 물체 인식
            self.execute_yolo_detection()
            
        elif self.current_state == MissionState.WAITING_LIFT_UP_COMPLETE:
            self.get_logger().info(f'✅ 리프트 위로 동작 완료 (응답: {msg.data})')
            # 그리퍼 닫기
            # 딜레이
            time.sleep(3)
            
            self.control_gripper(True) 
            # 다음 단계: 리프트 아래로 동작
            time.sleep(3)
            self.execute_lift_down()
            
        elif self.current_state == MissionState.WAITING_LIFT_DOWN_COMPLETE:
            self.get_logger().info(f'✅ 리프트 아래로 동작 완료 (응답: {msg.data})')
            # 다음 단계: 로봇 팔 배송 
            time.sleep(3)
            self.execute_robot_arm_delivery()
        else:
            self.get_logger().debug(f'리프트 완료 신호 수신 (현재 상태: {self.current_state})')

    def yolo_result_callback(self, msg):
        """YOLO 검출 결과 수신"""
        if self.current_state == MissionState.WAITING_YOLO_COMPLETE:
            try:
                result_data = json.loads(msg.data)
                self.get_logger().info(f'✅ YOLO 검출 완료: {result_data.get("target", "unknown")}')
                
                # 검출된 객체 저장
                if 'objects' in result_data and result_data['objects']:
                    objects = result_data['objects']
                    detected_objects_with_distance = []
                    
                    # 카메라 중앙 좌표 (640x480 해상도 기준)
                    camera_center_x = 320.0
                    camera_center_y = 240.0
                    
                    for i, obj in enumerate(objects):
                        # YOLO 노드에서 오는 데이터 형식에 맞게 처리
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
                        self.get_logger().info(f'물체 #{i+1} 검출: 픽셀({center_x:.0f}, {center_y:.0f}, {angle:.0f}°{depth_info}) → 로봇({robot_x:.1f}, {robot_y:.1f}, {robot_angle:.1f}°)')
                    
                    # 카메라 중앙으로부터의 거리 기준으로 정렬 (가까운 순)
                    detected_objects_with_distance.sort(key=lambda x: x['distance_from_center'])
                    
                    # 현재 미션의 수량만큼만 선택
                    required_quantity = self.current_mission['quantity'] - self.current_quantity_processed
                    selected_objects = detected_objects_with_distance[:required_quantity]
                    
                    self.detected_objects = selected_objects
                    
                    self.get_logger().info(f'✅ {len(detected_objects_with_distance)}개 물체 검출, {len(selected_objects)}개 선택 (카메라 중앙 기준 정렬)')
                    
                    if selected_objects:
                        # 첫 번째 선택된 객체로 로봇 팔 이동
                        detected_object = selected_objects[0]
                        self.execute_robot_arm_pickup(detected_object)
                    else:
                        self.get_logger().warning('⚠️ 선택된 물체가 없습니다.')
                        self.complete_mission()
                else:
                    self.get_logger().warning('⚠️ 검출된 물체가 없습니다.')
                    self.complete_mission()
                    
            except json.JSONDecodeError as e:
                self.get_logger().error(f'❌ YOLO 결과 파싱 오류: {e}')
                self.complete_mission()
            except Exception as e:
                self.get_logger().error(f'❌ YOLO 결과 처리 오류: {e}')
                self.complete_mission()
        else:
            self.get_logger().debug(f'YOLO 결과 수신 (현재 상태: {self.current_state})')


        
    def robot_arm_complete_callback(self, msg):
        """로봇 팔 완료 신호 수신"""
        if self.current_state == MissionState.WAITING_ROBOT_ARM_COMPLETE:
            self.get_logger().info(f'✅ 로봇 팔 이동 완료 (응답: {msg.data})')
            # 다음 단계: 리프트 위로 동작
            time.sleep(2)
            self.execute_lift_up()
            
        elif self.current_state == MissionState.WAITING_ROBOT_ARM_DELIVERY_COMPLETE:
            self.get_logger().info(f'✅ 로봇 팔 배송 완료 (응답: {msg.data})')
            self.current_quantity_processed += 1
            
            # 로봇팔 도착했을떄 그리퍼 열리는 속도
            time.sleep(0.5)
            self.open_gripper_for_delivery()
            
            # 수량 체크
            if self.current_quantity_processed < self.current_mission['quantity']:
                self.get_logger().info(f'📦 남은 수량: {self.current_mission["quantity"] - self.current_quantity_processed}')
                # 다음 물체 처리를 위해 YOLO 검출 재시작
                self.execute_yolo_detection()
            else:
                self.get_logger().info(f'✅ 모든 수량 처리 완료: {self.current_quantity_processed}/{self.current_mission["quantity"]}')
                time.sleep(2)
                # 로봇 팔 원점으로 이동 (x=0, y=4, z=0)
                arm_msg = Point()
                arm_msg.x = 0.0
                arm_msg.y = 4.0
                arm_msg.z = 0.0
                self.robot_arm_position_pub.publish(arm_msg)
                self.get_logger().info('🔄 로봇 팔 원점 위치로 이동: (0.0, 4.0, 0.0)')

                self.complete_mission()
        else:
            self.get_logger().debug(f'로봇 팔 완료 신호 수신 (현재 상태: {self.current_state})')

    def complete_mission(self):
        """미션 완료 처리"""
        if self.current_mission:
            self.get_logger().info(f'🎉 미션 완료: {self.current_mission["name"]} (처리 수량: {self.current_quantity_processed})')
            
            # 완료 상태 발행
            status_msg = String()
            status_msg.data = f"미션 완료: {self.current_mission['name']} (처리 수량: {self.current_quantity_processed})"
            self.status_pub.publish(status_msg)
        
        self.current_mission = None
        self.current_step = 0
        self.current_quantity_processed = 0
        self.detected_objects = []
        self.current_state = MissionState.IDLE
        arm_msg = Point()
        arm_msg.x = float(4.0)
        arm_msg.y = float(0.0)
        arm_msg.z = float(0.0)  # 로봇 팔 초기 위치로 이동
        self.get_logger().info('🦾 로봇 팔 초기 위치로 이동')
        self.robot_arm_position_pub.publish(arm_msg)
        
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
            processed_quantity = f"{self.current_quantity_processed}/{self.current_mission['quantity']}" if self.current_mission else '0/0'
            self.get_logger().info(f'📊 상태: {self.current_state.name} | 미션: {current_mission_name} | 수량: {processed_quantity} | 큐: {len(self.mission_queue)}')

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