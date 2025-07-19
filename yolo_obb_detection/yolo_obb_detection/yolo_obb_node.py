#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
import math
import json
from collections import deque

class YoloObbNode(Node):
    def __init__(self):
        super().__init__('yolo_obb_node')
        
        # 파라미터
        self.declare_parameter('model_path', '/home/xotn/ros2_ws/src/yolo_obb_detection/models/best.pt')
        self.declare_parameter('confidence_threshold', 0.80)
        self.declare_parameter('display_enabled', False)  # 디스플레이 옵션 추가
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.display_enabled = self.get_parameter('display_enabled').get_parameter_value().bool_value
        
        # YOLO 모델 로드
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO model loaded: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            return
        
        self.bridge = CvBridge()
        
        # 최신 이미지 저장용
        self.latest_color_image = None
        self.latest_depth_image = None
        
        # 뎁스 값 안정화를 위한 버퍼 (최근 5개 값의 평균)
        self.depth_buffer = deque(maxlen=5)
        
        # 구독자들
        self.color_sub = self.create_subscription(
            Image,
            '/d415/realsense_d415/color/image_raw',
            self.color_callback,
            1
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/d415/realsense_d415/depth/image_rect_raw',
            self.depth_callback,
            1
        )
        
        # 검출 트리거 구독
        self.detection_trigger_sub = self.create_subscription(
            String,
            '/yolo/detection_trigger',
            self.detection_trigger_callback,
            10
        )
        
        # 퍼블리셔들
        self.result_pub = self.create_publisher(Image, '/yolo_result', 1)
        self.detection_result_pub = self.create_publisher(String, '/yolo/detection_result', 10)
        
        # OpenCV 윈도우 설정 (다시 활성화)
        if self.display_enabled:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Detection Info', cv2.WINDOW_AUTOSIZE)
            self.get_logger().info('🖥️ OpenCV 디스플레이가 활성화되었습니다.')
        else:
            self.get_logger().info('🚫 OpenCV 디스플레이가 비활성화되었습니다.')
        
        # 처리 타이머 (10Hz로 낮춤 - 더 안정적)
        self.timer = self.create_timer(0.1, self.process_images)
        
        # 검출 요청 플래그
        self.detection_requested = False
        self.target_name = None
        
        # 디스플레이 정보 저장
        self.display_info = {
            'objects_count': 0,
            'last_detection_time': 0,
            'target': 'None',
            'fps': 0
        }
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        self.get_logger().info('🎯 YOLO OBB Node started with display')

    def color_callback(self, msg):
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Color callback error: {str(e)}')

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {str(e)}')

    def detection_trigger_callback(self, msg):
        """검출 트리거 수신"""
        try:
            trigger_data = json.loads(msg.data)
            self.target_name = trigger_data.get('target', 'unknown')
            self.detection_requested = True
            self.display_info['target'] = self.target_name
            self.get_logger().info(f'🔍 검출 요청 수신: {self.target_name}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'검출 트리거 파싱 오류: {e}')

    def get_depth_at_point(self, x, y):
        """특정 픽셀 위치의 depth 값 반환 (mm 단위) - 안정화된 버전"""
        if self.latest_depth_image is None:
            return None
        
        height, width = self.latest_depth_image.shape
        if 0 <= x < width and 0 <= y < height:
            # 주변 5x5 영역의 중앙값 계산 (노이즈 감소)
            x_start = max(0, x-2)
            x_end = min(width, x+3)
            y_start = max(0, y-2)
            y_end = min(height, y+3)
            
            depth_region = self.latest_depth_image[y_start:y_end, x_start:x_end]
            valid_depths = depth_region[depth_region > 0]
            
            if len(valid_depths) > 0:
                # 중앙값 사용 (평균보다 노이즈에 강함)
                current_depth = np.median(valid_depths)
                
                # 버퍼에 추가
                self.depth_buffer.append(current_depth)
                
                # 버퍼의 평균값 반환 (시간적 안정화)
                if len(self.depth_buffer) >= 3:  # 최소 3개 값이 있을 때만
                    return np.mean(self.depth_buffer)
                else:
                    return current_depth
        return None

    def pixel_to_robot_coordinates(self, pixel_x, pixel_y):
        """픽셀 좌표를 로봇 좌표로 변환 (4점 기반 Bilinear 보간)"""
        
        # Bilinear 보간
        robot_x = (1 / 15) * pixel_y - (113 / 15)
        robot_y = (-19 / 286) * pixel_x + (12697 / 286)
        
        return robot_x, robot_y

    def calculate_fps(self):
        """FPS 계산"""
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:  # 1초마다 업데이트
            self.display_info['fps'] = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time

    def create_info_display(self):
        """정보 디스플레이 창 생성"""
        info_img = np.zeros((300, 400, 3), dtype=np.uint8)
        
        # 제목
        cv2.putText(info_img, 'YOLO Detection Info', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 구분선
        cv2.line(info_img, (10, 40), (390, 40), (255, 255, 255), 1)
        
        # 정보 표시
        y_pos = 70
        info_lines = [
            f"Target: {self.display_info['target']}",
            f"Objects Count: {self.display_info['objects_count']}",
            f"FPS: {self.display_info['fps']:.1f}",
            f"Detection Requested: {'Yes' if self.detection_requested else 'No'}",
            "",
            "Controls:",
            "ESC - Exit",
            "SPACE - Manual trigger",
            "R - Reset info"
        ]
        
        for i, line in enumerate(info_lines):
            color = (0, 255, 0) if i < 4 else (200, 200, 200)  # 상태는 초록색, 설명은 회색
            cv2.putText(info_img, line, (10, y_pos + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return info_img

    def process_images(self):
        if self.latest_color_image is None:
            return
        try:
            # FPS 계산
            self.calculate_fps()
            
            # YOLO 추론
            results = self.model.predict(source=self.latest_color_image, imgsz=640, conf=self.conf_threshold, verbose=False)
            
            # 검출된 객체 수 업데이트
            if results[0].obb is not None:
                self.display_info['objects_count'] = len(results[0].obb)
            else:
                self.display_info['objects_count'] = 0
            
            # 결과 그리기
            annotated = self.draw_results(self.latest_color_image, results[0])
            
            # 검출 요청이 있으면 결과 발행
            if self.detection_requested:
                self.publish_detection_results(results[0])
                self.detection_requested = False
                self.display_info['last_detection_time'] = time.time()
            
            # 디스플레이 활성화된 경우 화면 표시
            if self.display_enabled:
                # 메인 검출 결과 표시
                cv2.imshow('YOLO Detection', annotated)
                
                # 정보 창 표시
                info_display = self.create_info_display()
                cv2.imshow('Detection Info', info_display)
                
                # 키 입력 처리
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    self.get_logger().info('ESC pressed, shutting down...')
                    rclpy.shutdown()
                elif key == ord(' '):  # SPACE - 수동 트리거
                    self.detection_requested = True
                    self.get_logger().info('🔍 수동 검출 트리거')
                elif key == ord('r') or key == ord('R'):  # R - 정보 리셋
                    self.display_info = {
                        'objects_count': 0,
                        'last_detection_time': 0,
                        'target': 'None',
                        'fps': 0
                    }
                    self.get_logger().info('📊 정보 리셋')
            
            # 결과 이미지를 토픽으로도 발행
            try:
                result_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                self.result_pub.publish(result_msg)
            except Exception as e:
                self.get_logger().error(f'Result image publish error: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}')

    def publish_detection_results(self, result):
        """검출 결과를 토픽으로 발행"""
        detected_objects = []
        
        if result.obb is not None and len(result.obb) > 0:
            obb_coords = result.obb.xyxyxyxy.cpu().numpy()
            
            for i, coords in enumerate(obb_coords):
                # OBB 좌표 (x0, y0, x1, y1, x2, y2, x3, y3)
                points = coords.reshape(4, 2).astype(np.int32)
                
                # 변의 길이 계산
                len_side1 = np.linalg.norm(points[1] - points[0])
                len_side2 = np.linalg.norm(points[2] - points[1])
                
                # 짧은 변 기준으로 각도 계산
                if len_side1 <= len_side2:
                    angle = math.degrees(math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
                else:
                    angle = math.degrees(math.atan2(points[2][1] - points[1][1], points[2][0] - points[1][0]))
                
                # 각도를 0-360 범위로 정규화
                if angle < 0:
                    angle += 360
                
                # 중심점 계산
                center_x = np.mean(points[:, 0])
                center_y = np.mean(points[:, 1])
                
                # 중심점의 depth 값
                depth_mm = self.get_depth_at_point(int(center_x), int(center_y))
                
                # 픽셀 좌표를 로봇 좌표로 변환
                robot_x, robot_y = self.pixel_to_robot_coordinates(center_x, center_y)
                
                detected_object = {
                    'id': i,
                    'pixel_x': float(center_x),
                    'pixel_y': float(center_y),
                    'angle': float(angle),
                    'depth_mm': float(depth_mm) if depth_mm is not None else None,
                    'robot_x': float(robot_x),
                    'robot_y': float(robot_y)
                }
                
                detected_objects.append(detected_object)
                
                # 로그에 픽셀 좌표와 로봇 좌표 모두 출력
                self.get_logger().info(f'검출 결과 #{i+1}: 픽셀({center_x:.0f}, {center_y:.0f}, {angle:.0f}°) → 로봇({robot_x:.2f}, {robot_y:.2f})')
        
        # 검출 결과 발행
        result_msg = String()
        result_data = {
            'target': self.target_name,
            'timestamp': time.time(),
            'objects': detected_objects
        }
        result_msg.data = json.dumps(result_data)
        self.detection_result_pub.publish(result_msg)
        
        self.get_logger().info(f'✅ 검출 결과 발행: {len(detected_objects)}개 객체 (로봇 좌표 포함)')

    def draw_results(self, image, result):
        """결과 그리기 - 향상된 시각화"""
        annotated = image.copy()
        
        # 상단에 정보 오버레이 추가
        overlay = annotated.copy()
        cv2.rectangle(overlay, (0, 0), (annotated.shape[1], 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, annotated, 0.3, 0, annotated)
        
        # 헤더 정보 표시
        cv2.putText(annotated, f"Target: {self.display_info['target']} | Objects: {self.display_info['objects_count']} | FPS: {self.display_info['fps']:.1f}", 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        status = "DETECTING..." if self.detection_requested else "MONITORING"
        status_color = (0, 255, 255) if self.detection_requested else (0, 255, 0)
        cv2.putText(annotated, f"Status: {status}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        if result.obb is not None and len(result.obb) > 0:
            obb_coords = result.obb.xyxyxyxy.cpu().numpy()
            
            for i, coords in enumerate(obb_coords):
                # OBB 좌표 (x0, y0, x1, y1, x2, y2, x3, y3)
                points = coords.reshape(4, 2).astype(np.int32)
                
                # 변의 길이 계산
                len_side1 = np.linalg.norm(points[1] - points[0])
                len_side2 = np.linalg.norm(points[2] - points[1])
                
                start_point = None
                end_point = None
                
                if len_side1 <= len_side2:
                    start_point = points[0]
                    end_point = points[1]
                    angle = math.degrees(math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
                else:
                    start_point = points[1]
                    end_point = points[2]
                    angle = math.degrees(math.atan2(points[2][1] - points[1][1], points[2][0] - points[1][0]))
                
                # 각도를 0-360 범위로 정규화
                if angle < 0:
                    angle += 360
                
                # 중심점 계산
                center_x = np.mean(points[:, 0])
                center_y = np.mean(points[:, 1])
                
                # 픽셀 좌표를 로봇 좌표로 변환
                robot_x, robot_y = self.pixel_to_robot_coordinates(center_x, center_y)
                
                # 색상 선택 (객체별로 다른 색상)
                colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
                color = colors[i % len(colors)]
                
                # OBB 박스 그리기 (더 두껍게)
                cv2.polylines(annotated, [points], True, color, 3)
                
                # 중심점 표시 (더 크게)
                cv2.circle(annotated, (int(center_x), int(center_y)), 8, (0, 0, 255), -1)
                cv2.circle(annotated, (int(center_x), int(center_y)), 12, (255, 255, 255), 2)
                
                # 짧은 변에 화살표 그리기 (방향 표시)
                if start_point is not None and end_point is not None:
                    cv2.arrowedLine(annotated, tuple(start_point), tuple(end_point), (0, 255, 255), 3, tipLength=0.3)
                
                # 객체 번호 표시
                cv2.putText(annotated, f"#{i+1}", (int(center_x) - 30, int(center_y) - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                # 픽셀 좌표 및 각도 정보
                pixel_info = f"Pixel: ({center_x:.0f}, {center_y:.0f}, {angle:.0f}deg)"
                robot_info = f"Robot: ({robot_x:.1f}, {robot_y:.1f})"
                
                # 텍스트 크기 계산
                pixel_text_size = cv2.getTextSize(pixel_info, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                robot_text_size = cv2.getTextSize(robot_info, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                
                # 텍스트 배경 (두 줄)
                bg_width = max(pixel_text_size[0], robot_text_size[0]) + 10
                bg_height = pixel_text_size[1] + robot_text_size[1] + 20
                
                cv2.rectangle(annotated, 
                             (int(center_x) - 80, int(center_y) + 15), 
                             (int(center_x) - 80 + bg_width, int(center_y) + 15 + bg_height),
                             (0, 0, 0), -1)
                
                # 픽셀 좌표 텍스트 (첫 번째 줄)
                cv2.putText(annotated, pixel_info, (int(center_x) - 75, int(center_y) + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                
                # 로봇 좌표 텍스트 (두 번째 줄)
                cv2.putText(annotated, robot_info, (int(center_x) - 75, int(center_y) + 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # 중앙 십자선 그리기 (카메라 중앙 표시)
        h, w = annotated.shape[:2]
        cv2.line(annotated, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 255), 2)
        cv2.line(annotated, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 255), 2)
        
        return annotated

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YoloObbNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        cv2.destroyAllWindows()  # 윈도우 정리
        rclpy.shutdown()

if __name__ == '__main__':
    main()