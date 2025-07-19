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
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        
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
        
        # OpenCV 윈도우 설정 제거
        # cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
        
        # 처리 타이머 (10Hz로 낮춤 - 더 안정적)
        self.timer = self.create_timer(0.1, self.process_images)
        
        # 검출 요청 플래그
        self.detection_requested = False
        self.target_name = None
        
        self.get_logger().info('YOLO OBB Node started - No display mode')

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

    def process_images(self):
        if self.latest_color_image is None:
            return
        try:
            # YOLO 추론
            results = self.model.predict(source=self.latest_color_image, imgsz=640, conf=self.conf_threshold, verbose=False)
            
            # 결과 그리기 (표시용이 아닌 결과 이미지 생성용)
            annotated = self.draw_results(self.latest_color_image, results[0])
            
            # 검출 요청이 있으면 결과 발행
            if self.detection_requested:
                self.publish_detection_results(results[0])
                self.detection_requested = False
            
            # 결과 이미지 표시 제거
            # cv2.imshow('YOLO Detection', annotated)
            
            # ESC 키 체크 제거 (GUI 없이 실행)
            # key = cv2.waitKey(1) & 0xFF
            # if key == 27:
            #     self.get_logger().info('ESC pressed, shutting down...')
            #     rclpy.shutdown()
            
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
                
                # 중심점의 depth 값 (depth는 사용하지 않지만 일단 계산)
                depth_mm = self.get_depth_at_point(int(center_x), int(center_y))
                
                detected_object = {
                    'id': i,
                    'pixel_x': float(center_x),
                    'pixel_y': float(center_y),
                    'angle': float(angle),
                    'depth_mm': float(depth_mm) if depth_mm is not None else None
                }
                
                detected_objects.append(detected_object)
                
                self.get_logger().info(f'검출 결과 #{i+1}: x={center_x:.0f}, y={center_y:.0f}, deg={angle:.0f}°')
        
        # 검출 결과 발행
        result_msg = String()
        result_data = {
            'target': self.target_name,
            'timestamp': time.time(),
            'objects': detected_objects
        }
        result_msg.data = json.dumps(result_data)
        self.detection_result_pub.publish(result_msg)
        
        self.get_logger().info(f'✅ 검출 결과 발행: {len(detected_objects)}개 객체')

    def draw_results(self, image, result):
        """결과 그리기 (이미지 토픽 발행용)"""
        annotated = image.copy()
        
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
                
                # 중심점의 depth 값 (안정화된 버전)
                depth_mm = self.get_depth_at_point(int(center_x), int(center_y))
                
                # OBB 박스 그리기
                cv2.polylines(annotated, [points], True, (0, 255, 0), 2)
                
                # 중심점 표시 (빨간색)
                cv2.circle(annotated, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                
                # 짧은 변에 화살표 그리기 (노란색)
                if start_point is not None and end_point is not None:
                    cv2.arrowedLine(annotated, tuple(start_point), tuple(end_point), (0, 255, 255), 2, tipLength=0.3)
                
                # 핵심 정보 표시: x, y, deg (depth 제외)
                info_text = f"x:{center_x:.0f} y:{center_y:.0f} deg:{angle:.0f}"
                
                # 텍스트
                cv2.putText(annotated, info_text, (int(center_x) - 70, int(center_y) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
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
        # cv2.destroyAllWindows() 제거
        rclpy.shutdown()

if __name__ == '__main__':
    main()