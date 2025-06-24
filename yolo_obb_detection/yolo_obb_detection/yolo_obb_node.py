#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
import math

class YoloObbNode(Node):
    def __init__(self):
        super().__init__('yolo_obb_node')
        
        # 파라미터
        self.declare_parameter('model_path', '/home/xotn/ros2_ws/src/yolo_obb_detection/models/best.pt')
        self.declare_parameter('confidence_threshold', 0.30)
        
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
        
        # 구독자들
        self.color_sub = self.create_subscription(
            Image,
            '/camera/realsense/color/image_raw',
            self.color_callback,
            1
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/realsense/depth/image_rect_raw',
            self.depth_callback,
            1
        )
        
        # 퍼블리셔
        self.result_pub = self.create_publisher(Image, '/yolo_result', 1)
        
        # OpenCV 윈도우 설정
        cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
        
        # 처리 타이머 (15Hz)
        self.timer = self.create_timer(0.067, self.process_images)
        
        self.get_logger().info('YOLO OBB Node started - Press ESC to quit')

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

    def get_depth_at_point(self, x, y):
        """특정 픽셀 위치의 depth 값 반환 (mm 단위)"""
        if self.latest_depth_image is None:
            return None
        
        height, width = self.latest_depth_image.shape
        if 0 <= x < width and 0 <= y < height:
            # 주변 3x3 영역의 평균값 계산
            x_start = max(0, x-1)
            x_end = min(width, x+2)
            y_start = max(0, y-1)
            y_end = min(height, y+2)
            
            depth_region = self.latest_depth_image[y_start:y_end, x_start:x_end]
            valid_depths = depth_region[depth_region > 0]
            
            if len(valid_depths) > 0:
                return np.mean(valid_depths)
        return None

    def process_images(self):
        if self.latest_color_image is None:
            return
        try:
            # YOLO 추론
            # imgsz는 224가 아닌, 모델 학습 시 사용한 이미지 크기나, 추론 시 성능과 정확도 간의 적절한 트레이드오프를 고려한 크기로 설정해야 합니다.
            # 일반적으로 640이 많이 사용됩니다. 여기서는 원본 이미지 크기 (None)를 사용하거나 적절한 값을 지정하세요.
            results = self.model.predict(source=self.latest_color_image, imgsz=self.latest_color_image.shape[0], conf=self.conf_threshold, verbose=False)
            
            # 결과 그리기
            annotated = self.draw_results(self.latest_color_image, results[0])
            
            # 결과 이미지 표시
            cv2.imshow('YOLO Detection', annotated)
            
            # ESC 키로 종료
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                self.get_logger().info('ESC pressed, shutting down...')
                rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}')

    def draw_results(self, image, result):
        annotated = image.copy()
        
        if result.obb is not None and len(result.obb) > 0:
            obb_coords = result.obb.xyxyxyxy.cpu().numpy()
            # confidences = result.obb.conf.cpu().numpy() # 굳이 사용하지 않으면 제거 가능
            
            for i, coords in enumerate(obb_coords):
                # OBB 좌표 (x0, y0, x1, y1, x2, y2, x3, y3)
                points = coords.reshape(4, 2).astype(np.int32)
                
                # 변의 길이 계산
                # OBB 꼭짓점 순서는 (좌상, 우상, 우하, 좌하) 또는 이와 유사한 순환 순서를 따릅니다.
                # 예를 들어, point0-point1-point2-point3-point0
                # side1: point0 to point1
                # side2: point1 to point2
                
                len_side1 = np.linalg.norm(points[1] - points[0])
                len_side2 = np.linalg.norm(points[2] - points[1])
                
                start_point = None
                end_point = None
                
                if len_side1 <= len_side2:
                    # points[0]에서 points[1]으로 가는 변이 짧은 변
                    start_point = points[0]
                    end_point = points[1]
                    # 각도 계산 (짧은 변 기준)
                    angle = math.degrees(math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
                else:
                    # points[1]에서 points[2]으로 가는 변이 짧은 변
                    start_point = points[1]
                    end_point = points[2]
                    # 각도 계산 (짧은 변 기준)
                    angle = math.degrees(math.atan2(points[2][1] - points[1][1], points[2][0] - points[1][0]))
                
                # 각도를 0-360 범위로 정규화
                if angle < 0:
                    angle += 360
                
                # 중심점 계산
                center_x = np.mean(points[:, 0])
                center_y = np.mean(points[:, 1])
                
                # 중심점의 depth 값
                depth_mm = self.get_depth_at_point(int(center_x), int(center_y))
                
                # OBB 박스 그리기
                cv2.polylines(annotated, [points], True, (0, 255, 0), 2)
                
                # 중심점 표시 (빨간색)
                cv2.circle(annotated, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                
                # 짧은 변에 화살표 그리기 (노란색)
                if start_point is not None and end_point is not None:
                    cv2.arrowedLine(annotated, tuple(start_point), tuple(end_point), (0, 255, 255), 2, tipLength=0.3)
                
                # 핵심 정보 표시: x, y, deg, depth
                if depth_mm is not None:
                    info_text = f"x:{center_x:.0f} y:{center_y:.0f} deg:{angle:.0f} depth:{depth_mm:.0f}mm"
                else:
                    info_text = f"x:{center_x:.0f} y:{center_y:.0f} deg:{angle:.0f} depth:N/A"
                
                # 텍스트 배경 (선택 사항, 필요시 주석 해제)
                # text_size = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                # text_x = int(center_x) - text_size[0]//2
                # text_y = int(center_y) - 20
                # cv2.rectangle(annotated, (text_x-5, text_y-15), (text_x+text_size[0]+5, text_y+5), (0, 0, 0), -1)
                
                # 텍스트
                cv2.putText(annotated, info_text, (int(center_x) - 70, int(center_y) - 20), # 텍스트 위치 조정
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 터미널 로그
                depth_str = f"{depth_mm:.0f}mm" if depth_mm is not None else "N/A"
                self.get_logger().info(f'Product #{i+1}: x={center_x:.0f}, y={center_y:.0f}, deg={angle:.0f}, depth={depth_str}')
        
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
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()