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
            results = self.model.predict(source=self.latest_color_image, imgsz=224, conf=self.conf_threshold, verbose=False)
            
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
            confidences = result.obb.conf.cpu().numpy()
            
            for i, (coords, conf) in enumerate(zip(obb_coords, confidences)):
                # OBB 좌표
                points = coords.reshape(4, 2).astype(np.int32)
                
                # 중심점, 각도 계산
                center_x = np.mean(points[:, 0])
                center_y = np.mean(points[:, 1])
                angle = math.degrees(math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
                if angle < 0:
                    angle += 360
                
                # 중심점의 depth 값
                depth_mm = self.get_depth_at_point(int(center_x), int(center_y))
                
                # OBB 박스 그리기
                cv2.polylines(annotated, [points], True, (0, 255, 0), 2)
                
                # 중심점 표시 (빨간색)
                cv2.circle(annotated, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                
                # # 방향 화살표 (노란색)
                # cv2.arrowedLine(annotated, tuple(points[0]), tuple(points[1]), (0, 255, 255), 2)
                
                # 핵심 정보만 표시: x, y, deg, depth
                if depth_mm is not None:
                    info_text = f"x:{center_x:.0f} y:{center_y:.0f} deg:{angle:.0f} depth:{depth_mm:.0f}mm"
                else:
                    info_text = f"x:{center_x:.0f} y:{center_y:.0f} deg:{angle:.0f} depth:N/A"
                
                # 텍스트 배경
                text_size = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                text_x = int(center_x) - text_size[0]//2
                text_y = int(center_y) - 20
                
                # # 배경 사각형
                # cv2.rectangle(annotated, (text_x-5, text_y-15), (text_x+text_size[0]+5, text_y+5), (0, 0, 0), -1)
                
                # 텍스트
                cv2.putText(annotated, info_text, (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                
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