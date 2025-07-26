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
        self.declare_parameter('confidence_threshold', 0.70)
        self.declare_parameter('display_enabled', True)
        
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
        
        # 뎁스 값 안정화를 위한 버퍼
        self.depth_buffer = deque(maxlen=5)
        
        # 구독자들
        self.color_sub = self.create_subscription(
            Image, '/d415/realsense_d415/color/image_raw', self.color_callback, 1)
        
        self.depth_sub = self.create_subscription(
            Image, '/d415/realsense_d415/depth/image_rect_raw', self.depth_callback, 1)
        
        self.detection_trigger_sub = self.create_subscription(
            String, '/yolo/detection_trigger', self.detection_trigger_callback, 10)
        
        # 퍼블리셔들
        self.result_pub = self.create_publisher(Image, '/yolo_result', 1)
        self.detection_result_pub = self.create_publisher(String, '/yolo/detection_result', 10)
        
        # OpenCV 윈도우 설정
        if self.display_enabled:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Detection Info', cv2.WINDOW_AUTOSIZE)
            self.get_logger().info('🖥️ OpenCV 디스플레이가 활성화되었습니다.')
        else:
            self.get_logger().info('🚫 OpenCV 디스플레이가 비활성화되었습니다.')
        
        # 처리 타이머
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
        try:
            trigger_data = json.loads(msg.data)
            self.target_name = trigger_data.get('target', 'unknown')
            self.detection_requested = True
            self.display_info['target'] = self.target_name
            self.get_logger().info(f'🔍 검출 요청 수신: {self.target_name}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'검출 트리거 파싱 오류: {e}')

    def get_depth_at_point(self, x, y):
        if self.latest_depth_image is None:
            return None
        
        height, width = self.latest_depth_image.shape
        if 0 <= x < width and 0 <= y < height:
            x_start = max(0, x-2)
            x_end = min(width, x+3)
            y_start = max(0, y-2)
            y_end = min(height, y+3)
            
            depth_region = self.latest_depth_image[y_start:y_end, x_start:x_end]
            valid_depths = depth_region[depth_region > 0]
            
            if len(valid_depths) > 0:
                current_depth = np.median(valid_depths)
                self.depth_buffer.append(current_depth)
                
                if len(self.depth_buffer) >= 3:
                    return np.mean(self.depth_buffer)
                else:
                    return current_depth
        return None

    def pixel_to_robot_coordinates(self, pixel_x, pixel_y):
        robot_x = (1 / 15) * pixel_y - (113 / 15)
        robot_y = (-19 / 286) * pixel_x + (12697 / 286)
        return robot_x, robot_y

    def calculate_fps(self):
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.display_info['fps'] = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time

    def create_info_display(self):
        info_img = np.zeros((300, 400, 3), dtype=np.uint8)
        
        # # 제목
        # cv2.putText(info_img, 'YOLO Detection Info', (10, 30), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 구분선
        cv2.line(info_img, (10, 40), (390, 40), (255, 255, 255), 1)
        
        # # 정보 표시
        # y_pos = 70
        # info_lines = [
        #     f"Target: {self.display_info['target']}",
        #     f"Objects Count: {self.display_info['objects_count']}",
        #     f"FPS: {self.display_info['fps']:.1f}",
        #     f"Detection Requested: {'Yes' if self.detection_requested else 'No'}",
        #     "",
        #     "Controls:",
        #     "ESC - Exit",
        #     "SPACE - Manual trigger",
        #     "R - Reset info"
        # ]
        
        # for i, line in enumerate(info_lines):
        #     color = (0, 255, 0) if i < 4 else (200, 200, 200)
        #     cv2.putText(info_img, line, (10, y_pos + i * 25), 
        #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return info_img

    def correct_rotation_with_hsv(self, image, obb_points, center_x, center_y):
        """HSV 기반으로 회전 각도 보정 - 버그 수정"""
        
        try:
            # ROI 영역 확장 (여유 공간 확보)
            x, y, w, h = cv2.boundingRect(obb_points)
            margin = 20
            roi_y_start = max(0, y-margin)
            roi_y_end = min(image.shape[0], y+h+margin)
            roi_x_start = max(0, x-margin)
            roi_x_end = min(image.shape[1], x+w+margin)
            
            roi = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
            
            # ROI가 유효한지 확인
            if roi.shape[0] <= 0 or roi.shape[1] <= 0:
                self.get_logger().debug('ROI 크기가 유효하지 않음')
                return None
            
            # HSV 변환
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # **수정된 부분**: ROI의 실제 크기 사용
            roi_h, roi_w = roi.shape[:2]
            if roi_h <= 0 or roi_w <= 0:
                self.get_logger().debug('ROI 크기가 0 이하')
                return None
                
            # ROI 중심점의 색상 (수정된 인덱스 사용)
            roi_center_color = hsv[roi_h//2, roi_w//2]
            
            # HSV 범위 설정 - 더 넓은 범위로 설정
            hue = roi_center_color[0]
            sat = roi_center_color[1] 
            val = roi_center_color[2]
            
            # 무채색 객체 처리
            if sat < 30:  # 채도가 낮으면 밝기 기반으로
                lower_hsv = np.array([0, 0, max(0, val - 40)])
                upper_hsv = np.array([180, 255, min(255, val + 40)])
            else:  # 유채색 객체
                lower_hsv = np.array([max(0, hue - 30), 30, 30])
                upper_hsv = np.array([min(180, hue + 30), 255, 255])
            
            # 마스크 생성
            color_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            
            # 노이즈 제거
            kernel = np.ones((3, 3), np.uint8)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
            
            # 컨투어 찾기
            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 가장 큰 컨투어 선택
                largest_contour = max(contours, key=cv2.contourArea)
                
                # 컨투어 크기 검증
                area = cv2.contourArea(largest_contour)
                if area < 50:  # 임계값을 50으로 낮춤
                    self.get_logger().debug(f'컨투어 크기가 너무 작음: {area}')
                    return None
                
                # MinAreaRect로 정확한 회전각 계산
                rect = cv2.minAreaRect(largest_contour)
                corrected_angle = rect[2]
                
                # 각도 보정
                if rect[1][0] < rect[1][1]:  # width < height
                    corrected_angle += 90
                
                # 0-360도 범위로 정규화
                if corrected_angle < 0:
                    corrected_angle += 360
                elif corrected_angle >= 360:
                    corrected_angle -= 360
                
                self.get_logger().debug(f'HSV 보정 성공: {corrected_angle:.1f}도 (컨투어 크기: {area:.1f})')
                return corrected_angle
            else:
                self.get_logger().debug('유효한 컨투어를 찾을 수 없음')
                return None
                
        except Exception as e:
            self.get_logger().warn(f'HSV 보정 중 예외 발생: {str(e)}')
            return None

    def calculate_correction_confidence(self, yolo_angle, corrected_angle):
        if corrected_angle is None:
            return 0.0
        
        angle_diff = abs(yolo_angle - corrected_angle)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        confidence = max(0, 1 - angle_diff / 90.0)
        return confidence

    def process_images(self):
        if self.latest_color_image is None:
            return
        try:
            self.calculate_fps()
            
            results = self.model.predict(source=self.latest_color_image, imgsz=640, conf=self.conf_threshold, verbose=False)
            
            if results[0].obb is not None:
                self.display_info['objects_count'] = len(results[0].obb)
            else:
                self.display_info['objects_count'] = 0
            
            annotated = self.draw_results(self.latest_color_image, results[0])
            
            if self.detection_requested:
                self.publish_detection_results(results[0])
                self.detection_requested = False
                self.display_info['last_detection_time'] = time.time()
            
            if self.display_enabled:
                cv2.imshow('YOLO Detection', annotated)
                
                info_display = self.create_info_display()
                cv2.imshow('Detection Info', info_display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    self.get_logger().info('ESC pressed, shutting down...')
                    rclpy.shutdown()
                elif key == ord(' '):  # SPACE
                    self.detection_requested = True
                    self.get_logger().info('🔍 수동 검출 트리거')
                elif key == ord('r') or key == ord('R'):  # R
                    self.display_info = {
                        'objects_count': 0,
                        'last_detection_time': 0,
                        'target': 'None',
                        'fps': 0
                    }
                    self.get_logger().info('📊 정보 리셋')
            
            try:
                result_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                self.result_pub.publish(result_msg)
            except Exception as e:
                self.get_logger().error(f'Result image publish error: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}')

    def publish_detection_results(self, result):
        detected_objects = []
        
        if result.obb is not None and len(result.obb) > 0:
            obb_coords = result.obb.xyxyxyxy.cpu().numpy()
            
            for i, coords in enumerate(obb_coords):
                points = coords.reshape(4, 2).astype(np.int32)
                
                len_side1 = np.linalg.norm(points[1] - points[0])
                len_side2 = np.linalg.norm(points[2] - points[1])
                
                if len_side1 <= len_side2:
                    yolo_angle = math.degrees(math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
                else:
                    yolo_angle = math.degrees(math.atan2(points[2][1] - points[1][1], points[2][0] - points[1][0]))
                
                if yolo_angle < 0:
                    yolo_angle += 360
                
                center_x = np.mean(points[:, 0])
                center_y = np.mean(points[:, 1])
                
                try:
                    corrected_angle = self.correct_rotation_with_hsv(
                        self.latest_color_image, points, center_x, center_y)
                    
                    confidence = self.calculate_correction_confidence(yolo_angle, corrected_angle)
                    final_angle = corrected_angle if (corrected_angle is not None and confidence > 0.3) else yolo_angle
                    
                except Exception as e:
                    self.get_logger().warn(f'HSV 보정 실패: {str(e)}')
                    corrected_angle = None
                    confidence = 0.0
                    final_angle = yolo_angle
                
                depth_mm = self.get_depth_at_point(int(center_x), int(center_y))
                robot_x, robot_y = self.pixel_to_robot_coordinates(center_x, center_y)
                
                detected_object = {
                    'id': i,
                    'pixel_x': float(center_x),
                    'pixel_y': float(center_y),
                    'yolo_angle': float(yolo_angle),
                    'corrected_angle': float(corrected_angle) if corrected_angle is not None else None,
                    'final_angle': float(final_angle),
                    'correction_confidence': float(confidence),
                    'depth_mm': float(depth_mm) if depth_mm is not None else None,
                    'robot_x': float(robot_x),
                    'robot_y': float(robot_y)
                }
                
                detected_objects.append(detected_object)
                
                if corrected_angle is not None:
                    self.get_logger().info(
                        f'🎯 보정 결과 #{i+1}: YOLO({yolo_angle:.0f}°) → HSV보정({corrected_angle:.0f}°) → 최종({final_angle:.0f}°) [신뢰도: {confidence:.2f}]'
                    )
                else:
                    self.get_logger().info(
                        f'📐 검출 결과 #{i+1}: YOLO({yolo_angle:.0f}°) → HSV보정 실패 → 최종({final_angle:.0f}°)'
                    )
        
        result_msg = String()
        result_data = {
            'target': self.target_name,
            'timestamp': time.time(),
            'objects': detected_objects
        }
        result_msg.data = json.dumps(result_data)
        self.detection_result_pub.publish(result_msg)
        
        self.get_logger().info(f'✅ 검출 결과 발행: {len(detected_objects)}개 객체 (HSV 보정 포함)')

    def draw_results(self, image, result):
        annotated = image.copy()
        
        overlay = annotated.copy()
        cv2.rectangle(overlay, (0, 0), (annotated.shape[1], 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, annotated, 0.3, 0, annotated)
        
        # # putText 모두 주석처리
        # cv2.putText(annotated, f"Target: {self.display_info['target']} | Objects: {self.display_info['objects_count']} | FPS: {self.display_info['fps']:.1f}", 
        #         (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # status = "DETECTING..." if self.detection_requested else "MONITORING"
        # status_color = (0, 255, 255) if self.detection_requested else (0, 255, 0)
        # cv2.putText(annotated, f"Status: {status} | HSV Correction: ON", (10, 55), 
        #         cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        if result.obb is not None and len(result.obb) > 0:
            obb_coords = result.obb.xyxyxyxy.cpu().numpy()
            
            for i, coords in enumerate(obb_coords):
                points = coords.reshape(4, 2).astype(np.int32)
                
                len_side1 = np.linalg.norm(points[1] - points[0])
                len_side2 = np.linalg.norm(points[2] - points[1])
                
                if len_side1 <= len_side2:
                    yolo_angle = math.degrees(math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
                else:
                    yolo_angle = math.degrees(math.atan2(points[2][1] - points[1][1], points[2][0] - points[1][0]))
                
                if yolo_angle < 0:
                    yolo_angle += 360
                
                center_x = np.mean(points[:, 0])
                center_y = np.mean(points[:, 1])
                
                corrected_angle = self.correct_rotation_with_hsv(image, points, center_x, center_y)
                confidence = self.calculate_correction_confidence(yolo_angle, corrected_angle)
                
                colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
                color = colors[i % len(colors)]
                
                cv2.polylines(annotated, [points], True, color, 3)
                
                cv2.circle(annotated, (int(center_x), int(center_y)), 8, (0, 0, 255), -1)
                cv2.circle(annotated, (int(center_x), int(center_y)), 12, (255, 255, 255), 2)
                
                final_angle = corrected_angle if (corrected_angle is not None and confidence > 0.3) else yolo_angle
                arrow_length = 40
                end_x = int(center_x + arrow_length * math.cos(math.radians(final_angle)))
                end_y = int(center_y + arrow_length * math.sin(math.radians(final_angle)))
                cv2.arrowedLine(annotated, (int(center_x), int(center_y)), (end_x, end_y), (0, 255, 255), 3, tipLength=0.3)
                
                # # 객체 번호와 각도 정보 텍스트도 주석처리
                # cv2.putText(annotated, f"#{i+1}", (int(center_x) - 30, int(center_y) - 30),
                #         cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                # yolo_info = f"YOLO: {yolo_angle:.0f}deg"
                # hsv_info = f"HSV: {corrected_angle:.0f}deg ({confidence:.2f})" if corrected_angle is not None else "HSV: Failed"
                # final_info = f"Final: {final_angle:.0f}deg"
                
                # bg_height = 75
                # cv2.rectangle(annotated, 
                #             (int(center_x) - 80, int(center_y) + 15), 
                #             (int(center_x) + 120, int(center_y) + 15 + bg_height),
                #             (0, 0, 0), -1)
                
                # cv2.putText(annotated, yolo_info, (int(center_x) - 75, int(center_y) + 35),
                #         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
                # cv2.putText(annotated, hsv_info, (int(center_x) - 75, int(center_y) + 55),
                #         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255) if corrected_angle else (100, 100, 100), 1)
                # cv2.putText(annotated, final_info, (int(center_x) - 75, int(center_y) + 75),
                #         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        
        # 중앙 십자선은 유지
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
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
