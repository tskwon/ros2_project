#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
from collections import deque

class BlackObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('black_object_detector_node')

        self.bridge = CvBridge()

        self.latest_color_image = None
        self.latest_depth_image = None
        self.depth_buffer = deque(maxlen=5)

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

        self.trigger_sub = self.create_subscription(
            String,
            '/yolo/detection_trigger',
            self.trigger_callback,
            10
        )

        self.result_pub = self.create_publisher(Image, '/yolo_result', 1)
        self.data_pub = self.create_publisher(String, '/yolo/detection_result', 10)

        self.detection_requested = False
        self.get_logger().info('BlackObjectDetectorNode initialized')

        # OpenCV 창 초기화 - NORMAL 윈도우로 변경하여 크기 조절 가능하게 함
        self.window_name = 'Black Object Detection'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # 창 크기를 원본 이미지 크기에 맞게 설정 (일반적인 RealSense 해상도)
        cv2.resizeWindow(self.window_name, 640, 480)

    def color_callback(self, msg):
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Color image error: {e}")
    
    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Depth image error: {e}")

    def get_average_depth_in_box(self, box):
        """바운딩 박스 영역의 평균 depth 계산 (median 기반)"""
        if self.latest_depth_image is None:
            return None

        # 바운딩 박스를 이용한 마스크 생성
        mask = np.zeros_like(self.latest_depth_image, dtype=np.uint8)
        cv2.drawContours(mask, [box], -1, 255, -1)

        # 마스크 영역의 depth 값들 추출
        region = cv2.bitwise_and(self.latest_depth_image, self.latest_depth_image, mask=mask)
        valid_depths = region[region > 0]
        
        if len(valid_depths) > 0:
            depth = np.median(valid_depths)
            self.depth_buffer.append(depth)
            # 버퍼에 충분한 데이터가 있으면 평균 사용, 아니면 현재 값 사용
            return np.mean(self.depth_buffer) if len(self.depth_buffer) >= 3 else depth
        return None

    def trigger_callback(self, msg):
        """검출 트리거 콜백 - non-blocking으로 수정"""
        if self.latest_color_image is not None and self.latest_depth_image is not None:
            self.detection_requested = True
            self.get_logger().info("Detection triggered")
            self.process()
        else:
            self.get_logger().warn("Images not available for detection")

    def pixel_to_robot_coordinates(self, x, y):
        """픽셀 좌표를 로봇 좌표로 변환"""
        robot_x = (1 / 15) * y - (113 / 15)
        robot_y = (-19 / 286) * x + (12697 / 286)
        return robot_x, robot_y

    def get_angle(self, p1, p2):
        """두 점 사이의 각도 계산"""
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        return np.degrees(np.arctan2(dy, dx))

    def process(self):
        """메인 처리 함수"""
        image = self.latest_color_image
        if image is None:
            self.get_logger().warn("No color image available")
            return

        # ROI 설정
        # roi_x, roi_y, roi_w, roi_h = 240, 40, 250, 280  # ROI 영역
        # roi = image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]


        roi_x, roi_y, roi_w, roi_h = 240, 113, 250, 210  # ROI 영역
        roi = image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

        # HSV 변환 및 V 채널 평활화
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v_eq = cv2.equalizeHist(v)
        hsv_eq = cv2.merge((h, s, v_eq))

        # 검정색 범위 마스크
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 55])
        mask = cv2.inRange(hsv_eq, lower_black, upper_black)

        # 침식 → 객체 분리
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)

        # 경계 다듬기
        mask = cv2.medianBlur(mask, 5)

        # 컨투어 검출
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result_img = image.copy()
        detected_objects = []

        print(f"🔍 총 contour 개수: {len(contours)}")

        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            rect = cv2.minAreaRect(cnt)
            (cx, cy), (w_box, h_box), angle = rect
            center_x, center_y = int(cx), int(cy)
            box = cv2.boxPoints(rect)
            box = np.intp(box)

            # ROI 기준 → 원본 이미지 기준으로 좌표 보정
            center_x_global = center_x + roi_x
            center_y_global = center_y + roi_y
            box_global = box + np.array([[roi_x, roi_y]])

            color = (0, 0, 255)
            reason = ""

            # 필터 조건
            if area < 1500:
                reason = "면적이 너무 작음"
            elif area > 10000:
                reason = "면적이 너무 큼"
            elif not (211 <= center_x_global <= 577 and 6 <= center_y_global <= 392):
                reason = "중심 좌표 범위 벗어남"
            elif w_box < 20:
                reason = "w 작음"
            elif w_box > 120:
                reason = "w 큼"
            elif h_box < 30:
                reason = "h 작음"
            elif h_box > 150:
                reason = "h 큼"
            else:
                color = (0, 255, 0)
                if w_box > h_box:
                    angle += 90
                robot_x, robot_y = self.pixel_to_robot_coordinates(center_x_global, center_y_global)
                reason = "✔️ 검출됨"
                
                # Depth 정보 계산
                depth_mm = self.get_average_depth_in_box(box_global)
                
                # 검출된 객체 정보 저장
                detected_object = {
                    'id': i,
                    'pixel_x': float(center_x_global),
                    'pixel_y': float(center_y_global),
                    'angle': round(angle, 2),
                    'width': round(w_box, 1),
                    'height': round(h_box, 1),
                    'area': round(area, 1),
                    'depth_mm': float(depth_mm) if depth_mm is not None else None,
                    'robot_x': round(robot_x, 2),
                    'robot_y': round(robot_y, 2)
                }
                detected_objects.append(detected_object)
                
                print(f"[✔️] ID:{i}, 중심=({center_x_global}, {center_y_global}), 회전각={angle:.1f}°, h={h_box:.1f}, w={w_box:.1f}, area={area:.1f}, robot=({robot_x:.1f}, {robot_y:.1f})")

            if reason != "✔️ 검출됨":
                print(f"[❌] ID:{i}, 이유: {reason}")

            # 시각화
            cv2.drawContours(result_img, [box_global], 0, color, 2)
            cv2.circle(result_img, (center_x_global, center_y_global), 4, (0, 0, 255), -1)
            label = f"ID:{i}"
            cv2.putText(result_img, label, (center_x_global - 20, center_y_global - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)



        # 결과 이미지 표시 (원본 크기 유지)
        cv2.imshow(self.window_name, result_img)
        cv2.waitKey(1)

        # ROS 메시지 발행
        try:
            result_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
            self.result_pub.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish result image: {e}")

        # 검출 결과 데이터 발행
        data_msg = String()
        data_msg.data = json.dumps({
            'timestamp': time.time(), 
            'objects': detected_objects
        })
        self.data_pub.publish(data_msg)

        self.get_logger().info(f"Detected {len(detected_objects)} black object(s)")
        self.detection_requested = False

    def destroy_node(self):
        """노드 소멸시 OpenCV 창 정리"""
        cv2.destroyWindow(self.window_name)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BlackObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
