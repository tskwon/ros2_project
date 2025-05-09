#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class OptimizedLaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # 파라미터 선언 및 가져오기
        self.declare_parameter('debug_view', True)
        self.declare_parameter('frame_rate', 10)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('roi_percentage', 0.5)
        
        self.debug_view = self.get_parameter('debug_view').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        roi_percentage = self.get_parameter('roi_percentage').value
        
        # 이미지 처리 상수 미리 계산
        self.roi_height = int(self.height * roi_percentage)
        self.roi_y = self.height - self.roi_height
        
        # 메모리 재사용을 위한 버퍼 할당
        self.gray_roi = np.zeros((self.roi_height, self.width), dtype=np.uint8)
        self.binary_roi = np.zeros((self.roi_height, self.width), dtype=np.uint8)
        self.edges = np.zeros((self.roi_height, self.width), dtype=np.uint8)
        
        # cv_bridge 인스턴스
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.lane_pub = self.create_publisher(Image, '/camera/lane_image', 10)
        
        # 카메라 초기화
        if not self.init_camera():
            self.get_logger().error("Failed to initialize camera")
            return
        
        # 타이머 설정
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.process_frame)
        
        self.get_logger().info("Optimized Lane Detection Node started")
    
    def init_camera(self):
        """카메라 초기화 및 최적화 설정"""
        self.cap = None
        
        # V4L2 장치 자동 검색
        for i in range(0, 12, 2):
            device_path = f"/dev/video{i}"
            if os.path.exists(device_path):
                try:
                    self.cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                    if self.cap.isOpened():
                        self.get_logger().info(f"Webcam opened at index {i} (V4L2)")
                        break
                except Exception as e:
                    self.get_logger().warn(f"Failed to open camera at index {i}: {e}")
        
        # 카메라 설정 최적화
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 크기 최소화
        except:
            self.get_logger().warn("CAP_PROP_BUFFERSIZE not supported")
        
        # 카메라 속성 확인
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f"Camera initialized: {actual_width}x{actual_height} at {actual_fps}fps")
        return True
    
    def process_frame(self):
        """프레임 처리 메서드"""
        for _ in range(2):  # Skip up to 2 old frames
            self.cap.grab()
        # 최신 프레임 가져오기
        self.cap.grab()  # 가장 최신 프레임으로 이동
        ret, frame = self.cap.retrieve()
        
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
        
        # 원본 이미지 발행 (구독자가 있을 때만)
        if self.image_pub.get_subscription_count() > 0:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(img_msg)
        
        # 차선 감지 처리
        lane_image = self.detect_lanes_optimized(frame)
        
        # 처리된 이미지 발행 (구독자가 있을 때만)
        if self.lane_pub.get_subscription_count() > 0:
            lane_msg = self.bridge.cv2_to_imgmsg(lane_image, encoding="bgr8")
            self.lane_pub.publish(lane_msg)
        
        # 디버그 화면 (활성화된 경우만)
        if self.debug_view:
            # 화면 크기가 작으면 그대로 보여주고, 크면 축소
            if self.width <= 320 and self.height <= 240:
                cv2.imshow("Lane Detection", lane_image)
            else:
                small_image = cv2.resize(lane_image, (320, 240))
                cv2.imshow("Lane Detection", small_image)
            cv2.waitKey(1)
    
    def detect_lanes_optimized(self, frame):
        """최적화된 차선 감지 알고리즘"""
        # ROI 추출
        roi = frame[self.roi_y:, :]
        
        # 그레이스케일 변환 (미리 할당된 버퍼 사용)
        cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY, self.gray_roi)
        
        # 노이즈 제거 (제자리 처리)
        cv2.GaussianBlur(self.gray_roi, (5, 5), 0, self.gray_roi)
        
        # 적응형 이진화 (제자리 처리)
        cv2.adaptiveThreshold(
            self.gray_roi, 255, 
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 11, 2, 
            self.binary_roi
        )
        
        # # 허프 변환으로 차선 검출 (더 정확도 높은 방법)
        # cv2.Canny(self.binary_roi, 50, 150, self.edges)
        # lines = cv2.HoughLinesP(
        #     self.edges, 1, np.pi/180, 
        #     threshold=10, 
        #     minLineLength=8, 
        #     maxLineGap=4
        # )
        
        # 차선 중심점 계산
        # if lines is not None and len(lines) > 0:
        #     # 검출된 선들의 중점 계산
        #     cx_sum, cy_sum, count = 0, 0, 0
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
                
        #         # 수직에 가까운 선만 선택 (차선일 가능성 높음)
        #         if abs(x2 - x1) < self.roi_height / 4:  # 수직에 가까움
        #             cx_sum += (x1 + x2) // 2
        #             cy_sum += (y1 + y2) // 2 + self.roi_y
        #             count += 1
            
        #     if count > 0:
        #         cx = cx_sum // count
        #         cy = cy_sum // count
        #     else:
        #         # 모멘트 기반 감지로 대체
        #         M = cv2.moments(self.binary_roi)
        #         if M["m00"] != 0:
        #             cx = int(M["m10"] / M["m00"])
        #             cy = int(M["m01"] / M["m00"]) + self.roi_y
        #         else:
        #             cx, cy = self.width // 2, self.height // 2
        # else:
        #     # 모멘트 기반 감지 (대체 방법)
        M = cv2.moments(self.binary_roi)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + self.roi_y
        else:
            cx, cy = self.width // 2, self.height // 2
    
        # 차선 중심과 이미지 중심의 편차 계산
        center_deviation = cx - (self.width // 2)
        
        # 시각화 (디버그 모드에서만)
        if self.debug_view:
            # 중앙점 표시
            cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
            
            # 차선 중심선 표시
            cv2.line(frame, (cx, self.roi_y), (cx, self.height), (255, 0, 0), 1)
            
            # ROI 영역 표시
            cv2.rectangle(frame, (0, self.roi_y), (self.width, self.height), (0, 255, 0), 1)
            
            # 중앙 기준선 표시
            center_x = self.width // 2
            cv2.line(frame, (center_x, self.roi_y), (center_x, self.height), (0, 255, 255), 1)
            
            # 편차 텍스트 표시
            cv2.putText(
                frame, f"Dev: {center_deviation}", 
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
        
        return frame
    
    def destroy_node(self):
        """노드 종료 시 자원 정리"""
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        
        cv2.destroyAllWindows()
        self.get_logger().info("Lane detection node stopped")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedLaneDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()