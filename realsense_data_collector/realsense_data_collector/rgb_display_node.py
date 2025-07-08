import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import message_filters
from tf2_ros import TransformBroadcaster
from collections import deque

class OptimizedArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # CV Bridge (재사용)
        self.bridge = CvBridge()
        
        # 라즈베리파이 최적화 파라미터
        self.target_width = 320
        self.target_height = 240
        self.target_fps = 15  # 조금 높임 (처리 효율화로)
        self.frame_delay = 1.0 / self.target_fps
        
        # 프레임 스키핑 (처리 부하 감소)
        self.frame_skip = 2  # 2프레임마다 1번 처리
        self.frame_count = 0
        
        # 동적 품질 조절
        self.processing_time_history = deque(maxlen=10)
        self.adaptive_quality = True
        self.current_quality = 1.0  # 1.0 = 풀 해상도
        
        # 메모리 재사용을 위한 버퍼들
        self.rgb_buffer = None
        self.depth_buffer = None
        self.gray_buffer = None
        
        # RealSense 토픽 구독 설정
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        
        # 카메라 정보 구독
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 1)
        
        # 적응적 동기화 (슬롭 증가)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.1)  # 큐 크기 감소
        self.ts.registerCallback(self.synchronized_callback)
        
        # 퍼블리셔 (큐 크기 최적화)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/marker_pose', 1)
        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 1)
        
        # TF 브로드캐스터
        self.tf_broadcaster = TransformBroadcaster(self)

        # ArUco 설정 (최적화)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # ArUco 파라미터 최적화 (속도 우선)
        self.aruco_params.adaptiveThreshWinSizeMin = 13  # 기본값보다 크게
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 5  # 더 큰 스텝
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE  # 코너 개선 비활성화
        self.aruco_params.cornerRefinementWinSize = 3  # 작게 설정
        self.aruco_params.cornerRefinementMaxIterations = 10  # 반복 횟수 감소
        
        # OpenCV 버전에 따른 검출기 생성
        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except:
            self.detector = None
            self.use_new_api = False
        
        # 파라미터
        self.declare_parameter('marker_size', 0.1)
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        
        # 카메라 파라미터 (초기값 설정)
        self.camera_fx = 320.0
        self.camera_fy = 320.0
        self.camera_cx = 160.0
        self.camera_cy = 120.0
        self.camera_info_received = False
        
        # 뎁스 처리 최적화
        self.depth_scale = 0.001
        self.depth_filter_size = 3  # 필터 크기 감소
        self.min_depth = 0.2  # 범위 줄임
        self.max_depth = 5.0
        
        # 성능 모니터링 (올바른 계산)
        self.last_frame_time = time.time()
        self.start_time = time.time()
        self.processed_frames = 0  # 실제 처리된 프레임 수
        self.total_frames = 0      # 전체 받은 프레임 수
        self.performance_counter = 0
        self.total_processing_time = 0
        
        # 디버그 설정 (마커 검출 확인용)
        self.show_debug_window = True   # OpenCV 윈도우 활성화 (마커 확인용)
        self.publish_debug_image = True  # 디버그 이미지 발행 활성화
        self.enable_logging = True       # 상세 로깅 활성화
        self.show_detection_info = True  # 검출 정보 표시
        
        # 뎁스 히스토리 (안정성)
        self.depth_history = {}  # marker_id: deque of depths
        self.max_history_size = 5
        
        self.get_logger().info(f"ArUco Detector 시작 - 해상도: {self.target_width}x{self.target_height}")
        self.get_logger().info(f"마커 크기: {self.marker_size}m")

    def camera_info_callback(self, msg):
        """카메라 파라미터 수신 (한 번만)"""
        if not self.camera_info_received:
            self.camera_fx = msg.k[0]
            self.camera_fy = msg.k[4]
            self.camera_cx = msg.k[2]
            self.camera_cy = msg.k[5]
            self.camera_info_received = True
            
            self.get_logger().info(f"카메라 파라미터 로드됨: fx={self.camera_fx:.1f}, fy={self.camera_fy:.1f}")
            
            # 카메라 정보 구독 해제 (메모리 절약)
            self.destroy_subscription(self.camera_info_sub)

    def synchronized_callback(self, rgb_msg, depth_msg):
        """최적화된 동기화 콜백"""
        self.total_frames += 1
        
        # 프레임 스키핑
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return
            
        # FPS 제어
        current_time = time.time()
        if (current_time - self.last_frame_time) < self.frame_delay:
            return

        processing_start = time.time()
        self.processed_frames += 1

        try:
            # 효율적인 이미지 변환
            rgb_image = self.efficient_image_conversion(rgb_msg, 'bgr8')
            depth_image = self.efficient_image_conversion(depth_msg, '16UC1')
            
            if rgb_image is None or depth_image is None:
                return

            # 동적 해상도 조절
            if self.adaptive_quality:
                rgb_image, depth_image = self.adaptive_resize(rgb_image, depth_image)
            else:
                rgb_image, depth_image = self.standard_resize(rgb_image, depth_image)

            # 카메라 파라미터 스케일링 (한 번만 계산)
            fx, fy, cx, cy = self.get_scaled_camera_params(rgb_image.shape[1], rgb_image.shape[0])

            # ArUco 검출 (최적화)
            corners, ids = self.detect_aruco_optimized(rgb_image)
            
            # 디버그 이미지 생성
            debug_image = rgb_image.copy()
            
            if ids is not None and len(ids) > 0:
                # 마커 검출됨 - 시각화
                cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
                
                self.process_detected_markers(ids, corners, depth_image, fx, fy, cx, cy, rgb_msg.header, debug_image)
            else:
                # 마커 검출 안됨 - 조용히 처리
                pass
            
            # 디버그 이미지 표시
            if self.show_debug_window:
                cv2.imshow('ArUco Detection', debug_image)
                cv2.waitKey(1)
            
            if self.publish_debug_image:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)
            
            # 성능 모니터링 및 적응적 품질 조절
            processing_time = time.time() - processing_start
            self.update_performance_metrics(processing_time)
            
            self.last_frame_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"처리 오류: {e}")

    def efficient_image_conversion(self, msg, encoding):
        """효율적인 이미지 변환 (메모리 재사용)"""
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            return None

    def adaptive_resize(self, rgb_image, depth_image):
        """적응적 이미지 리사이즈"""
        # 성능에 따라 동적으로 해상도 조절
        if self.current_quality < 1.0:
            width = int(self.target_width * self.current_quality)
            height = int(self.target_height * self.current_quality)
            # 최소 해상도 보장
            width = max(160, width)
            height = max(120, height)
        else:
            width, height = self.target_width, self.target_height
        
        rgb_resized = cv2.resize(rgb_image, (width, height), interpolation=cv2.INTER_LINEAR)
        depth_resized = cv2.resize(depth_image, (width, height), interpolation=cv2.INTER_NEAREST)
        
        return rgb_resized, depth_resized

    def standard_resize(self, rgb_image, depth_image):
        """표준 리사이즈"""
        rgb_resized = cv2.resize(rgb_image, (self.target_width, self.target_height), 
                               interpolation=cv2.INTER_LINEAR)
        depth_resized = cv2.resize(depth_image, (self.target_width, self.target_height), 
                                 interpolation=cv2.INTER_NEAREST)
        return rgb_resized, depth_resized

    def get_scaled_camera_params(self, width, height):
        """스케일된 카메라 파라미터 계산"""
        scale_x = width / 640.0  # RealSense 기본 해상도 기준
        scale_y = height / 480.0
        
        fx = self.camera_fx * scale_x
        fy = self.camera_fy * scale_y
        cx = self.camera_cx * scale_x
        cy = self.camera_cy * scale_y
        
        return fx, fy, cx, cy

    def detect_aruco_optimized(self, rgb_image):
        """최적화된 ArUco 검출"""
        # 그레이스케일 변환 (메모리 재사용)
        if self.gray_buffer is None or self.gray_buffer.shape != rgb_image.shape[:2]:
            self.gray_buffer = np.empty(rgb_image.shape[:2], dtype=np.uint8)
        
        cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY, dst=self.gray_buffer)
        
        # ArUco 검출
        if self.use_new_api and self.detector:
            corners, ids, _ = self.detector.detectMarkers(self.gray_buffer)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                self.gray_buffer, self.aruco_dict, parameters=self.aruco_params)
        
        return corners, ids

    def process_detected_markers(self, ids, corners, depth_image, fx, fy, cx, cy, header, debug_image):
        """검출된 마커들 처리"""
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            
            # 뎁스값 계산
            depth_value = self.get_fast_depth(depth_image, center_x, center_y)
            
            if self.min_depth <= depth_value <= self.max_depth:
                # 뎁스 히스토리 필터링
                filtered_depth = self.filter_depth_with_history(marker_id, depth_value)
                
                # 3D 위치 계산
                world_pos = self.pixel_to_world_fast(center_x, center_y, filtered_depth, fx, fy, cx, cy)
                
                # 방향
                orientation = [0.0, 0.0, 0.0, 1.0]
                
                # 포즈 발행
                self.publish_marker_pose_fast(marker_id, world_pos, orientation, header)
                
                # 실시간 위치 출력 (매 프레임)
                print(f"마커 {marker_id}: X={world_pos[0]:.3f}, Y={world_pos[1]:.3f}, Z={world_pos[2]:.3f}m")
                
                # 디버그 이미지에 정보 표시
                info_text = f"ID:{marker_id}"
                cv2.putText(debug_image, info_text, (center_x-30, center_y-40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                depth_text = f"{filtered_depth:.2f}m"
                cv2.putText(debug_image, depth_text, (center_x-30, center_y-20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                pos_text = f"({world_pos[0]:.2f},{world_pos[1]:.2f},{world_pos[2]:.2f})"
                cv2.putText(debug_image, pos_text, (center_x-60, center_y+20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # 중심점과 축 표시
                cv2.circle(debug_image, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.arrowedLine(debug_image, (center_x, center_y), 
                               (center_x + 30, center_y), (0, 0, 255), 2)
                cv2.arrowedLine(debug_image, (center_x, center_y), 
                               (center_x, center_y - 30), (0, 255, 0), 2)

    def get_fast_depth(self, depth_image, center_x, center_y):
        """빠른 뎁스값 계산 (개선된 필터링)"""
        h, w = depth_image.shape
        
        # 5x5 윈도우로 확장 (더 안정적)
        window_size = 5
        half_size = window_size // 2
        x1 = max(0, center_x - half_size)
        x2 = min(w, center_x + half_size + 1)
        y1 = max(0, center_y - half_size)
        y2 = min(h, center_y + half_size + 1)
        
        depth_window = depth_image[y1:y2, x1:x2]
        valid_depths = depth_window[depth_window > 0]
        
        if len(valid_depths) >= 5:  # 최소 5개 유효값 필요
            # 이상치 제거
            depths_m = valid_depths.astype(np.float32) * self.depth_scale
            
            # 중심값 우선 - 중심 픽셀이 유효하면 가중치 부여
            center_depth = depth_image[center_y, center_x] * self.depth_scale
            if center_depth > 0:
                # 중심값이 유효하면 중심값과 주변값의 가중 평균
                median_depth = np.median(depths_m)
                if abs(center_depth - median_depth) < 0.1:  # 중심값이 합리적이면
                    return center_depth * 0.7 + median_depth * 0.3
            
            return np.median(depths_m)
        elif len(valid_depths) > 0:
            return np.median(valid_depths) * self.depth_scale
        else:
            return 0.0

    def filter_depth_with_history(self, marker_id, depth_value):
        """뎁스 히스토리를 이용한 필터링"""
        if marker_id not in self.depth_history:
            self.depth_history[marker_id] = deque(maxlen=self.max_history_size)
        
        self.depth_history[marker_id].append(depth_value)
        
        # 이동 평균
        return np.mean(list(self.depth_history[marker_id]))

    def pixel_to_world_fast(self, pixel_x, pixel_y, depth_m, fx, fy, cx, cy):
        """빠른 3D 변환"""
        z = depth_m
        x = (pixel_x - cx) * z / fx
        y = (pixel_y - cy) * z / fy
        return [x, y, z]

    def publish_marker_pose_fast(self, marker_id, position, orientation, header):
        """빠른 포즈 발행 (TF 생략 옵션)"""
        # PoseStamped만 발행 (TF는 필요시에만)
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_color_optical_frame"
        
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        
        self.pose_pub.publish(pose_msg)

    def update_performance_metrics(self, processing_time):
        """성능 메트릭 업데이트 및 적응적 품질 조절"""
        self.processing_time_history.append(processing_time)
        self.performance_counter += 1
        self.total_processing_time += processing_time
        
        # 적응적 품질 조절
        if self.adaptive_quality and len(self.processing_time_history) >= 5:
            avg_time = np.mean(list(self.processing_time_history))
            target_time = self.frame_delay * 0.8  # 타겟 시간의 80%
            
            if avg_time > target_time:
                # 처리 시간이 길면 품질 낮춤
                self.current_quality = max(0.5, self.current_quality - 0.1)
            elif avg_time < target_time * 0.5:
                # 처리 시간이 충분히 짧으면 품질 높임
                self.current_quality = min(1.0, self.current_quality + 0.1)
        
        # 성능 로깅 (30프레임마다) - 간단하게
        if self.processed_frames % 30 == 0 and self.processed_frames > 0:
            elapsed_time = time.time() - self.start_time
            actual_fps = self.processed_frames / elapsed_time if elapsed_time > 0 else 0
            avg_processing_time = self.total_processing_time / self.processed_frames
            
            print(f"FPS: {actual_fps:.1f}, 처리시간: {avg_processing_time*1000:.1f}ms")
                
            # 5초마다 전체 통계 리셋 (메모리 관리)
            if self.processed_frames % 150 == 0:
                self.start_time = time.time()
                self.processed_frames = 0
                self.total_frames = 0
                self.total_processing_time = 0

def main():
    rclpy.init()
    
    print("ArUco 마커 검출기 시작...")
    
    node = OptimizedArucoDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()