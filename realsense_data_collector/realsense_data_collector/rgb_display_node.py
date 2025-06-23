
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime
import numpy as np

class RealSenseRGBDisplay(Node):

    def __init__(self):
        super().__init__('realsense_rgb_display_node_py')
        self.get_logger().info('RealSense RGB Display Node (Python) has been started.')

        # CvBridge 인스턴스 생성
        self.br = CvBridge()
        
        # 현재 이미지 저장
        self.current_rgb_image = None
        self.current_depth_image = None
        
        # 깊이 이미지 처리를 위한 변수
        self.depth_min = 0
        self.depth_max = 10000  # 초기값, 실제 데이터에 따라 자동 조정됨
        self.depth_scale = 0.001  # 미터 단위로 스케일링 (기본값, RealSense 카메라마다 다를 수 있음)
        
        # 이미지 저장 경로 설정
        # 먼저 환경 변수에서 홈 디렉토리 가져오기
        self.home_dir = os.path.expanduser('~')
        self.seg_project_dir = os.path.join(self.home_dir, 'segmentation_project')
        
        # 이미지 저장 디렉토리 설정 (기본값: train)
        self.dataset_type = 'train'  # 'train', 'val', 'test' 중 하나
        self.image_save_dir = os.path.join(self.seg_project_dir, 'data', 'raw', 'images', self.dataset_type)
        self.depth_save_dir = os.path.join(self.seg_project_dir, 'data', 'raw', 'depth', self.dataset_type)
        
        # 디렉토리 존재 확인 및 생성
        os.makedirs(self.image_save_dir, exist_ok=True)
        os.makedirs(self.depth_save_dir, exist_ok=True)
        
        self.get_logger().info(f'Images will be saved to: {self.image_save_dir}')
        self.get_logger().info(f'Depth images will be saved to: {self.depth_save_dir}')
        
        # 저장된 이미지 카운터
        self.saved_count = 0

        # RGB 이미지 토픽 구독 설정
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/realsense/color/image_raw',
            self.rgb_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1
            ) 
        )
        
        # 깊이 이미지 토픽 구독 설정
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/realsense/depth/image_rect_raw',  # 깊이 이미지 토픽 이름
            self.depth_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1
            ) 
        )

        # OpenCV 윈도우 생성
        cv2.namedWindow("RealSense Camera Feed", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info('OpenCV window created. Waiting for images...')
        self.get_logger().info('Press SPACE to save the current images.')
        self.get_logger().info('Press T/V/E to switch between train/val/test datasets.')
        self.get_logger().info('Press Q to quit.')
        
        # 디스플레이 타이머 생성 (주기적으로 이미지 업데이트)
        self.display_timer = self.create_timer(0.05, self.display_images)  # 20fps

    def __del__(self):
        # 노드 소멸 시 OpenCV 윈도우 닫기
        cv2.destroyAllWindows()
        self.get_logger().info('OpenCV window destroyed.')

    def rgb_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 현재 이미지 저장
            self.current_rgb_image = cv_image.copy()
            
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')
            
    def depth_callback(self, msg):
        try:
            # 깊이 이미지 변환
            cv_depth = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 현재 깊이 이미지 저장
            self.current_depth_image = cv_depth.copy()
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def display_images(self):
        # 두 이미지 모두 받았는지 확인
        if self.current_depth_image is None or self.current_rgb_image is None:
            return
            
        try:
            # 깊이 이미지 시각화 (colormap 적용)
            depth_display = self.current_depth_image.copy()
            
            # 최소/최대 깊이 값을 로그에 기록 (디버깅용)
            min_val = np.min(depth_display) if depth_display.size > 0 else 0
            max_val = np.max(depth_display) if depth_display.size > 0 else 0
            
            # 깊이 값 범위가 너무 좁으면 스케일 조정
            if max_val - min_val < 100:  # 범위가 너무 작으면 확장
                self.get_logger().warn(f"Small depth range detected: min={min_val}, max={max_val}")
                # 범위를 인위적으로 확장
                depth_display = (depth_display - min_val) * (255.0 / max(1, max_val - min_val))
                depth_display = depth_display.astype(np.uint8)
            else:
                # 깊이 이미지를 8비트로 변환하고 컬러맵 적용
                depth_display = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # 컬러맵 적용
            depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
            
            # 원본 비율을 유지하면서 192x192 크기로 리사이즈
            
            # RGB 이미지 리사이즈
            h_rgb, w_rgb = self.current_rgb_image.shape[:2]
            if h_rgb > w_rgb:
                new_h_rgb, new_w_rgb = 192, int(w_rgb * 192 / h_rgb)
            else:
                new_h_rgb, new_w_rgb = int(h_rgb * 192 / w_rgb), 192
                
            rgb_resized = cv2.resize(self.current_rgb_image, (new_w_rgb, new_h_rgb))
            
            # 깊이 이미지 리사이즈
            h_depth, w_depth = depth_colormap.shape[:2]
            if h_depth > w_depth:
                new_h_depth, new_w_depth = 192, int(w_depth * 192 / h_depth)
            else:
                new_h_depth, new_w_depth = int(h_depth * 192 / w_depth), 192
                
            depth_resized = cv2.resize(depth_colormap, (new_w_depth, new_h_depth))
            
            # 두 이미지를 위한 캔버스 생성 (좌측 깊이, 우측 RGB)
            display_width = 480  # 두 이미지 + 중간 간격
            display_image = np.zeros((192, display_width, 3), dtype=np.uint8)
            
            # 리사이즈된 이미지를 각 영역의 중앙에 배치
            # 왼쪽 (깊이 이미지)
            y_offset_depth = (192 - new_h_depth) // 2
            x_offset_depth = (192 - new_w_depth) // 2
            display_image[y_offset_depth:y_offset_depth+new_h_depth, x_offset_depth:x_offset_depth+new_w_depth] = depth_resized
            
            # 우측 (RGB 이미지)
            y_offset_rgb = (192 - new_h_rgb) // 2
            x_offset_rgb = 192 + (192 - new_w_rgb) // 2  # 우측 절반 시작 위치
            display_image[y_offset_rgb:y_offset_rgb+new_h_rgb, x_offset_rgb:x_offset_rgb+new_w_rgb] = rgb_resized
            
            # 중앙 구분선 그리기
            cv2.line(display_image, (192, 0), (192, 192), (255, 255, 255), 1)
            
            # "DEPTH" 및 "RGB" 라벨 추가
            cv2.putText(display_image, "DEPTH", (90, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(display_image, "RGB", (192 + 90, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # 깊이 값 범위 표시 
            cv2.putText(display_image, f"D: {min_val}-{max_val}", (5, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            
            # 저장 정보 표시
            info_text = f"Dataset: {self.dataset_type} | Saved: {self.saved_count} | SPACE: Save"
            cv2.putText(display_image, info_text, (10, 210), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # 이미지 표시
            cv2.imshow("RealSense Camera Feed", display_image)
            
            # 키 입력 처리
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' '):  # 스페이스바
                self.save_current_images()
            elif key == ord('t'):  # 't' 키: 훈련 세트로 전환
                self.dataset_type = 'train'
                self.update_save_paths()
            elif key == ord('v'):  # 'v' 키: 검증 세트로 전환
                self.dataset_type = 'val'
                self.update_save_paths()
            elif key == ord('e'):  # 'e' 키: 테스트 세트로 전환
                self.dataset_type = 'test'
                self.update_save_paths()
            elif key == ord('q'):  # 'q' 키: 종료
                self.get_logger().info('Quit requested by user')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error displaying images: {str(e)}')
            
    def update_save_paths(self):
        # 데이터셋 유형이 변경되면 저장 경로 업데이트
        self.image_save_dir = os.path.join(self.seg_project_dir, 'data', 'raw', 'images', self.dataset_type)
        self.depth_save_dir = os.path.join(self.seg_project_dir, 'data', 'raw', 'depth', self.dataset_type)
        
        # 디렉토리 존재 확인 및 생성
        os.makedirs(self.image_save_dir, exist_ok=True)
        os.makedirs(self.depth_save_dir, exist_ok=True)
        
        self.get_logger().info(f'Dataset changed to: {self.dataset_type}')
        self.get_logger().info(f'Images will be saved to: {self.image_save_dir}')

    def save_current_images(self):
        if self.current_rgb_image is None or self.current_depth_image is None:
            self.get_logger().warn('Both RGB and depth images must be available to save')
            return
            
        try:
            # 현재 타임스탬프 생성
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            
            # RGB 이미지 저장 (192x192로 리사이즈)
            rgb_filename = os.path.join(self.image_save_dir, f"{timestamp}_rgb.jpg")
            rgb_resized = cv2.resize(self.current_rgb_image, (192, 192))
            cv2.imwrite(rgb_filename, rgb_resized)
            
            # 깊이 이미지 저장 (원본 데이터)
            depth_raw_filename = os.path.join(self.depth_save_dir, f"{timestamp}_depth_raw.png")
            depth_resized = cv2.resize(self.current_depth_image, (192, 192))
            cv2.imwrite(depth_raw_filename, depth_resized)
            
            # 시각화된 깊이 이미지도 저장 (컬러맵 적용)
            # 깊이 이미지를 8비트로 변환하고 컬러맵 적용
            depth_display = cv2.normalize(depth_resized, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
            depth_vis_filename = os.path.join(self.depth_save_dir, f"{timestamp}_depth_vis.png")
            cv2.imwrite(depth_vis_filename, depth_colormap)
            
            self.saved_count += 1
            self.get_logger().info(f'Images saved successfully ({self.saved_count}):')
            self.get_logger().info(f'  RGB: {rgb_filename}')
            self.get_logger().info(f'  Depth (Raw): {depth_raw_filename}')
            self.get_logger().info(f'  Depth (Visualized): {depth_vis_filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving images: {str(e)}')


def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)

    # 노드 생성
    image_display_node = RealSenseRGBDisplay()

    try:
        # 노드 스핀 (콜백 함수 실행)
        rclpy.spin(image_display_node)
    except KeyboardInterrupt:
        pass # Ctrl+C 시 종료

    # 노드 정리 및 종료
    image_display_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows() # 혹시 남은 윈도우가 있다면 모두 닫기

if __name__ == '__main__':
    main()