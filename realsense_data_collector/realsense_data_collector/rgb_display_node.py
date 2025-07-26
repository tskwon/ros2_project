#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime
import time

class RealSenseDataCollector(Node):
    def __init__(self):
        super().__init__('realsense_data_collector')
        
        # 파라미터 선언
        self.declare_parameter('save_path', '/home/xotn/segmentation_project/data/image')
        self.declare_parameter('display_enabled', True)
        self.declare_parameter('image_topic', '/d415/realsense_d415/color/image_raw')
        
        # 파라미터 가져오기
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        self.display_enabled = self.get_parameter('display_enabled').get_parameter_value().bool_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        # 저장 폴더 생성
        os.makedirs(self.save_path, exist_ok=True)
        self.get_logger().info(f"📁 이미지 저장 경로: {self.save_path}")
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 이미지 카운터 및 상태
        self.image_counter = 0
        self.latest_image = None
        self.save_requested = False
        
        # 성능 모니터링
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        
        # RealSense RGB 이미지 구독
        self.rgb_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.color_callback,
            10
        )
        
        # OpenCV 윈도우 설정
        if self.display_enabled:
            cv2.namedWindow('RealSense Data Collector', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Collection Info', cv2.WINDOW_AUTOSIZE)
            self.get_logger().info('🖥️ OpenCV 디스플레이가 활성화되었습니다.')
        else:
            self.get_logger().info('🚫 OpenCV 디스플레이가 비활성화되었습니다.')
        
        # 타이머로 디스플레이 업데이트 (30Hz)
        if self.display_enabled:
            self.timer = self.create_timer(0.033, self.update_display)
        
        self.get_logger().info(f"📷 토픽 구독: {self.image_topic}")
        self.get_logger().info("🎯 RealSense 데이터 수집기가 시작되었습니다!")
        self.get_logger().info("⌨️  조작법: 스페이스바(저장), ESC/Q(종료)")
        
    def color_callback(self, msg):
        """ROS2 이미지 메시지를 받아서 OpenCV 이미지로 변환"""
        try:
            # ROS2 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image.copy()
            
            # FPS 계산
            self.calculate_fps()
            
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
    
    def calculate_fps(self):
        """FPS 계산"""
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:  # 1초마다 업데이트
            self.fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def create_info_display(self):
        """정보 디스플레이 창 생성"""
        info_img = np.zeros((400, 500, 3), dtype=np.uint8)
        
        # 제목
        cv2.putText(info_img, 'RealSense Data Collector', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 구분선
        cv2.line(info_img, (10, 40), (490, 40), (255, 255, 255), 1)
        
        # 정보 표시
        y_pos = 70
        info_lines = [
            f"Topic: {self.image_topic}",
            f"Save Path: {self.save_path}",
            f"Images Saved: {self.image_counter}",
            f"FPS: {self.fps:.1f}",
            f"Camera Status: {'Connected' if self.latest_image is not None else 'Disconnected'}",
            "",
            "Controls:",
            "SPACE - Save current image",
            "ESC/Q - Exit program",
            "R - Reset counter",
            "",
            "Status:",
            f"Display: {'Enabled' if self.display_enabled else 'Disabled'}",
            f"Last Save: {datetime.now().strftime('%H:%M:%S') if self.image_counter > 0 else 'None'}"
        ]
        
        for i, line in enumerate(info_lines):
            if i < 5:  # 상태 정보는 초록색
                color = (0, 255, 0)
            elif "Controls:" in line or "Status:" in line:  # 섹션 제목은 노란색
                color = (0, 255, 255)
            else:  # 설명은 흰색
                color = (255, 255, 255)
                
            cv2.putText(info_img, line, (10, y_pos + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return info_img
    
    def update_display(self):
        """디스플레이 업데이트"""
        if not self.display_enabled:
            return
            
        try:
            # 메인 이미지 표시
            if self.latest_image is not None:
                # 이미지에 오버레이 정보 추가
                display_image = self.latest_image.copy()
                
                # 상단 정보 오버레이
                overlay = display_image.copy()
                cv2.rectangle(overlay, (0, 0), (display_image.shape[1], 60), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.7, display_image, 0.3, 0, display_image)
                
                # 정보 텍스트
                cv2.putText(display_image, f"Images: {self.image_counter} | FPS: {self.fps:.1f}", 
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_image, "SPACE: Save | ESC/Q: Exit", 
                           (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 중앙 십자선
                h, w = display_image.shape[:2]
                cv2.line(display_image, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 255), 2)
                cv2.line(display_image, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 255), 2)
                
                cv2.imshow('RealSense Data Collector', display_image)
            
            # 정보 창 표시
            info_display = self.create_info_display()
            cv2.imshow('Collection Info', info_display)
            
            # 키 입력 처리
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q') or key == ord('Q'):  # ESC 또는 Q
                self.get_logger().info('🛑 종료 요청')
                rclpy.shutdown()
            elif key == ord(' '):  # 스페이스바
                self.save_current_image()
            elif key == ord('r') or key == ord('R'):  # R - 카운터 리셋
                self.image_counter = 0
                self.get_logger().info('🔄 이미지 카운터 리셋')
                
        except Exception as e:
            self.get_logger().error(f'디스플레이 업데이트 오류: {e}')
    
    def save_current_image(self):
        """현재 이미지를 저장"""
        if self.latest_image is not None:
            # 타임스탬프 생성
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"rgb_{timestamp}_{self.image_counter:04d}.jpg"
            filepath = os.path.join(self.save_path, filename)
            
            # 이미지 저장
            success = cv2.imwrite(filepath, self.latest_image)
            
            if success:
                self.image_counter += 1
                self.get_logger().info(f"💾 이미지 저장 완료 ({self.image_counter}): {filename}")
                
                # 콘솔에도 간단한 피드백
                print(f"📸 #{self.image_counter:04d} 저장됨: {filename}")
            else:
                self.get_logger().error(f"❌ 이미지 저장 실패: {filename}")
        else:
            self.get_logger().warn("⚠️ 저장할 이미지가 없습니다. 카메라 연결을 확인하세요.")
    
    def destroy_node(self):
        """노드 종료시 정리"""
        if self.display_enabled:
            cv2.destroyAllWindows()
        self.get_logger().info(f"✅ 총 {self.image_counter}개의 이미지가 저장되었습니다.")
        super().destroy_node()

def main():
    # ROS2 초기화
    rclpy.init()
    
    print("\n🚀 ROS2 RealSense 데이터 수집기")
    print("📷 RGB 이미지 수집 프로그램")
    print("=" * 40)
    
    # 노드 생성
    node = RealSenseDataCollector()
    
    try:
        # ROS2 스핀
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n⏹️ Ctrl+C로 종료되었습니다.")
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
    finally:
        # 정리
        try:
            node.destroy_node()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        cv2.destroyAllWindows()
        print("🔚 프로그램이 정상적으로 종료되었습니다.")

if __name__ == '__main__':
    main()