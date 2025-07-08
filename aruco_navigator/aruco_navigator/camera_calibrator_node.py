import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os

class CameraCalibratorNode(Node):
    def __init__(self):
        super().__init__('camera_calibrator_node')

        # ROS 토픽 구독 설정
        self.subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # 캘리브레이션 파라미터
        self.chessboard_size = (7, 5)  # 체스보드 내부 코너 개수 (가로 x 세로)
        self.square_size_mm = 27.5    # 체스보드 한 칸의 실제 크기 (mm)
        self.square_size_m = self.square_size_mm / 1000.0 # 미터 단위
        self.target_width = 320      # 캘리브레이션에 사용할 이미지 폭
        self.target_height = 320     # 캘리브레이션에 사용할 이미지 높이
        self.capture_interval_sec = 1.0 # 이미지를 캡처할 최소 시간 간격 (1초)
        self.min_images_for_calibration = 40 # 캘리브레이션에 필요한 최소 이미지 수

        # 캘리브레이션 데이터 저장 변수
        self.objpoints = []  # 3D 점 (체스보드의 실제 3D 좌표)
        self.imgpoints = []  # 2D 점 (이미지 평면의 코너 좌표)
        self.last_capture_time = time.time()
        self.image_count = 0

        # 체스보드의 3D 좌표 생성
        # Z=0 평면에 있는 점들. (0,0,0), (1,0,0), (2,0,0) ...
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2) * self.square_size_m

        self.get_logger().info(f"카메라 캘리브레이터 노드 시작. 타겟 해상도: {self.target_width}x{self.target_height}, 체스보드: {self.chessboard_size}, 사각형 크기: {self.square_size_mm}mm")
        self.get_logger().info(f"최소 {self.min_images_for_calibration}개의 이미지를 성공적으로 캡처해야 캘리브레이션이 시작됩니다.")

        # 카메라 파라미터 선언 (캘리브레이션 후 퍼블리싱 예정)
        self.declare_parameter('camera_matrix', [])
        self.declare_parameter('dist_coeffs', [])

    def image_callback(self, msg):
        current_time = time.time()

        # 이미지를 너무 빠르게 처리하지 않도록 제어
        if (current_time - self.last_capture_time) < self.capture_interval_sec:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge 오류: {e}")
            return

        # 이미지 크기 조정 (캘리브레이션에 사용할 해상도로)
        if cv_image.shape[0] != self.target_height or cv_image.shape[1] != self.target_width:
            cv_image = cv2.resize(cv_image, (self.target_width, self.target_height), interpolation=cv2.INTER_AREA)
            
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 체스보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

        if ret:
            # 코너를 찾았을 경우
            # 코너 위치를 더 정확하게 조정
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                              (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners_refined)
            self.image_count += 1
            self.last_capture_time = current_time

            self.get_logger().info(f"체스보드 이미지 캡처 성공: {self.image_count} / {self.min_images_for_calibration}개")

            # 이미지에 코너 그리기 (시각화)
            cv2.drawChessboardCorners(cv_image, self.chessboard_size, corners_refined, ret)
            
            # 최소 이미지 수에 도달하면 캘리브레이션 수행
            if self.image_count >= self.min_images_for_calibration:
                self.get_logger().info("최소 이미지 수에 도달. 카메라 캘리브레이션 시작...")
                self.perform_calibration(gray.shape[::-1]) # 이미지 크기 (width, height)

        else:
            self.get_logger().debug("체스보드 코너를 찾을 수 없음.")

        # 결과 이미지 디스플레이
        cv2.imshow('Calibration Board', cv_image)
        cv2.waitKey(1)

    def perform_calibration(self, img_size):
        if len(self.objpoints) < self.min_images_for_calibration:
            self.get_logger().warn("캘리브레이션에 필요한 이미지 수가 부족합니다.")
            return

        try:
            # 캘리브레이션 수행
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, img_size, None, None)

            if ret:
                self.get_logger().info("--- 카메라 캘리브레이션 완료! ---")
                self.get_logger().info(f"\n카메라 매트릭스 (fx, fy, cx, cy):\n{camera_matrix}")
                self.get_logger().info(f"\n왜곡 계수 (k1, k2, p1, p2, k3):\n{dist_coeffs}")

                # 캘리브레이션 결과를 ROS2 파라미터로 설정
                # 이를 통해 다른 노드에서 이 파라미터를 읽어올 수 있습니다.
                self.set_parameters([
                    rclpy.Parameter('camera_matrix', rclpy.Parameter.Type.DOUBLE_ARRAY, camera_matrix.flatten().tolist()),
                    rclpy.Parameter('dist_coeffs', rclpy.Parameter.Type.DOUBLE_ARRAY, dist_coeffs.flatten().tolist())
                ])
                self.get_logger().info("캘리브레이션 결과가 ROS2 파라미터로 설정되었습니다.")

                # (선택 사항) 결과를 파일로 저장
                self.save_calibration_to_file(camera_matrix, dist_coeffs)
                
                # 캘리브레이션 성공 후 노드 종료
                self.get_logger().info("캘리브레이션이 완료되어 노드를 종료합니다. 결과를 파일에서 확인하세요.")
                rclpy.shutdown() # 노드 및 ROS2 시스템 종료

            else:
                self.get_logger().error("카메라 캘리브레이션에 실패했습니다.")

        except Exception as e:
            self.get_logger().error(f"캘리브레이션 중 오류 발생: {e}")

    def save_calibration_to_file(self, camera_matrix, dist_coeffs):
        output_dir = os.path.expanduser('~/ros2_ws/src/aruco_navigator/config')
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, 'calibrated_camera_params_320x320.yaml')

        with open(filepath, 'w') as f:
            f.write(f"camera_matrix:\n")
            f.write(f"  data: {camera_matrix.flatten().tolist()}\n")
            f.write(f"  rows: {camera_matrix.shape[0]}\n")
            f.write(f"  cols: {camera_matrix.shape[1]}\n")
            f.write(f"dist_coeffs:\n")
            f.write(f"  data: {dist_coeffs.flatten().tolist()}\n")
            f.write(f"  rows: {dist_coeffs.shape[0]}\n")
            f.write(f"  cols: {dist_coeffs.shape[1]}\n")
        
        self.get_logger().info(f"캘리브레이션 결과가 '{filepath}' 에 저장되었습니다.")


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibratorNode()
    try:
        rclpy.spin(node)
    except SystemExit: # rclpy.shutdown() 호출 시 발생하는 SystemExit 예외 처리
        pass
    except KeyboardInterrupt:
        node.get_logger().info("노드 종료 (Keyboard Interrupt).")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok(): # 이미 shutdown되지 않았을 경우에만 호출
            rclpy.shutdown()

if __name__ == '__main__':
    main()