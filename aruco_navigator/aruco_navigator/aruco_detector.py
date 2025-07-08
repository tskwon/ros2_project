import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

class FastArucoWithDepth(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.target_width = 320
        self.target_height = 240
        self.frame_count = 0
        self.frame_skip = 2
        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 1)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 1)
        self.target_id_sub = self.create_subscription(Int32, '/aruco/target_id', self.target_id_callback, 1)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/marker_pose', 1)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 9
        self.aruco_params.adaptiveThreshWinSizeStep = 3

        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except:
            self.detector = None
            self.use_new_api = False

        # Camera parameters
        self.camera_fx = 154.25
        self.camera_fy = 154.25
        self.camera_cx = 160.0
        self.camera_cy = 120.0
        self.camera_info_received = False

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_header = None

        self.depth_scale = 0.001
        self.min_depth = 0.3
        self.max_depth = 3.0

        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.last_process_time = time.time()

        self.rgb_small = np.zeros((self.target_height, self.target_width, 3), dtype=np.uint8)
        self.depth_small = np.zeros((self.target_height, self.target_width), dtype=np.uint16)
        self.gray_buffer = np.zeros((self.target_height, self.target_width), dtype=np.uint8)

        cv2.namedWindow('ArUco with Depth', cv2.WINDOW_AUTOSIZE)

        self.target_id = -1

        # Marker size (in meters) for pose estimation
        self.marker_size = 0.1  
        
        # 큐브 높이를 마커 크기의 절반으로 설정하고 위쪽으로 뻗어나가도록 z 좌표를 양수로 설정
        cube_height = self.marker_size / 2.0
        self.cube_points = np.float32([
            [-self.marker_size/2, -self.marker_size/2, 0],  # Bottom-left (마커 평면)
            [ self.marker_size/2, -self.marker_size/2, 0],  # Bottom-right (마커 평면)
            [ self.marker_size/2,  self.marker_size/2, 0],  # Top-right (마커 평면)
            [-self.marker_size/2,  self.marker_size/2, 0],  # Top-left (마커 평면)
            # 위쪽으로 뻗어나가는 큐브의 상단 점들 (z값이 양수)
            [-self.marker_size/2, -self.marker_size/2, cube_height], 
            [ self.marker_size/2, -self.marker_size/2, cube_height],
            [ self.marker_size/2,  self.marker_size/2, cube_height],
            [-self.marker_size/2,  self.marker_size/2, cube_height]
        ])

        self.get_logger().info("Fast ArUco+Depth detector started")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            scale_x = self.target_width / 640.0
            scale_y = self.target_height / 480.0
            self.camera_fx = msg.k[0] * scale_x
            self.camera_fy = msg.k[4] * scale_y
            self.camera_cx = msg.k[2] * scale_x
            self.camera_cy = msg.k[5] * scale_y
            self.camera_info_received = True
            self.get_logger().info(f"Camera parameters: fx={self.camera_fx:.1f}")
            self.destroy_subscription(self.camera_info_sub)

    def target_id_callback(self, msg):
        self.target_id = msg.data
        self.get_logger().info(f"Target marker ID: {self.target_id}")

    def rgb_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.resize(rgb_image, (self.target_width, self.target_height),
                         dst=self.rgb_small, interpolation=cv2.INTER_LINEAR)
            self.latest_rgb = self.rgb_small.copy()
            self.latest_header = msg.header
        except Exception as e:
            self.get_logger().error(f"RGB error: {e}")

    def depth_callback(self, msg):
        if self.latest_rgb is None:
            return

        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        current_time = time.time()
        if current_time - self.last_process_time < 0.04:
            return
        self.last_process_time = current_time

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            cv2.resize(depth_image, (self.target_width, self.target_height),
                         dst=self.depth_small, interpolation=cv2.INTER_NEAREST)
            self.process_with_depth()
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def draw_axes(self, image, rvec, tvec, camera_matrix, dist_coeffs, length=0.05):
        """마커의 방향 벡터(축)를 그리는 함수 - 월드 좌표계에 맞게 수정"""
        # 월드 좌표계에 맞는 축 방향 정의
        # X: 빨강 (앞쪽), Y: 초록 (왼쪽), Z: 파랑 (위쪽)
        axis_points = np.float32([
            [0, 0, 0],          # 원점
            [length, 0, 0],     # X축 (빨강) - 앞쪽
            [0, length, 0],     # Y축 (초록) - 왼쪽  
            [0, 0, length]      # Z축 (파랑) - 위쪽
        ]).reshape(-1, 3)

        # 3D 점들을 2D 이미지 평면으로 투영
        imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        origin = tuple(imgpts[0].ravel())
        x_axis = tuple(imgpts[1].ravel())
        y_axis = tuple(imgpts[2].ravel())
        z_axis = tuple(imgpts[3].ravel()) 

        # 각 축 그리기
        cv2.line(image, origin, x_axis, (0, 0, 255), 3)  # X축 (빨강)
        cv2.line(image, origin, y_axis, (0, 255, 0), 3)  # Y축 (초록)
        cv2.line(image, origin, z_axis, (255, 0, 0), 3)  # Z축 (파랑)

        # 각 축에 라벨 추가
        cv2.putText(image, "X", x_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(image, "Y", y_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(image, "Z", z_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return image

    def correct_marker_pose(self, rvec, tvec):
        """바닥에 놓인 마커의 포즈를 월드 좌표계에 맞게 보정"""
        # 회전 벡터를 회전 행렬로 변환
        rmat, _ = cv2.Rodrigues(rvec)
        
        # 카메라가 아래를 바라보고 있을 때, 바닥 마커의 Z축이 아래를 향하는 문제 해결
        # 마커의 Z축을 위쪽으로 향하도록 180도 회전 적용
        flip_matrix = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ], dtype=np.float32)
        
        # 보정된 회전 행렬
        corrected_rmat = np.dot(rmat, flip_matrix)
        
        # 회전 행렬을 다시 회전 벡터로 변환
        corrected_rvec, _ = cv2.Rodrigues(corrected_rmat)
        
        return corrected_rvec, tvec

    def process_with_depth(self):
        try:
            cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2GRAY, dst=self.gray_buffer)

            if self.use_new_api and self.detector:
                corners, ids, _ = self.detector.detectMarkers(self.gray_buffer)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(self.gray_buffer, self.aruco_dict, parameters=self.aruco_params)

            display_image = self.latest_rgb.copy()

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display_image, corners, ids)

                # Camera matrix and distortion coefficients
                camera_matrix = np.array([[self.camera_fx, 0, self.camera_cx],
                                          [0, self.camera_fy, self.camera_cy],
                                          [0, 0, 1]], dtype=np.float32)
                dist_coeffs = np.zeros((4, 1))

                for i, marker_id in enumerate(ids.flatten()):
                    if self.target_id == -1 or marker_id != self.target_id:
                        continue

                    marker_corners = corners[i][0]
                    center_x = int(np.mean(marker_corners[:, 0]))
                    center_y = int(np.mean(marker_corners[:, 1]))

                    depth_value = self.get_fast_depth(center_x, center_y)

                    if self.min_depth <= depth_value <= self.max_depth:
                        # Define 2D-3D correspondences for the marker corners
                        marker_points = np.float32([
                            [-self.marker_size/2, -self.marker_size/2, 0],
                            [ self.marker_size/2, -self.marker_size/2, 0],
                            [ self.marker_size/2,  self.marker_size/2, 0],
                            [-self.marker_size/2,  self.marker_size/2, 0]
                        ])

                        # Estimate pose using solvePnP
                        success, rvec, tvec = cv2.solvePnP(marker_points, marker_corners, camera_matrix, dist_coeffs)

                        if success:
                            # 마커 포즈를 월드 좌표계에 맞게 보정
                            corrected_rvec, corrected_tvec = self.correct_marker_pose(rvec, tvec)
                            
                            # Project 3D cube points onto the image plane (보정된 포즈 사용)
                            cube_points_2d, _ = cv2.projectPoints(self.cube_points, corrected_rvec, corrected_tvec, camera_matrix, dist_coeffs)
                            cube_points_2d = np.int32(cube_points_2d).reshape(-1, 2)

                            # Draw cube edges
                            cv2.polylines(display_image, [cube_points_2d[0:4]], True, (0, 255, 0), 2) # Bottom face
                            cv2.polylines(display_image, [cube_points_2d[4:8]], True, (0, 255, 0), 2) # Top face
                            for j in range(4): # Connecting edges
                                cv2.line(display_image, tuple(cube_points_2d[j]), tuple(cube_points_2d[j+4]), (0, 255, 0), 2)

                            # 보정된 마커 방향 벡터 그리기
                            display_image = self.draw_axes(display_image, corrected_rvec.flatten(), corrected_tvec.flatten(), camera_matrix, dist_coeffs)

                            # Publish pose with corrected rotation
                            world_pos = corrected_tvec.flatten()
                            self.publish_3d_pose(marker_id, world_pos, corrected_rvec)

                            # Visualization
                            cv2.circle(display_image, (center_x, center_y), 4, (0, 0, 255), -1)

                            info_text = f"ID{marker_id}: {depth_value:.2f}m"
                            cv2.putText(display_image, info_text, (center_x - 40, center_y - 15),
                                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                            coord_text = f"({world_pos[0]:.2f},{world_pos[1]:.2f},{world_pos[2]:.2f})"
                            cv2.putText(display_image, coord_text, (center_x - 50, center_y + 15),
                                         cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            self.fps_counter += 1
            if time.time() - self.fps_start_time >= 1.0:
                fps = self.fps_counter / (time.time() - self.fps_start_time)
                self.fps_counter = 0
                self.fps_start_time = time.time()
                print(f"FPS: {fps:.1f}, Markers: {len(ids) if ids is not None else 0}")

            cv2.putText(display_image, f"Processing with Depth...", (10, 30),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow('ArUco with Depth', display_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    def get_fast_depth(self, x, y):
        h, w = self.depth_small.shape
        if x < 0 or x >= w or y < 0 or y >= h:
            return 0.0

        d = self.depth_small[y, x]
        if d > 0:
            return d * self.depth_scale

        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                d = self.depth_small[ny, nx]
                if d > 0:
                    return d * self.depth_scale

        for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                d = self.depth_small[ny, nx]
                if d > 0:
                    return d * self.depth_scale

        return 0.0

    def pixel_to_3d(self, x, y, depth):
        z = depth
        x_3d = (x - self.camera_cx) * z / self.camera_fx
        y_3d = (y - self.camera_cy) * z / self.camera_fy
        return [x_3d, y_3d, z]

    def publish_3d_pose(self, marker_id, world_pos, rvec):
        pose_msg = PoseStamped()
        pose_msg.header = self.latest_header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_color_optical_frame"

        pose_msg.pose.position.x = world_pos[0]
        pose_msg.pose.position.y = world_pos[1]
        pose_msg.pose.position.z = world_pos[2]

        # Convert rotation vector to quaternion
        rmat, _ = cv2.Rodrigues(rvec)
        trace = np.trace(rmat)
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (rmat[2, 1] - rmat[1, 2]) / s
            y = (rmat[0, 2] - rmat[2, 0]) / s
            z = (rmat[1, 0] - rmat[0, 1]) / s
        else:
            if rmat[0, 0] > rmat[1, 1] and rmat[0, 0] > rmat[2, 2]:
                s = math.sqrt(1.0 + rmat[0, 0] - rmat[1, 1] - rmat[2, 2]) * 2.0
                w = (rmat[2, 1] - rmat[1, 2]) / s
                x = 0.25 * s
                y = (rmat[0, 1] + rmat[1, 0]) / s
                z = (rmat[0, 2] + rmat[2, 0]) / s
            elif rmat[1, 1] > rmat[2, 2]:
                s = math.sqrt(1.0 + rmat[1, 1] - rmat[0, 0] - rmat[2, 2]) * 2.0
                w = (rmat[0, 2] - rmat[2, 0]) / s
                x = (rmat[0, 1] + rmat[1, 0]) / s
                y = 0.25 * s
                z = (rmat[1, 2] + rmat[2, 1]) / s
            else:
                s = math.sqrt(1.0 + rmat[2, 2] - rmat[0, 0] - rmat[1, 1]) * 2.0
                w = (rmat[1, 0] - rmat[0, 1]) / s
                x = (rmat[0, 2] + rmat[2, 0]) / s
                y = (rmat[1, 2] + rmat[2, 1]) / s
                z = 0.25 * s

        pose_msg.pose.orientation.w = w
        pose_msg.pose.orientation.x = x
        pose_msg.pose.orientation.y = y
        pose_msg.pose.orientation.z = z

        self.pose_pub.publish(pose_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    print("Fast ArUco+Depth detector started... (press 'q' to quit)")

    node = FastArucoWithDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()