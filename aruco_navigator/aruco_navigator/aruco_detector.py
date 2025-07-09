import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import cv2
import numpy as np
import time
import math

# 상수 정의
TARGET_WIDTH = 320
TARGET_HEIGHT = 240
MARKER_SIZE = 0.1
DEPTH_SCALE = 0.001
MIN_DEPTH = 0.3
MAX_DEPTH = 3.0
FRAME_SKIP = 2

class FastArucoWithDepth(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.frame_count = 0
        self.target_id = -1
        self.camera_info_received = False

        # TF2 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 이미지 버퍼 생성
        self.rgb_small = np.zeros((TARGET_HEIGHT, TARGET_WIDTH, 3), dtype=np.uint8)
        self.depth_small = np.zeros((TARGET_HEIGHT, TARGET_WIDTH), dtype=np.uint16)
        self.gray_buffer = np.zeros((TARGET_HEIGHT, TARGET_WIDTH), dtype=np.uint8)

        # 구독/발행
        self.rgb_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 1)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 1)
        self.target_id_sub = self.create_subscription(Int32, '/aruco/target_id', self.target_id_callback, 1)
        
        # 발행자
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/marker_pose', 1)
        self.odom_pose_pub = self.create_publisher(PoseStamped, '/aruco/marker_pose_odom', 1)
        self.visualization_pub = self.create_publisher(Image, '/aruco/visualization', 1)

        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except:
            self.detector = None
            self.use_new_api = False

        # 카메라 파라미터 기본값
        self.camera_fx = 154.25
        self.camera_fy = 154.25
        self.camera_cx = 160.0
        self.camera_cy = 120.0

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_header = None

        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.last_process_time = time.time()

        self.get_logger().info("Fast ArUco+Depth detector started (RViz2 + odom)")

        # 큐브 포인트 정의
        cube_height = MARKER_SIZE / 2.0
        self.cube_points = np.float32([
            [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
            [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
            [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
            [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
            [-MARKER_SIZE/2, -MARKER_SIZE/2, cube_height],
            [ MARKER_SIZE/2, -MARKER_SIZE/2, cube_height],
            [ MARKER_SIZE/2,  MARKER_SIZE/2, cube_height],
            [-MARKER_SIZE/2,  MARKER_SIZE/2, cube_height]
        ])

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            scale_x = TARGET_WIDTH / 640.0
            scale_y = TARGET_HEIGHT / 480.0
            self.camera_fx = msg.k[0] * scale_x
            self.camera_fy = msg.k[4] * scale_y
            self.camera_cx = msg.k[2] * scale_x
            self.camera_cy = msg.k[5] * scale_y
            self.camera_info_received = True
            self.destroy_subscription(self.camera_info_sub)
            self.get_logger().info(f"Camera info received: fx={self.camera_fx:.2f}, fy={self.camera_fy:.2f}")

    def target_id_callback(self, msg):
        self.target_id = msg.data
        self.get_logger().info(f"Target ID set to: {self.target_id}")

    def rgb_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb = cv2.resize(rgb_image, (TARGET_WIDTH, TARGET_HEIGHT), interpolation=cv2.INTER_LINEAR)
            self.latest_header = msg.header
        except Exception as e:
            self.get_logger().error(f"RGB error: {e}")

    def depth_callback(self, msg):
        if self.latest_rgb is None:
            return
        self.frame_count += 1
        if self.frame_count % FRAME_SKIP != 0:
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self.depth_small = cv2.resize(depth_image, (TARGET_WIDTH, TARGET_HEIGHT), interpolation=cv2.INTER_NEAREST)
            self.process_with_depth()
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def process_with_depth(self):
        try:
            cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2GRAY, dst=self.gray_buffer)
            if self.use_new_api and self.detector:
                corners, ids, _ = self.detector.detectMarkers(self.gray_buffer)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(self.gray_buffer, self.aruco_dict, parameters=self.aruco_params)
            
            display_image = self.latest_rgb.copy()
            markers_detected = 0
            
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                camera_matrix = np.array([[self.camera_fx, 0, self.camera_cx],
                                         [0, self.camera_fy, self.camera_cy],
                                         [0, 0, 1]], dtype=np.float32)
                dist_coeffs = np.zeros((4, 1))
                
                for i, marker_id in enumerate(ids.flatten()):
                    if self.target_id == -1 or marker_id == self.target_id:
                        marker_corners = corners[i][0]
                        center_x = int(np.mean(marker_corners[:, 0]))
                        center_y = int(np.mean(marker_corners[:, 1]))
                        depth_value = self.get_fast_depth(center_x, center_y)
                        
                        if MIN_DEPTH <= depth_value <= MAX_DEPTH:
                            markers_detected += 1
                            marker_points = np.float32([
                                [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                                [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                                [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                                [-MARKER_SIZE/2,  MARKER_SIZE/2, 0]
                            ])
                            
                            success, rvec, tvec = cv2.solvePnP(marker_points, marker_corners, camera_matrix, dist_coeffs)
                            
                            if success:
                                corrected_rvec, corrected_tvec = self.correct_marker_pose(rvec, tvec)
                                
                                # 큐브 그리기
                                cube_points_2d, _ = cv2.projectPoints(self.cube_points, corrected_rvec, corrected_tvec, camera_matrix, dist_coeffs)
                                cube_points_2d = np.int32(cube_points_2d).reshape(-1, 2)
                                
                                # 큐브 면 그리기
                                cv2.polylines(display_image, [cube_points_2d[0:4]], True, (0, 255, 0), 2)
                                cv2.polylines(display_image, [cube_points_2d[4:8]], True, (0, 255, 0), 2)
                                for j in range(4):
                                    cv2.line(display_image, tuple(cube_points_2d[j]), tuple(cube_points_2d[j+4]), (0, 255, 0), 2)
                                
                                # 축 그리기
                                display_image = self.draw_axes(display_image, corrected_rvec.flatten(), corrected_tvec.flatten(), camera_matrix, dist_coeffs)
                                
                                # 카메라 좌표계에서의 위치
                                camera_pos = corrected_tvec.flatten()
                                
                                # odom 좌표계로 변환
                                odom_pos = self.transform_to_odom(camera_pos, corrected_rvec)
                                
                                # 포즈 발행
                                self.publish_3d_pose(marker_id, camera_pos, corrected_rvec)
                                self.publish_3d_pose_odom(marker_id, odom_pos, corrected_rvec)
                                
                                # 중심점 표시
                                cv2.circle(display_image, (center_x, center_y), 4, (0, 0, 255), -1)
                                
                                # 정보 텍스트
                                info_text = f"ID{marker_id}: {depth_value:.2f}m"
                                cv2.putText(display_image, info_text, (center_x - 40, center_y - 15),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
                                # 카메라 좌표계 좌표 (노란색)
                                camera_coord_text = f"CAM:({camera_pos[0]:.2f},{camera_pos[1]:.2f},{camera_pos[2]:.2f})"
                                cv2.putText(display_image, camera_coord_text, (center_x - 70, center_y + 15),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
                                
                                # odom 좌표계 좌표 (청록색)
                                if odom_pos is not None:
                                    odom_coord_text = f"ODOM:({odom_pos[0]:.2f},{odom_pos[1]:.2f},{odom_pos[2]:.2f})"
                                    cv2.putText(display_image, odom_coord_text, (center_x - 70, center_y + 30),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
                                else:
                                    cv2.putText(display_image, "ODOM: Transform failed", (center_x - 70, center_y + 30),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
            
            # FPS 계산
            self.fps_counter += 1
            if time.time() - self.fps_start_time >= 1.0:
                fps = self.fps_counter / (time.time() - self.fps_start_time)
                self.fps_counter = 0
                self.fps_start_time = time.time()
                self.get_logger().info(f"FPS: {fps:.1f}, Target markers: {markers_detected}")
            
            # 상태 정보 오버레이
            status_text = f"Target ID: {self.target_id} | Markers: {markers_detected}"
            cv2.putText(display_image, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            coord_info = "Yellow: Camera | Cyan: Odom"
            cv2.putText(display_image, coord_info, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # RViz2를 위한 이미지 발행
            self.publish_visualization_image(display_image)
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    def transform_to_odom(self, camera_pos, rvec):
        """카메라 좌표계에서 odom 좌표계로 변환"""
        try:
            # 카메라 좌표계에서 PoseStamped 메시지 생성
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "camera_color_optical_frame"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = camera_pos[0]
            pose_msg.pose.position.y = camera_pos[1]
            pose_msg.pose.position.z = camera_pos[2]
            
            # 회전 정보 추가
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
            
            # odom 좌표계로 변환
            pose_in_odom = self.tf_buffer.transform(pose_msg, "odom", timeout=rclpy.duration.Duration(seconds=0.1))
            
            return np.array([
                pose_in_odom.pose.position.x,
                pose_in_odom.pose.position.y,
                pose_in_odom.pose.position.z
            ])
            
        except Exception as e:
            self.get_logger().warn(f"Transform to odom failed: {e}")
            return None

    def publish_visualization_image(self, image):
        """시각화 이미지를 RViz2에서 볼 수 있도록 발행"""
        try:
            # BGR을 RGB로 변환 (RViz2에서 올바른 색상 표시를 위해)
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Image 메시지로 변환
            img_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
            img_msg.header = self.latest_header
            img_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 발행
            self.visualization_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f"Visualization publish error: {e}")

    def get_fast_depth(self, x, y):
        h, w = self.depth_small.shape
        if x < 0 or x >= w or y < 0 or y >= h:
            return 0.0
        
        d = self.depth_small[y, x]
        if d > 0:
            return d * DEPTH_SCALE
        
        # 주변 픽셀 검색
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                d = self.depth_small[ny, nx]
                if d > 0:
                    return d * DEPTH_SCALE
        
        # 대각선 검색
        for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                d = self.depth_small[ny, nx]
                if d > 0:
                    return d * DEPTH_SCALE
        
        return 0.0

    def correct_marker_pose(self, rvec, tvec):
        rmat, _ = cv2.Rodrigues(rvec)
        flip_matrix = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ], dtype=np.float32)
        corrected_rmat = np.dot(rmat, flip_matrix)
        corrected_rvec, _ = cv2.Rodrigues(corrected_rmat)
        return corrected_rvec, tvec

    def draw_axes(self, image, rvec, tvec, camera_matrix, dist_coeffs, length=0.05):
        axis_points = np.float32([
            [0, 0, 0],
            [length, 0, 0],
            [0, length, 0],
            [0, 0, length]
        ]).reshape(-1, 3)
        
        imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        
        origin = tuple(imgpts[0].ravel())
        x_axis = tuple(imgpts[1].ravel())
        y_axis = tuple(imgpts[2].ravel())
        z_axis = tuple(imgpts[3].ravel())
        
        # 축 그리기 (X: 빨강, Y: 초록, Z: 파랑)
        cv2.line(image, origin, x_axis, (0, 0, 255), 3)
        cv2.line(image, origin, y_axis, (0, 255, 0), 3)
        cv2.line(image, origin, z_axis, (255, 0, 0), 3)
        
        # 축 레이블
        cv2.putText(image, "X", x_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(image, "Y", y_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(image, "Z", z_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return image

    def publish_3d_pose(self, marker_id, world_pos, rvec):
        """카메라 좌표계에서 마커 포즈 발행"""
        pose_msg = PoseStamped()
        pose_msg.header = self.latest_header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_color_optical_frame"
        
        pose_msg.pose.position.x = world_pos[0]
        pose_msg.pose.position.y = world_pos[1]
        pose_msg.pose.position.z = world_pos[2]
        
        # 회전 행렬을 쿼터니언으로 변환
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

    def publish_3d_pose_odom(self, marker_id, odom_pos, rvec):
        """odom 좌표계에서 마커 포즈 발행"""
        if odom_pos is None:
            return
            
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = odom_pos[0]
        pose_msg.pose.position.y = odom_pos[1]
        pose_msg.pose.position.z = odom_pos[2]
        
        # 회전 정보는 일단 기본값으로 설정 (필요시 변환 로직 추가)
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        
        self.odom_pose_pub.publish(pose_msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down ArUco detector...")
        super().destroy_node()

def main():
    rclpy.init()
    print("Fast ArUco+Depth detector started (RViz2 + odom)...")
    print("Add Image display in RViz2 with topic: /aruco/visualization")
    print("Coordinate transformation: camera_color_optical_frame -> odom")
    print("Yellow text: Camera coordinates | Cyan text: Odom coordinates")
    
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