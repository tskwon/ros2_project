import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Int32, Int32MultiArray
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import cv2
import numpy as np
import time
import math
from collections import deque
from geometry_msgs.msg import PoseArray, Pose

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

        # 마커별 이동평균 버퍼
        self.position_buffers = {}  # {marker_id: deque([...])}

        self.odom_pose_array_pub = self.create_publisher(PoseArray, '/aruco/marker_pose_array_odom', 1)

        # 파라미터 선언 및 가져오기
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('marker_size', 0.1)
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('ground_z_offset', 0.0)  # 지면 기준 Z 오프셋
        self.declare_parameter('use_ground_projection', True)  # 지면 투영 사용 여부
        self.declare_parameter('force_2d_plane', True)  # 강제 2D 평면 투영
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.marker_size = self.get_parameter('marker_size').value
        self.target_distance = self.get_parameter('target_distance').value
        self.ground_z_offset = self.get_parameter('ground_z_offset').value
        self.use_ground_projection = self.get_parameter('use_ground_projection').value
        self.force_2d_plane = self.get_parameter('force_2d_plane').value
        
        self.get_logger().info(f"ArUco Detector initialized:")
        self.get_logger().info(f"  - Camera frame: {self.camera_frame}")
        self.get_logger().info(f"  - Reference frame: {self.reference_frame}")
        self.get_logger().info(f"  - Marker size: {self.marker_size}")
        self.get_logger().info(f"  - Ground Z offset: {self.ground_z_offset}")
        self.get_logger().info(f"  - Use ground projection: {self.use_ground_projection}")
        self.get_logger().info(f"  - Force 2D plane: {self.force_2d_plane}")

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
        self.detected_ids_pub = self.create_publisher(Int32MultiArray, '/aruco/detected_marker_ids', 1)

        # 상태 확인을 위한 타이머 (디버깅용)
        self.status_timer = self.create_timer(2.0, self.status_callback)

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

        self.get_logger().info("Fast ArUco+Depth detector started (RViz2 + odom + 2D plane)")

    def status_callback(self):
        if self.latest_rgb is not None:
            self.get_logger().info(f"Camera active. Target ID: {self.target_id}")
            self.get_logger().info(f"Visualization subscribers: {self.visualization_pub.get_subscription_count()}")
        else:
            self.get_logger().warn("No camera data received")

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
            small_rgb = cv2.resize(rgb_image, (TARGET_WIDTH, TARGET_HEIGHT), interpolation=cv2.INTER_LINEAR)
            cv2.cvtColor(small_rgb, cv2.COLOR_BGR2GRAY, dst=self.gray_buffer)
            self.latest_rgb = small_rgb
            self.latest_header = msg.header
            if not hasattr(self, 'first_image_received'):
                self.get_logger().info("First RGB image received!")
                self.first_image_received = True
        except Exception as e:
            self.get_logger().error(f"RGB error: {e}")

    def depth_callback(self, msg):
        if self.latest_rgb is None:
            self.get_logger().warn("Depth received but no RGB image yet")
            return
        self.frame_count += 1
        if self.frame_count % FRAME_SKIP != 0:
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            cv2.resize(depth_image, (TARGET_WIDTH, TARGET_HEIGHT), dst=self.depth_small, interpolation=cv2.INTER_NEAREST)
            if not hasattr(self, 'first_depth_processed'):
                self.get_logger().info("First depth image processed!")
                self.first_depth_processed = True
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

            if ids is None or len(ids) == 0 or corners is None or len(corners) == 0:
                self.publish_visualization_image(display_image)
                return

            cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
            camera_matrix = np.array([
                [self.camera_fx, 0, self.camera_cx],
                [0, self.camera_fy, self.camera_cy],
                [0, 0, 1]
            ], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1))

            pose_array = []
            odom_pose_array = PoseArray()
            odom_pose_array.header.stamp = self.latest_header.stamp
            odom_pose_array.header.frame_id = self.reference_frame

            for marker_id, marker_corners in zip(ids.flatten(), corners):
                marker_corners = marker_corners[0]
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))
                depth_value = self.get_fast_depth(center_x, center_y)

                if not (MIN_DEPTH <= depth_value <= MAX_DEPTH):
                    continue

                markers_detected += 1
                marker_points = np.float32([
                    [-self.marker_size/2, -self.marker_size/2, 0],
                    [ self.marker_size/2, -self.marker_size/2, 0],
                    [ self.marker_size/2,  self.marker_size/2, 0],
                    [-self.marker_size/2,  self.marker_size/2, 0]
                ])

                success, rvec, tvec = cv2.solvePnP(marker_points, marker_corners, camera_matrix, dist_coeffs)
                if not success:
                    continue

                corrected_rvec, corrected_tvec = self.correct_marker_pose(rvec, tvec)
                display_image = self.draw_axes(display_image, corrected_rvec.flatten(), corrected_tvec.flatten(), camera_matrix, dist_coeffs)
                camera_pos = corrected_tvec.flatten()

                if marker_id not in self.position_buffers:
                    self.position_buffers[marker_id] = deque(maxlen=5)
                self.position_buffers[marker_id].append(camera_pos)
                avg_pos = np.mean(self.position_buffers[marker_id], axis=0)

                # 수정된 odom 변환 호출
                odom_pose = self.transform_to_odom(avg_pos, corrected_rvec)
                
                # 카메라 좌표계에서의 포즈 발행
                self.publish_3d_pose(marker_id, avg_pos, corrected_rvec)
                
                # odom 좌표계에서의 포즈 발행
                if odom_pose is not None:
                    self.publish_3d_pose_odom(marker_id, odom_pose)
                    
                    # PoseArray에 추가
                    pose = Pose()
                    pose.position.x = odom_pose.pose.position.x
                    pose.position.y = odom_pose.pose.position.y
                    pose.position.z = odom_pose.pose.position.z
                    pose.orientation = odom_pose.pose.orientation
                    odom_pose_array.poses.append(pose)

                pose_array.append((marker_id, avg_pos, corrected_rvec))

            # odom_pose_array 발행
            if len(odom_pose_array.poses) > 0:
                self.odom_pose_array_pub.publish(odom_pose_array)
                # 인식된 마커 ID들을 pose 순서와 동일하게 발행
                detected_ids_msg = Int32MultiArray()
                detected_ids_msg.data = [int(marker_id) for marker_id, _, _ in pose_array]
                self.detected_ids_pub.publish(detected_ids_msg)
                
            self.fps_counter += 1
            if time.time() - self.fps_start_time >= 1.0:
                fps = self.fps_counter / (time.time() - self.fps_start_time)
                self.fps_counter = 0
                self.fps_start_time = time.time()
                self.get_logger().info(f"FPS: {fps:.1f}, Target markers: {markers_detected}")

            self.publish_visualization_image(display_image)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    def transform_to_odom(self, camera_pos, rvec):
        """
        카메라 좌표계에서 감지된 마커의 포즈를 odom 좌표계로 변환하고 2D 평면에 투영
        """
        try:
            # 1단계: 카메라 좌표계에서의 마커 포즈 생성
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.camera_frame
            pose_msg.header.stamp = self.latest_header.stamp
            
            # 위치 설정
            pose_msg.pose.position.x = float(camera_pos[0])
            pose_msg.pose.position.y = float(camera_pos[1])
            pose_msg.pose.position.z = float(camera_pos[2])

            # 회전 행렬을 쿼터니언으로 변환
            rmat, _ = cv2.Rodrigues(rvec)
            quat = self.rotation_matrix_to_quaternion(rmat)
            
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            timeout = rclpy.duration.Duration(seconds=0.1)
            
            # 2단계: camera_frame -> base_link 변환
            pose_in_base_link = self.tf_buffer.transform(pose_msg, "base_link", timeout=timeout)
            
            # 3단계: base_link -> odom 변환
            pose_in_odom = self.tf_buffer.transform(pose_in_base_link, self.reference_frame, timeout=timeout)
            
            # 4단계: 2D 평면으로 투영
            if self.force_2d_plane:
                # 강제로 2D 평면에 투영
                original_z = pose_in_odom.pose.position.z
                pose_in_odom.pose.position.z = self.ground_z_offset
                
                # 회전도 2D 평면에 맞게 수정 (Z축 회전만 유지)
                pose_in_odom = self.project_rotation_to_2d_plane(pose_in_odom)
                
                self.get_logger().debug(f"2D projection: original_z={original_z:.3f}, projected_z={self.ground_z_offset:.3f}")
            
            elif self.use_ground_projection:
                # 기존 지면 투영 방식
                pose_in_odom.pose.position.z = self.ground_z_offset
                self.get_logger().debug(f"Ground projection: z = {self.ground_z_offset}")
            
            return pose_in_odom
            
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            self.get_logger().warn("Make sure TF chain exists: camera_frame -> base_link -> odom")
            return None

    def project_rotation_to_2d_plane(self, pose_stamped):
        """
        3D 회전을 2D 평면의 Z축 회전으로 투영
        """
        # 현재 쿼터니언 추출
        q = pose_stamped.pose.orientation
        
        # 쿼터니언을 오일러 각으로 변환
        # Roll (x축), Pitch (y축), Yaw (z축)
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        # 2D 평면에서는 Z축 회전(yaw)만 유지
        # Roll과 Pitch는 0으로 설정
        new_roll = 0.0
        new_pitch = 0.0
        new_yaw = yaw
        
        # 새로운 쿼터니언 생성
        new_quat = self.euler_to_quaternion(new_roll, new_pitch, new_yaw)
        
        # 포즈 업데이트
        pose_stamped.pose.orientation.x = new_quat[0]
        pose_stamped.pose.orientation.y = new_quat[1]
        pose_stamped.pose.orientation.z = new_quat[2]
        pose_stamped.pose.orientation.w = new_quat[3]
        
        return pose_stamped

    def quaternion_to_euler(self, x, y, z, w):
        """
        쿼터니언을 오일러 각으로 변환
        """
        # Roll (x축 회전)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y축 회전)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z축 회전)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        오일러 각을 쿼터니언으로 변환
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]

    def project_to_ground_plane(self, pose_in_odom):
        """
        마커 위치를 지면 평면으로 투영하는 고급 방법
        """
        # 카메라에서 마커까지의 벡터 계산
        marker_x = pose_in_odom.pose.position.x
        marker_y = pose_in_odom.pose.position.y
        marker_z = pose_in_odom.pose.position.z
        
        # 로봇 base_link 위치 (일반적으로 odom 원점 기준)
        robot_x = 0.0
        robot_y = 0.0
        robot_z = 0.0
        
        # 마커가 지면에 있다고 가정하고 XY 평면으로 투영
        # 거리 비율을 유지하면서 Z=0 평면으로 투영
        if abs(marker_z - robot_z) > 0.01:  # 높이 차이가 있는 경우
            # 비례 계산으로 지면 위치 추정
            ground_x = marker_x
            ground_y = marker_y
            ground_z = self.ground_z_offset
        else:
            ground_x = marker_x
            ground_y = marker_y  
            ground_z = marker_z
            
        return ground_x, ground_y, ground_z

    def rotation_matrix_to_quaternion(self, rmat):
        """
        회전 행렬을 쿼터니언으로 변환 (개선된 버전)
        """
        trace = np.trace(rmat)
        
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (rmat[2, 1] - rmat[1, 2]) / s
            y = (rmat[0, 2] - rmat[2, 0]) / s
            z = (rmat[1, 0] - rmat[0, 1]) / s
        elif rmat[0, 0] > rmat[1, 1] and rmat[0, 0] > rmat[2, 2]:
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
            
        return [x, y, z, w]

    def publish_visualization_image(self, image):
        try:
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            img_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
            img_msg.header = self.latest_header
            img_msg.header.stamp = self.get_clock().now().to_msg()
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
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                d = self.depth_small[ny, nx]
                if d > 0:
                    return d * DEPTH_SCALE
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
        cv2.line(image, origin, x_axis, (0, 0, 255), 3)
        cv2.line(image, origin, y_axis, (0, 255, 0), 3)
        cv2.line(image, origin, z_axis, (255, 0, 0), 3)
        cv2.putText(image, "X", x_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(image, "Y", y_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(image, "Z", z_axis, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return image

    def publish_3d_pose(self, marker_id, world_pos, rvec):
        pose_msg = PoseStamped()
        pose_msg.header = self.latest_header
        pose_msg.header.stamp = self.latest_header.stamp
        pose_msg.header.frame_id = self.camera_frame  # 올바른 프레임 ID 사용
        pose_msg.pose.position.x = world_pos[0]
        pose_msg.pose.position.y = world_pos[1]
        pose_msg.pose.position.z = world_pos[2]
        
        # 회전 행렬을 쿼터니언으로 변환
        rmat, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(rmat)
        
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)

    def publish_3d_pose_odom(self, marker_id, odom_pose):
        """
        odom 좌표계에서의 마커 포즈를 발행
        """
        if odom_pose is None:
            return
            
        # 이미 변환된 PoseStamped 메시지를 그대로 발행
        self.odom_pose_pub.publish(odom_pose)

    def destroy_node(self):
        self.get_logger().info("Shutting down ArUco detector...")
        super().destroy_node()

def main():
    rclpy.init()
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