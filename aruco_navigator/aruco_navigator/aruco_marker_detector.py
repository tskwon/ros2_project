#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # 카메라 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # 카메라 토픽 이름
            self.image_callback,
            1)
        # 포즈 메시지 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/marker_pose', 2)
        self.bridge = CvBridge()
        # ArUco 사전 및 검출기 파라미터 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        # 카메라 내부 파라미터 (camera_info에서 추출)
        self.camera_matrix = np.array([[604.9893798828125, 0.0, 319.0706787109375],
                                     [0.0, 604.4199829101562, 252.50985717773438],
                                     [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        # 마커 크기 (미터 단위, 예: 5cm)
        self.marker_size = 0.1
        self.frame_count = 0 
        self.frame_skip = 2
        self.get_logger().info('ArUco Detector Node Started')

    def image_callback(self, msg):
        self.frame_count = (self.frame_count + 1) % 1000000
        if self.frame_count % self.frame_skip != 0:
            return  # 프레임 스킵

        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # ArUco 마커 검출
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                for i, corner in enumerate(corners):
                    half_size = self.marker_size / 2
                    object_points = np.array([
                        [-half_size, -half_size, 0],
                        [half_size, -half_size, 0],
                        [half_size, half_size, 0],
                        [-half_size, half_size, 0]
                    ], dtype=np.float32)

                    image_points = corner[0].astype(np.float32)
                    success, rvec, tvec = cv2.solvePnP(
                        object_points, 
                        image_points, 
                        self.camera_matrix, 
                        self.dist_coeffs
                    )

                    if success:
                        center = np.mean(corner[0], axis=0).astype(int)
                        cv2.putText(cv_image, f'ID: {ids[i][0]}',
                                    (center[0], center[1]),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs,
                                        rvec, tvec, self.marker_size * 0.5)

                        tvec_flat = tvec.flatten()
                        distance = np.linalg.norm(tvec_flat)
                        self.get_logger().info(
                            f'Marker ID {ids[i][0]}: x={tvec_flat[0]:.3f}m, y={tvec_flat[1]:.3f}m, '
                            f'z={tvec_flat[2]:.3f}m, distance={distance:.3f}m')

                        pose_msg = PoseStamped()
                        pose_msg.header = msg.header
                        pose_msg.header.frame_id = f'marker_{ids[i][0]}'
                        pose_msg.pose.position.x = float(tvec_flat[0])
                        pose_msg.pose.position.y = float(tvec_flat[1])
                        pose_msg.pose.position.z = float(tvec_flat[2])

                        rot_matrix, _ = cv2.Rodrigues(rvec)
                        quaternion = self.rotation_matrix_to_quaternion(rot_matrix)
                        pose_msg.pose.orientation.x = quaternion[0]
                        pose_msg.pose.orientation.y = quaternion[1]
                        pose_msg.pose.orientation.z = quaternion[2]
                        pose_msg.pose.orientation.w = quaternion[3]

                        self.pose_pub.publish(pose_msg)

            # cv2.imshow('ArUco Markers', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


    def rotation_matrix_to_quaternion(self, R):
        """회전 행렬을 쿼터니언으로 변환"""
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2.0
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
        return np.array([x, y, z, w])

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()