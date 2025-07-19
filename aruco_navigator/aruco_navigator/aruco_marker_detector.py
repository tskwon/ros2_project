#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # 카메라 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/d435/realsense_d435/color/image_raw',  # 카메라 토픽 이름
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
        self.frame_skip = 1
        
        # 이동평균 필터 설정
        self.filter_window_size = 5  # 이동평균 윈도우 크기
        self.marker_filters = {}  # 마커 ID별로 필터 관리
        
        self.get_logger().info('ArUco Detector Node Started')
        self.get_logger().info(f'Moving average filter enabled (window size: {self.filter_window_size})')

    def init_marker_filter(self, marker_id):
        """마커별 필터 초기화"""
        self.marker_filters[marker_id] = {
            'position_buffer': {'x': [], 'y': [], 'z': []},
            'orientation_buffer': {'qx': [], 'qy': [], 'qz': [], 'qw': []},
            'filtered_position': {'x': None, 'y': None, 'z': None},
            'filtered_orientation': {'qx': None, 'qy': None, 'qz': None, 'qw': None}
        }
    
    def apply_moving_average_filter(self, marker_id, tvec, quaternion):
        """이동평균 필터 적용"""
        if marker_id not in self.marker_filters:
            self.init_marker_filter(marker_id)
        
        filter_data = self.marker_filters[marker_id]
        
        # 새로운 값 추가
        filter_data['position_buffer']['x'].append(tvec[0])
        filter_data['position_buffer']['y'].append(tvec[1])
        filter_data['position_buffer']['z'].append(tvec[2])
        
        filter_data['orientation_buffer']['qx'].append(quaternion[0])
        filter_data['orientation_buffer']['qy'].append(quaternion[1])
        filter_data['orientation_buffer']['qz'].append(quaternion[2])
        filter_data['orientation_buffer']['qw'].append(quaternion[3])
        
        # 윈도우 크기 초과 시 오래된 값 제거
        for key in filter_data['position_buffer']:
            if len(filter_data['position_buffer'][key]) > self.filter_window_size:
                filter_data['position_buffer'][key].pop(0)
        
        for key in filter_data['orientation_buffer']:
            if len(filter_data['orientation_buffer'][key]) > self.filter_window_size:
                filter_data['orientation_buffer'][key].pop(0)
        
        # 이동평균 계산
        filter_data['filtered_position']['x'] = sum(filter_data['position_buffer']['x']) / len(filter_data['position_buffer']['x'])
        filter_data['filtered_position']['y'] = sum(filter_data['position_buffer']['y']) / len(filter_data['position_buffer']['y'])
        filter_data['filtered_position']['z'] = sum(filter_data['position_buffer']['z']) / len(filter_data['position_buffer']['z'])
        
        filter_data['filtered_orientation']['qx'] = sum(filter_data['orientation_buffer']['qx']) / len(filter_data['orientation_buffer']['qx'])
        filter_data['filtered_orientation']['qy'] = sum(filter_data['orientation_buffer']['qy']) / len(filter_data['orientation_buffer']['qy'])
        filter_data['filtered_orientation']['qz'] = sum(filter_data['orientation_buffer']['qz']) / len(filter_data['orientation_buffer']['qz'])
        filter_data['filtered_orientation']['qw'] = sum(filter_data['orientation_buffer']['qw']) / len(filter_data['orientation_buffer']['qw'])
        
        # 쿼터니언 정규화
        qx = filter_data['filtered_orientation']['qx']
        qy = filter_data['filtered_orientation']['qy']
        qz = filter_data['filtered_orientation']['qz']
        qw = filter_data['filtered_orientation']['qw']
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        
        if norm > 0:
            filter_data['filtered_orientation']['qx'] /= norm
            filter_data['filtered_orientation']['qy'] /= norm
            filter_data['filtered_orientation']['qz'] /= norm
            filter_data['filtered_orientation']['qw'] /= norm
        
        # 필터링된 값 반환
        filtered_tvec = np.array([
            filter_data['filtered_position']['x'],
            filter_data['filtered_position']['y'],
            filter_data['filtered_position']['z']
        ])
        
        filtered_quaternion = np.array([
            filter_data['filtered_orientation']['qx'],
            filter_data['filtered_orientation']['qy'],
            filter_data['filtered_orientation']['qz'],
            filter_data['filtered_orientation']['qw']
        ])
        
        return filtered_tvec, filtered_quaternion
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """쿼터니언을 오일러 각도로 변환"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])

    def rotation_vector_to_euler(self, rvec):
        """회전 벡터를 오일러 각도(roll, pitch, yaw)로 변환"""
        # 회전 벡터를 회전 행렬로 변환
        rot_matrix, _ = cv2.Rodrigues(rvec)
        
        # 회전 행렬을 오일러 각도로 변환
        # ZYX 순서 (yaw, pitch, roll)
        sy = math.sqrt(rot_matrix[0, 0] * rot_matrix[0, 0] + rot_matrix[1, 0] * rot_matrix[1, 0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(rot_matrix[2, 1], rot_matrix[2, 2])  # roll
            y = math.atan2(-rot_matrix[2, 0], sy)               # pitch
            z = math.atan2(rot_matrix[1, 0], rot_matrix[0, 0])  # yaw
        else:
            x = math.atan2(-rot_matrix[1, 2], rot_matrix[1, 1])  # roll
            y = math.atan2(-rot_matrix[2, 0], sy)                # pitch
            z = 0                                                # yaw
        
        return np.array([x, y, z])  # [roll, pitch, yaw]

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

                        # 원본 위치 정보
                        tvec_flat = tvec.flatten()
                        distance = np.linalg.norm(tvec_flat)
                        
                        # 원본 각도 정보 계산
                        euler_angles = self.rotation_vector_to_euler(rvec)
                        roll = euler_angles[0]
                        pitch = euler_angles[1]
                        yaw = euler_angles[2]
                        
                        # 쿼터니언 계산
                        rot_matrix, _ = cv2.Rodrigues(rvec)
                        quaternion = self.rotation_matrix_to_quaternion(rot_matrix)
                        
                        # 이동평균 필터 적용
                        filtered_tvec, filtered_quaternion = self.apply_moving_average_filter(
                            ids[i][0], tvec_flat, quaternion)
                        
                        # 필터링된 값들로 다시 계산
                        filtered_distance = np.linalg.norm(filtered_tvec)
                        filtered_euler = self.quaternion_to_euler(
                            filtered_quaternion[0], filtered_quaternion[1], 
                            filtered_quaternion[2], filtered_quaternion[3])
                        filtered_roll = filtered_euler[0]
                        filtered_pitch = filtered_euler[1]
                        filtered_yaw = filtered_euler[2]
                        
                        # 필터링 효과 확인을 위한 노이즈 계산
                        if ids[i][0] in self.marker_filters:
                            filter_data = self.marker_filters[ids[i][0]]
                            if len(filter_data['position_buffer']['z']) >= self.filter_window_size:
                                distance_std = np.std(filter_data['position_buffer']['z'])
                                if distance_std > 0.005:  # 5mm 이상의 노이즈
                                    self.get_logger().debug(f'Marker {ids[i][0]} distance noise std: {distance_std:.4f}m')
                        
                        # 로그 출력 (필터링된 값 + 원본 값 비교)
                        self.get_logger().info(
                            f'Marker ID {ids[i][0]} [FILTERED]: '
                            f'x={filtered_tvec[0]:.3f}m, y={filtered_tvec[1]:.3f}m, z={filtered_tvec[2]:.3f}m, '
                            f'distance={filtered_distance:.3f}m | '
                            f'roll={math.degrees(filtered_roll):.1f}°, pitch={math.degrees(filtered_pitch):.1f}°, yaw={math.degrees(filtered_yaw):.1f}° | '
                            f'roll={filtered_roll:.3f}rad, pitch={filtered_pitch:.3f}rad, yaw={filtered_yaw:.3f}rad')

                        # 포즈 메시지 생성 (필터링된 값 사용)
                        pose_msg = PoseStamped()
                        pose_msg.header = msg.header
                        pose_msg.header.frame_id = f'marker_{ids[i][0]}'
                        pose_msg.pose.position.x = float(filtered_tvec[0])
                        pose_msg.pose.position.y = float(filtered_tvec[1])
                        pose_msg.pose.position.z = float(filtered_tvec[2])

                        # 필터링된 쿼터니언 설정
                        pose_msg.pose.orientation.x = filtered_quaternion[0]
                        pose_msg.pose.orientation.y = filtered_quaternion[1]
                        pose_msg.pose.orientation.z = filtered_quaternion[2]
                        pose_msg.pose.orientation.w = filtered_quaternion[3]

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