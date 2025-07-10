import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Int32MultiArray
import math

MAX_ANGULAR_VEL = 0.5
MIN_ANGULAR_VEL = 0.3
ANGLE_TOLERANCE = 0.1

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MoveToArucoTarget(Node):
    def __init__(self):
        super().__init__('move_to_aruco_target')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.target_id_sub = self.create_subscription(Int32, '/aruco/target_id', self.target_id_callback, 1)
        self.marker_pose_array_sub = self.create_subscription(PoseArray, '/aruco/marker_pose_array_odom', self.marker_pose_array_callback, 10)
        self.marker_ids_sub = self.create_subscription(Int32MultiArray, '/aruco/detected_marker_ids', self.marker_ids_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # State variables
        self.target_id = -1
        self.marker_poses = []  # PoseArray의 poses
        self.marker_ids = []    # 해당 poses와 순서가 일치하는 ID들
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.state = 'IDLE'  # IDLE, TURN_TO_TARGET

        # 디버깅용 변수들
        self.last_marker_time = None
        self.marker_update_count = 0
        self.control_loop_count = 0
        self.angle_diff_history = []

        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        self.get_logger().info("ArUco 배열 기반 회전 제어 노드 시작")

    def target_id_callback(self, msg):
        self.target_id = msg.data
        self.get_logger().info(f"[타겟 ID 수신] {self.target_id}")

    def marker_pose_array_callback(self, msg):
        self.marker_poses = msg.poses
        self.marker_update_count += 1
        self.last_marker_time = self.get_clock().now()
        
        self.get_logger().info(f"[마커 Pose 배열 수신 #{self.marker_update_count}] {len(self.marker_poses)}개 마커")
        
        # 배열 내용 디버깅
        for i, pose in enumerate(self.marker_poses):
            self.get_logger().info(
                f"  Pose[{i}]: x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
                f"heading={quaternion_to_yaw(pose.orientation):.3f}"
            )

    def marker_ids_callback(self, msg):
        self.marker_ids = msg.data
        self.get_logger().info(f"[마커 ID 배열 수신] {self.marker_ids}")
        
        # target_id가 배열에 있는지 확인
        if self.target_id in self.marker_ids and self.state == 'IDLE':
            self.state = 'TURN_TO_TARGET'
            self.get_logger().info(f"[상태 전환] TURN_TO_TARGET (타겟 ID {self.target_id} 발견)")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def get_target_pose(self):
        """target_id에 해당하는 pose를 배열에서 찾아 반환"""
        try:
            if self.target_id == -1:
                return None
            
            if self.target_id not in self.marker_ids:
                return None
            
            # target_id의 인덱스를 찾아서 해당하는 pose 반환
            target_index = self.marker_ids.index(self.target_id)
            
            if target_index >= len(self.marker_poses):
                self.get_logger().warn(f"[오류] ID 배열과 Pose 배열 크기 불일치: ID={len(self.marker_ids)}, Pose={len(self.marker_poses)}")
                return None
            
            return self.marker_poses[target_index]
            
        except Exception as e:
            self.get_logger().error(f"[오류] get_target_pose: {e}")
            return None

    def control_loop(self):
        self.control_loop_count += 1
        
        # 마커 데이터 유효성 검사
        if len(self.marker_poses) == 0 or len(self.marker_ids) == 0:
            if self.control_loop_count % 50 == 0:  # 5초마다 출력
                self.get_logger().warn("[디버그] 마커 배열 데이터 없음")
            return
            
        # 마커 데이터 시간 검사
        current_time = self.get_clock().now()
        if self.last_marker_time is not None:
            time_diff = (current_time - self.last_marker_time).nanoseconds / 1e9
            if time_diff > 0.5:  # 0.5초 이상 오래된 데이터
                self.get_logger().warn(f"[디버그] 마커 데이터가 오래됨: {time_diff:.2f}초")
                return

        if self.state == 'TURN_TO_TARGET':
            self.turn_to_target_debug()
        else:
            if self.control_loop_count % 100 == 0:  # 10초마다 출력
                self.get_logger().info(f"[디버그] 상태: {self.state}, 타겟ID: {self.target_id}")

    def turn_to_target_debug(self):
        target_pose = self.get_target_pose()
        
        if target_pose is None:
            self.get_logger().warn(f"[경고] 타겟 ID {self.target_id}에 해당하는 pose를 찾을 수 없음")
            self.state = 'IDLE'
            return
        
        target_x = target_pose.position.x
        target_y = target_pose.position.y
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.robot_yaw)

        # 각도 변화 기록
        self.angle_diff_history.append(angle_diff)
        if len(self.angle_diff_history) > 20:
            self.angle_diff_history.pop(0)

        # 각도 변화 분석
        angle_diff_change = 0
        if len(self.angle_diff_history) >= 2:
            angle_diff_change = self.angle_diff_history[-1] - self.angle_diff_history[-2]

        # 상세 디버깅 정보
        target_index = self.marker_ids.index(self.target_id)
        self.get_logger().info(
            f"[회전 디버깅] "
            f"타겟ID={self.target_id}(idx:{target_index}), "
            f"로봇위치=({self.robot_x:.3f},{self.robot_y:.3f}), "
            f"마커위치=({target_x:.3f},{target_y:.3f}), "
            f"거리={distance:.3f}, "
            f"목표각={math.degrees(angle_to_target):.1f}°, "
            f"현재각={math.degrees(self.robot_yaw):.1f}°, "
            f"각도차={math.degrees(angle_diff):.1f}°, "
            f"각도변화={math.degrees(angle_diff_change):.1f}°"
        )

        # 문제점 분석
        if distance < 0.1:
            self.get_logger().warn("[문제] 마커가 너무 가까움 - 부정확한 각도 계산 가능")
        
        if abs(angle_diff) > math.pi * 0.9:
            self.get_logger().warn("[문제] 각도차가 너무 큼 - 180도 근처에서 진동 가능")
        
        if len(self.angle_diff_history) >= 5:
            recent_changes = [abs(self.angle_diff_history[i] - self.angle_diff_history[i-1]) 
                            for i in range(1, len(self.angle_diff_history))]
            avg_change = sum(recent_changes) / len(recent_changes)
            if avg_change > 0.1:
                self.get_logger().warn(f"[문제] 각도가 불안정함 - 평균변화: {math.degrees(avg_change):.1f}°")

        # 회전 명령 계산 및 발행
        if abs(angle_diff) > ANGLE_TOLERANCE:
            angular = clamp(0.8 * angle_diff, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
            if abs(angular) < MIN_ANGULAR_VEL:
                angular = MIN_ANGULAR_VEL * (1 if angular > 0 else -1)
            
            self.get_logger().info(f"[회전 명령] angular.z={angular:.3f}")
            
            # 실제 모터 명령 발행 (주석 해제하여 사용)
            twist = Twist()
            twist.angular.z = angular
            self.cmd_pub.publish(twist)  # 주석 해제하면 실제 회전
            
            # 로봇이 계속 회전하는 문제 체크
            if abs(angle_diff_change) < 0.01 and len(self.angle_diff_history) >= 10:
                self.get_logger().warn("[문제] 각도차가 줄어들지 않음 - 마커 위치가 계속 업데이트되는지 확인")
        else:
            self.get_logger().info("[완료] 타겟 방향 정렬 완료")
            self.stop_robot()
            self.state = 'IDLE'
            self.angle_diff_history.clear()

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)  # 주석 해제하면 실제 정지
        self.get_logger().info("[로봇 정지] 명령 전송")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main():
    rclpy.init()
    node = MoveToArucoTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C 종료")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("=== Node shutdown complete ===")

if __name__ == '__main__':
    main()