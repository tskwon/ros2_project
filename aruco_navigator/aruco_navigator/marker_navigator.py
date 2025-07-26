import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Int32MultiArray
import math

MAX_ANGULAR_VEL = 0.5
MIN_ANGULAR_VEL = 0.3
MAX_LINEAR_VEL = 0.2
MIN_LINEAR_VEL = 0.05
ANGLE_TOLERANCE = math.radians(5)  # 5도로 설정
GOAL_TOLERANCE = 0.2

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
        self.saved_target_poses = {}  # 마커별 저장된 위치 {marker_id: pose}
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.state = 'IDLE'  # IDLE, TURN_TO_TARGET, MOVE_TO_TARGET, ALIGN_HEADING

        # 디버깅용 변수들
        self.last_marker_time = None
        self.marker_update_count = 0
        self.control_loop_count = 0
        self.angle_diff_history = []

        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        self.get_logger().info("ArUco 배열 기반 회전+직진 제어 노드 시작 (5도 정밀도)")

    def target_id_callback(self, msg):
        new_target_id = msg.data
        
        # 현재 작업 중이면 새로운 타겟 ID 무시
        if self.state != 'IDLE':
            self.get_logger().warn(f"[타겟 ID 무시] 현재 상태: {self.state}, 새 ID: {new_target_id} 무시됨")
            return
        
        # IDLE 상태일 때만 새로운 타겟 ID 수용
        self.target_id = new_target_id
        self.get_logger().info(f"[타겟 ID 수신] {self.target_id}")
        self.angle_diff_history.clear()

    def marker_pose_array_callback(self, msg):
        self.marker_poses = msg.poses
        self.marker_update_count += 1
        self.last_marker_time = self.get_clock().now()
        
        self.get_logger().info(f"[마커 Pose 배열 수신 #{self.marker_update_count}] {len(self.marker_poses)}개 마커")

    def marker_ids_callback(self, msg):
        self.marker_ids = msg.data
        self.get_logger().info(f"[마커 ID 배열 수신] {self.marker_ids}")
        
        # 현재 인식된 마커들의 위치를 저장
        for i, marker_id in enumerate(self.marker_ids):
            if i < len(self.marker_poses):
                self.saved_target_poses[marker_id] = self.marker_poses[i]
                self.get_logger().info(f"[마커 저장] ID {marker_id} 위치 저장")
        
        # target_id가 있고 IDLE 상태에서만 시작
        if self.target_id != -1 and self.state == 'IDLE':
            if (self.target_id in self.marker_ids or self.target_id in self.saved_target_poses):
                self.state = 'TURN_TO_TARGET'
                self.get_logger().info(f"[상태 전환] TURN_TO_TARGET (타겟 ID {self.target_id} 발견)")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def get_target_pose(self):
        """target_id에 해당하는 pose를 반환"""
        if self.target_id == -1:
            return None
        
        # 1순위: 실시간 인식된 마커
        if self.target_id in self.marker_ids:
            target_index = self.marker_ids.index(self.target_id)
            if target_index < len(self.marker_poses):
                return self.marker_poses[target_index]
        
        # 2순위: 저장된 위치
        if self.target_id in self.saved_target_poses:
            return self.saved_target_poses[self.target_id]
        
        return None

    def control_loop(self):
        self.control_loop_count += 1
        
        # 진행 가능 여부 확인
        can_proceed = False
        if self.target_id != -1:
            if len(self.marker_poses) > 0 or self.target_id in self.saved_target_poses:
                can_proceed = True
        
        if not can_proceed:
            if self.control_loop_count % 50 == 0:
                self.get_logger().warn("[디버그] 마커 데이터 없음")
            return

        if self.state == 'TURN_TO_TARGET':
            self.turn_to_target_debug()
        elif self.state == 'MOVE_TO_TARGET':
            self.move_to_target_debug()
        elif self.state == 'ALIGN_HEADING':
            self.align_heading_debug()

    def turn_to_target_debug(self):
        target_pose = self.get_target_pose()
        if target_pose is None:
            self.get_logger().warn(f"[경고] 타겟 ID {self.target_id} pose 없음")
            self.state = 'IDLE'
            return
        
        target_x = target_pose.position.x
        target_y = target_pose.position.y
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.robot_yaw)

        self.get_logger().info(f"[회전] 각도차={math.degrees(angle_diff):.1f}°")

        if abs(angle_diff) > ANGLE_TOLERANCE:
            angular = clamp(0.8 * angle_diff, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
            if abs(angular) < MIN_ANGULAR_VEL:
                angular = MIN_ANGULAR_VEL * (1 if angular > 0 else -1)
            
            twist = Twist()
            twist.angular.z = angular
            self.cmd_pub.publish(twist)
        else:
            self.get_logger().info("[완료] 회전 완료")
            self.state = 'MOVE_TO_TARGET'

    def move_to_target_debug(self):
        target_pose = self.get_target_pose()
        if target_pose is None:
            self.state = 'IDLE'
            return
        
        target_x = target_pose.position.x
        target_y = target_pose.position.y
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.robot_yaw)

        self.get_logger().info(f"[직진] 거리={distance:.3f}m, 각도차={math.degrees(angle_diff):.1f}°")

        # 목표 도달 확인
        if distance <= GOAL_TOLERANCE:
            self.get_logger().info("[완료] 목표 도달")
            self.state = 'ALIGN_HEADING'
            return

        # # 방향 벗어남 확인 (5도 이상일 때 다시 회전)
        # if abs(angle_diff) > ANGLE_TOLERANCE:
        #     self.get_logger().info("[방향 수정] 다시 회전")
        #     self.state = 'TURN_TO_TARGET'
        #     return

        # 직진 명령 (미세 조정 없이 순수 직진)
        linear = clamp(0.5 * distance, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        if abs(linear) < MIN_LINEAR_VEL:
            linear = MIN_LINEAR_VEL * (1 if linear > 0 else -1)
        
        # 직진 중에는 각속도 0 (미세 조정 제거)
        angular = 0.0
        
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def align_heading_debug(self):
        target_pose = self.get_target_pose()
        if target_pose is None:
            self.state = 'IDLE'
            return
        
        target_heading = quaternion_to_yaw(target_pose.orientation)
        angle_diff = self.normalize_angle(target_heading - self.robot_yaw)
        
        self.get_logger().info(f"[방향정렬] 각도차={math.degrees(angle_diff):.1f}°")
        
        # 완료 조건 (5도 이하면 완료)
        if abs(angle_diff) <= ANGLE_TOLERANCE:
            self.get_logger().info(f"[완료] 방향정렬 완료! 각도차: {math.degrees(angle_diff):.1f}°")
            self.stop_robot()
            
            # 완료된 타겟 정리
            completed_target_id = self.target_id
            self.state = 'IDLE'
            self.target_id = -1
            
            # 저장된 위치 삭제
            if completed_target_id in self.saved_target_poses:
                del self.saved_target_poses[completed_target_id]
                self.get_logger().info(f"[정리] 타겟 ID {completed_target_id} 삭제")
            
            self.get_logger().info("[미션 완료] IDLE 상태로 복귀")
            return
        
        # 회전 명령 (단계별 속도)
        if abs(angle_diff) > math.radians(30):
            angular = clamp(0.8 * angle_diff, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        elif abs(angle_diff) > ANGLE_TOLERANCE:
            angular = clamp(0.5 * angle_diff, -MAX_ANGULAR_VEL * 0.6, MAX_ANGULAR_VEL * 0.6)
        else:
            angular = clamp(0.3 * angle_diff, -MAX_ANGULAR_VEL * 0.3, MAX_ANGULAR_VEL * 0.3)
        
        if abs(angle_diff) > ANGLE_TOLERANCE and abs(angular) < MIN_ANGULAR_VEL:
            angular = MIN_ANGULAR_VEL * (1 if angular > 0 else -1)
        
        twist = Twist()
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.get_logger().info("[로봇 정지]")

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