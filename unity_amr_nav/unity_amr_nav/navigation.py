import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
import math
import numpy as np
from collections import deque
import heapq
from enum import Enum

class NavigationState(Enum):
    IDLE = 0
    ROTATING_TO_WAYPOINT = 1
    MOVING_TO_WAYPOINT = 2
    REACHED_WAYPOINT = 3
    GOAL_REACHED = 4

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        self.goal_sub = self.create_subscription(PointStamped, '/clicked_point', self.goal_callback, 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/unity_odom', self.odom_callback, 10)
        
        # Navigation state
        self.current_x = None
        self.current_y = None
        self.current_yaw = 0.0
        self.current_linear_velocity = 0.0
        self.goal_x = None
        self.goal_y = None
        
        # 상태머신 관련
        self.navigation_state = NavigationState.IDLE
        self.current_waypoint_index = 0
        self.planned_path = []
        self.target_yaw = 0.0  # 목표 회전각
        
        # 회전 제어 관련 (안정성 개선)
        self.rotation_start_time = None
        self.last_yaw_error = 0.0
        self.rotation_stable_count = 0
        
        # 데이터 수신 상태 추적
        self.odom_received_count = 0
        
        # Map and grid settings
        self.grid_resolution = 0.1
        self.map_width = 66
        self.map_height = 64
        self.map_origin_x = -3.34
        self.map_origin_y = -3.29
        self.occupancy_grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # 장애물 설정
        self.setup_obstacles()
        
        # Control parameters (개선된 값들)
        self.rotation_tolerance = math.radians(15)  # 15도로 증가 (더 관대하게)
        self.position_tolerance = 0.2
        self.max_angular_speed = 0.7  # 증가 (더 빠른 회전)
        self.min_angular_speed = 0.2  # 최소 속도 추가
        self.max_linear_speed = 0.2
        
        # 적응형 제어 게인
        self.kp_angular_high = 1.0  # 큰 오차용
        self.kp_angular_mid = 0.5   # 중간 오차용  
        self.kp_angular_low = 0.2   # 작은 오차용
        
        # Control timer
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        
        # 상태 체크 타이머
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('개선된 State Machine Navigation System 시작됨')

    def print_status(self):
        """주기적으로 시스템 상태를 출력"""
        if self.current_x is not None:
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw) if self.navigation_state == NavigationState.ROTATING_TO_WAYPOINT else 0
            self.get_logger().info(
                f'상태: {self.navigation_state.name}, '
                f'위치: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'현재각: {math.degrees(self.current_yaw):.1f}°, '
                f'목표각: {math.degrees(self.target_yaw):.1f}°, '
                f'오차: {math.degrees(yaw_error):.1f}°, '
                f'웨이포인트: {self.current_waypoint_index}/{len(self.planned_path)}'
            )
        else:
            self.get_logger().warn('Odometry 데이터 대기 중...')

    def setup_obstacles(self):
        """장애물 설정"""
        shelf_points = [
            (0.37, 1.07), (0.03, 1.06), (0.06, 0.07), (0.36, 0.04)
        ]
        min_x = min(x for x, _ in shelf_points)
        max_x = max(x for x, _ in shelf_points)
        min_y = min(y for _, y in shelf_points)
        max_y = max(y for _, y in shelf_points)
        
        obstacle_count = 0
        for x in np.arange(min_x, max_x + self.grid_resolution, self.grid_resolution):
            for y in np.arange(min_y, max_y + self.grid_resolution, self.grid_resolution):
                grid_x, grid_y = self.world_to_grid(x, y)
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    self.occupancy_grid[grid_y][grid_x] = 1
                    obstacle_count += 1
        
        self.get_logger().info(f'장애물 설정 완료: {obstacle_count}개 셀')

    def world_to_grid(self, x, y):
        grid_x = int((x - self.map_origin_x) / self.grid_resolution)
        grid_y = int((y - self.map_origin_y) / self.grid_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y):
        x = grid_x * self.grid_resolution + self.map_origin_x
        y = grid_y * self.grid_resolution + self.map_origin_y
        return (x, y)

    def goal_callback(self, msg):
        """목표 위치 수신 및 경로 계획"""
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        
        self.get_logger().info(f'새 목표 수신: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn("로봇의 현재 위치 정보를 아직 수신하지 못했습니다.")
            return

        # 기존 네비게이션 중단
        self.stop_robot()
        
        # 경로 계획
        path = self.plan_path_astar(
            (self.current_x, self.current_y), 
            (self.goal_x, self.goal_y)
        )
        
        if path:
            self.planned_path = path
            self.current_waypoint_index = 0
            self.navigation_state = NavigationState.ROTATING_TO_WAYPOINT
            self.calculate_target_yaw()
            self.publish_path_to_unity(path)
            self.rotation_start_time = self.get_clock().now()  # 회전 시작 시간 기록
            self.get_logger().info(f'경로 계획 완료: {len(path)}개 웨이포인트')
            self.get_logger().info(f'첫 번째 웨이포인트로 회전 시작: ({path[0][0]:.2f}, {path[0][1]:.2f})')
        else:
            self.get_logger().error('경로 계획 실패!')
            self.navigation_state = NavigationState.IDLE

    def calculate_target_yaw(self):
        """현재 웨이포인트를 향한 목표 회전각 계산"""
        if self.current_waypoint_index >= len(self.planned_path):
            return
            
        target_point = self.planned_path[self.current_waypoint_index]
        dx = target_point[0] - self.current_x
        dy = target_point[1] - self.current_y
        
        self.target_yaw = math.atan2(dy, dx)
        
        # 현재 각도와 목표 각도의 차이를 최단 경로로 계산
        yaw_diff = self.normalize_angle(self.target_yaw - self.current_yaw)
        
        self.get_logger().info(
            f'목표 회전각 계산: 웨이포인트({target_point[0]:.2f}, {target_point[1]:.2f}), '
            f'현재각: {math.degrees(self.current_yaw):.1f}°, '
            f'목표각: {math.degrees(self.target_yaw):.1f}°, '
            f'최단경로 오차: {math.degrees(yaw_diff):.1f}°'
        )

    def state_machine_loop(self):
        """상태머신 메인 루프"""
        if self.current_x is None or self.current_y is None:
            return
            
        if self.navigation_state == NavigationState.IDLE:
            pass
            
        elif self.navigation_state == NavigationState.ROTATING_TO_WAYPOINT:
            self.handle_rotation_state()
            
        elif self.navigation_state == NavigationState.MOVING_TO_WAYPOINT:
            self.handle_movement_state()
            
        elif self.navigation_state == NavigationState.REACHED_WAYPOINT:
            self.handle_waypoint_reached()
            
        elif self.navigation_state == NavigationState.GOAL_REACHED:
            self.handle_goal_reached()

    def handle_rotation_state(self):
        """개선된 회전 상태 처리"""
        yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
        
        # 회전 완료 체크 (안정성 확인 추가)
        if abs(yaw_error) < self.rotation_tolerance:
            self.rotation_stable_count += 1
            
            # 3번 연속으로 허용 오차 내에 있으면 회전 완료
            if self.rotation_stable_count >= 3:
                self.get_logger().info(f'회전 완료! 최종 오차: {math.degrees(yaw_error):.1f}°')
                self.stop_robot()
                self.rotation_stable_count = 0
                self.navigation_state = NavigationState.REACHED_WAYPOINT
                return
        else:
            self.rotation_stable_count = 0  # 오차가 크면 카운트 리셋
        
        # 적응형 제어 게인 선택
        abs_error = abs(yaw_error)
        if abs_error > math.radians(90):      # 90도 이상
            kp = self.kp_angular_high * 0.6   # 큰 오차에서는 더 부드럽게
        elif abs_error > math.radians(30):    # 30-90도
            kp = self.kp_angular_mid
        else:                                 # 30도 미만
            kp = self.kp_angular_low
        
        # 회전 제어 명령 생성
        cmd = Twist()
        cmd.angular.z = kp * yaw_error
        
        # 동적 속도 제한
        if abs_error > math.radians(60):      # 60도 이상: 빠르게
            max_speed = self.max_angular_speed
        elif abs_error > math.radians(20):    # 20-60도: 중간 속도
            max_speed = self.max_angular_speed * 0.7
        else:                                 # 20도 미만: 천천히
            max_speed = self.max_angular_speed * 0.4
        
        # 속도 제한 적용
        cmd.angular.z = max(min(cmd.angular.z, max_speed), -max_speed)
        
        # 최소 속도 보장 (데드존 방지)
        if abs(cmd.angular.z) < self.min_angular_speed and abs_error > self.rotation_tolerance:
            cmd.angular.z = self.min_angular_speed if cmd.angular.z >= 0 else -self.min_angular_speed
        
        self.cmd_vel_pub.publish(cmd)
        
        # 회전 시간 초과 체크 (10초)
        if self.rotation_start_time is not None:
            elapsed = (self.get_clock().now() - self.rotation_start_time).nanoseconds / 1e9
            if elapsed > 10.0:  # 10초 초과시 강제 완료
                self.get_logger().warn(f'회전 시간 초과! 현재 오차: {math.degrees(yaw_error):.1f}°')
                self.navigation_state = NavigationState.REACHED_WAYPOINT
                return
        
        # 가끔씩 로그 출력
        if self.odom_received_count % 10 == 0:
            self.get_logger().info(
                f'회전 중: 현재각={math.degrees(self.current_yaw):.1f}°, '
                f'목표각={math.degrees(self.target_yaw):.1f}°, '
                f'오차={math.degrees(yaw_error):.1f}°, '
                f'각속도={cmd.angular.z:.2f}rad/s, '
                f'게인={kp:.1f}, 안정카운트={self.rotation_stable_count}'
            )

    def handle_movement_state(self):
        """직진 상태 처리 (현재 주석처리)"""
        self.navigation_state = NavigationState.REACHED_WAYPOINT

    def handle_waypoint_reached(self):
        """웨이포인트 도달 처리"""
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index >= len(self.planned_path):
            self.navigation_state = NavigationState.GOAL_REACHED
            self.get_logger().info('모든 웨이포인트 완료!')
        else:
            next_waypoint = self.planned_path[self.current_waypoint_index]
            self.get_logger().info(
                f'다음 웨이포인트({self.current_waypoint_index+1}/{len(self.planned_path)})로 회전: '
                f'({next_waypoint[0]:.2f}, {next_waypoint[1]:.2f})'
            )
            self.calculate_target_yaw()
            self.navigation_state = NavigationState.ROTATING_TO_WAYPOINT
            self.rotation_start_time = self.get_clock().now()  # 새 회전 시작 시간
            self.rotation_stable_count = 0

    def handle_goal_reached(self):
        """목표 도달 처리"""
        self.stop_robot()
        self.navigation_state = NavigationState.IDLE
        self.get_logger().info('목표 도달 완료!')

    def stop_robot(self):
        """로봇 정지"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화 (최단 경로)"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def plan_path_astar(self, start, goal):
        """A* 알고리즘으로 경로 계획"""
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        def get_neighbors(pos):
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                nx, ny = pos[0] + dx, pos[1] + dy
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height and self.occupancy_grid[ny][nx] == 0:
                    neighbors.append((nx, ny))
            return neighbors
        
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])

        if not (0 <= start_grid[0] < self.map_width and 0 <= start_grid[1] < self.map_height):
            self.get_logger().error(f"시작 그리드 ({start_grid})가 맵 범위를 벗어남")
            return None
        if not (0 <= goal_grid[0] < self.map_width and 0 <= goal_grid[1] < self.map_height):
            self.get_logger().error(f"목표 그리드 ({goal_grid})가 맵 범위를 벗어남")
            return None
        
        if self.occupancy_grid[start_grid[1]][start_grid[0]] == 1:
            self.get_logger().error(f"시작점이 장애물입니다")
            return None
        if self.occupancy_grid[goal_grid[1]][goal_grid[0]] == 1:
            self.get_logger().error(f"목표점이 장애물입니다")
            return None

        self.get_logger().info(f"A* 경로 계획: {start_grid} -> {goal_grid}")
        
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: heuristic(start_grid, goal_grid)}
        
        while open_set:
            current_f_score, current = heapq.heappop(open_set)
            
            if current == goal_grid:
                path = []
                while current in came_from:
                    world_pos = self.grid_to_world(current[0], current[1])
                    path.append(world_pos)
                    current = came_from[current]
                world_pos = self.grid_to_world(start_grid[0], start_grid[1])
                path.append(world_pos)
                path.reverse()
                
                simplified_path = self.simplify_path(path)
                return simplified_path
            
            for neighbor in get_neighbors(current):
                cost = 1.0 if abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1]) == 1 else math.sqrt(2)
                tentative_g_score = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None

    def simplify_path(self, path):
        """경로 간소화"""
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = i + 1
            max_jump = min(len(path) - 1, i + 10)
            last_valid = i + 1
            
            for k in range(i + 2, max_jump + 1):
                if self.is_line_clear(simplified[-1], path[k]):
                    last_valid = k
                else:
                    break
            
            simplified.append(path[last_valid])
            i = last_valid
        
        self.get_logger().info(f'경로 간소화: {len(path)} -> {len(simplified)} 웨이포인트')
        return simplified

    def is_line_clear(self, start, end):
        """직선 경로 장애물 체크"""
        steps = int(math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2) / self.grid_resolution)
        if steps == 0:
            return True
            
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            
            grid_x, grid_y = self.world_to_grid(x, y)
            
            if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                return False
            
            if self.occupancy_grid[grid_y][grid_x] == 1:
                return False
        return True

    def publish_path_to_unity(self, path):
        """Unity로 경로 전송"""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for point in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def odom_callback(self, msg):
        """Odometry 콜백"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_linear_velocity = msg.twist.twist.linear.x
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()