import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
import math
import numpy as np
from collections import deque
import heapq

class AdvancedNavigation(Node):
    def __init__(self):
        super().__init__('advanced_navigation')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        self.goal_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.goal_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/unity_odom', self.odom_callback, 10)
        
        # Navigation state
        self.current_x = None
        self.current_y = None
        self.current_yaw = 0.0
        self.current_linear_velocity = 0.0
        self.goal_x = None
        self.goal_y = None
        self.is_navigating = False
        
        # 데이터 수신 상태 추적
        self.odom_received_count = 0
        self.last_odom_time = None
        
        # Path planning
        self.planned_path = []
        self.current_path_index = 0
        self.lookahead_distance = 0.15
        
        # Map and grid settings
        self.grid_resolution = 0.1  # 10cm
        self.map_width = 66  # 6.57m / 0.1 ≈ 66
        self.map_height = 64  # 6.36m / 0.1 ≈ 64
        self.map_origin_x = -3.34  # 맵의 최소 x
        self.map_origin_y = -3.29  # 맵의 최소 y
        self.occupancy_grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # 장애물(선반) 설정
        self.setup_obstacles()
        
        # Control parameters
        self.goal_tolerance = 0.3
        self.path_tolerance = 0.2
        self.max_linear_speed = 0.2
        self.max_angular_speed = 2.0
        self.kp_angular = 1.0
        self.kv = 0.3
        
        # Control timer
        self.timer = self.create_timer(0.05, self.navigation_loop)
        
        # 상태 체크 타이머 (디버깅용)
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('Advanced Navigation System 시작됨')
        self.get_logger().info(f'구독 토픽: /unity_odom, /clicked_point')
        self.get_logger().info(f'발행 토픽: /cmd_vel, /planned_path')

    def print_status(self):
        """주기적으로 시스템 상태를 출력"""
        if self.current_x is not None:
            self.get_logger().info(
                f'시스템 상태 - 위치: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'Odom 수신 횟수: {self.odom_received_count}, '
                f'네비게이션 중: {self.is_navigating}'
            )
        else:
            self.get_logger().warn('Odometry 데이터를 아직 수신하지 못했습니다. Unity에서 /unity_odom 토픽이 발행되고 있는지 확인하세요.')

    def world_to_grid(self, x, y):
        """월드 좌표를 그리드 좌표로 변환"""
        grid_x = int((x - self.map_origin_x) / self.grid_resolution)
        grid_y = int((y - self.map_origin_y) / self.grid_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y):
        """그리드 좌표를 월드 좌표로 변환"""
        x = grid_x * self.grid_resolution + self.map_origin_x
        y = grid_y * self.grid_resolution + self.map_origin_y
        return (x, y)

    def setup_obstacles(self):
        """선반 좌표를 기반으로 occupancy grid에 장애물 설정"""
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
                else:
                    self.get_logger().warn(f'장애물 좌표 ({x:.2f}, {y:.2f})가 맵 경계를 벗어납니다.')
        
        self.get_logger().info(f'장애물 설정 완료: {obstacle_count}개 셀, 범위 x=[{min_x:.2f}, {max_x:.2f}], y=[{min_y:.2f}, {max_y:.2f}]')

    def goal_callback(self, msg):
        """목표 위치 수신 및 경로 계획"""
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        
        self.get_logger().info(f'새 목표 수신: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn("로봇의 현재 위치 정보를 아직 수신하지 못했습니다. 경로 계획을 연기합니다.")
            self.get_logger().warn("Unity에서 /unity_odom 토픽이 제대로 발행되고 있는지 확인하세요.")
            return

        self.get_logger().info(f'현재 위치: ({self.current_x:.2f}, {self.current_y:.2f}) -> 목표: ({self.goal_x:.2f}, {self.goal_y:.2f})')

        path = self.plan_path_astar(
            (self.current_x, self.current_y), 
            (self.goal_x, self.goal_y)
        )
        
        if path:
            self.planned_path = path
            self.current_path_index = 0
            self.is_navigating = True
            self.publish_path_to_unity(path)
            self.get_logger().info(f'경로 계획 완료: {len(path)}개 웨이포인트')
        else:
            self.get_logger().error('경로 계획 실패! 목표 지점이 장애물 안에 있거나 도달 불가능한 위치일 수 있습니다.')

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

        # 경계 체크
        if not (0 <= start_grid[0] < self.map_width and 0 <= start_grid[1] < self.map_height):
            self.get_logger().error(f"시작 그리드 ({start_grid})가 맵 범위를 벗어남. 맵 크기: {self.map_width}x{self.map_height}")
            return None
        if not (0 <= goal_grid[0] < self.map_width and 0 <= goal_grid[1] < self.map_height):
            self.get_logger().error(f"목표 그리드 ({goal_grid})가 맵 범위를 벗어남. 맵 크기: {self.map_width}x{self.map_height}")
            return None
        
        # 장애물 체크
        if self.occupancy_grid[start_grid[1]][start_grid[0]] == 1:
            self.get_logger().error(f"시작점 그리드({start_grid})이 장애물입니다.")
            return None
        if self.occupancy_grid[goal_grid[1]][goal_grid[0]] == 1:
            self.get_logger().error(f"목표점 그리드({goal_grid})이 장애물입니다.")
            return None

        self.get_logger().info(f"A* 경로 계획 시작: {start_grid} -> {goal_grid}")
        
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: heuristic(start_grid, goal_grid)}
        
        nodes_explored = 0
        while open_set:
            current_f_score, current = heapq.heappop(open_set)
            nodes_explored += 1
            
            if current == goal_grid:
                # 경로 재구성
                path = []
                while current in came_from:
                    world_pos = self.grid_to_world(current[0], current[1])
                    path.append(world_pos)
                    current = came_from[current]
                world_pos = self.grid_to_world(start_grid[0], start_grid[1])
                path.append(world_pos)
                path.reverse()
                
                smoothed_path = self.smooth_path(path)
                self.get_logger().info(f'A* 완료: {nodes_explored}개 노드 탐색, 원본 {len(path)}개 -> 스무딩 후 {len(smoothed_path)}개 포인트')
                return smoothed_path
            
            for neighbor in get_neighbors(current):
                cost = 1.0 if abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1]) == 1 else math.sqrt(2)
                tentative_g_score = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        self.get_logger().error(f"A* 경로 찾기 실패. {nodes_explored}개 노드 탐색함")
        return None

    def smooth_path(self, path):
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = i + 1
            last_valid_j = j
            while j < len(path):
                if self.is_line_clear(smoothed[-1], path[j]):
                    last_valid_j = j
                    j += 1
                else:
                    break
            smoothed.append(path[last_valid_j])
            i = last_valid_j
        
        return smoothed

    def is_line_clear(self, start, end):
        def world_to_grid_local(x, y):
            grid_x = int((x - self.map_origin_x) / self.grid_resolution)
            grid_y = int((y - self.map_origin_y) / self.grid_resolution)
            return (grid_x, grid_y)

        steps = int(math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2) / self.grid_resolution)
        if steps == 0:
            return True
            
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            
            grid_x, grid_y = world_to_grid_local(x, y)
            
            if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                return False
            
            if self.occupancy_grid[grid_y][grid_x] == 1:
                return False
        return True

    def publish_path_to_unity(self, path):
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
        self.get_logger().info(f'Unity로 경로 전송 완료: {len(path)}개 포인트를 /planned_path 토픽으로 발행')

    def odom_callback(self, msg):
        """Odometry 콜백 - 로봇의 현재 위치 정보 업데이트"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_linear_velocity = msg.twist.twist.linear.x
        
        # 회전각 계산 (쿼터니언 -> 오일러)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received_count += 1
        self.last_odom_time = self.get_clock().now()
        
        # 처음 몇 번의 수신은 로그로 확인
        if self.odom_received_count <= 5 or self.odom_received_count % 50 == 0:
            self.get_logger().info(
                f'Odom 수신 #{self.odom_received_count}: '
                f'위치=({self.current_x:.2f}, {self.current_y:.2f}), '
                f'각도={math.degrees(self.current_yaw):.1f}°, '
                f'속도={self.current_linear_velocity:.2f}m/s'
            )

    def navigation_loop(self):
        if not self.is_navigating or not self.planned_path:
            return
        
        if self.current_x is None or self.current_y is None:
            return
        
        # 최종 목표까지의 거리 체크
        final_target_dist = math.sqrt(
            (self.planned_path[-1][0] - self.current_x)**2 + 
            (self.planned_path[-1][1] - self.current_y)**2
        )
        
        if final_target_dist < self.goal_tolerance:
            self.stop_navigation()
            return
        
        # Pure Pursuit 제어
        current_speed = self.get_current_linear_velocity()
        dynamic_lookahead = self.lookahead_distance + self.kv * current_speed
        dynamic_lookahead = max(0.1, min(dynamic_lookahead, 1.0))

        target_point = self.find_lookahead_point(dynamic_lookahead)
        
        if target_point is None:
            target_point = self.planned_path[-1]
        
        cmd = self.pure_pursuit_control(target_point)
        
        # 매번 로그를 출력하지 않고 가끔씩만 출력
        if self.odom_received_count % 20 == 0:
            self.get_logger().info(
                f'제어 명령: 선속도={cmd.linear.x:.2f}m/s, 각속도={cmd.angular.z:.2f}rad/s, '
                f'목표점=({target_point[0]:.2f}, {target_point[1]:.2f})'
            )
        
        # self.cmd_vel_pub.publish(cmd)
        
        self.update_path_progress()

    def find_lookahead_point(self, lookahead_dist):
        for i in range(self.current_path_index, len(self.planned_path)):
            point = self.planned_path[i]
            dist = math.sqrt((point[0] - self.current_x)**2 + (point[1] - self.current_y)**2)
            
            if dist >= lookahead_dist or i == len(self.planned_path) - 1:
                return point
        
        return None

    def pure_pursuit_control(self, target_point):
        cmd = Twist()
        
        relative_x = target_point[0] - self.current_x
        relative_y = target_point[1] - self.current_y
        
        rotated_x = relative_x * math.cos(self.current_yaw) + relative_y * math.sin(self.current_yaw)
        rotated_y = -relative_x * math.sin(self.current_yaw) + relative_y * math.cos(self.current_yaw)
        
        distance = math.sqrt(rotated_x**2 + rotated_y**2)
        
        if distance < self.goal_tolerance:
            return cmd

        alpha = math.atan2(rotated_y, rotated_x)
        
        cmd.linear.x = self.max_linear_speed
        cmd.angular.z = self.kp_angular * alpha
        
        cmd.linear.x = min(self.max_linear_speed, cmd.linear.x)
        cmd.angular.z = min(max(cmd.angular.z, -self.max_angular_speed), self.max_angular_speed)
        
        if cmd.linear.x < 0:
            cmd.linear.x = 0.0

        return cmd

    def update_path_progress(self):
        if self.current_path_index < len(self.planned_path):
            current_target = self.planned_path[self.current_path_index]
            dist_to_target = math.sqrt(
                (current_target[0] - self.current_x)**2 + 
                (current_target[1] - self.current_y)**2
            )
            
            if dist_to_target < self.path_tolerance:
                self.current_path_index += 1
                if self.current_path_index < len(self.planned_path):
                    next_target = self.planned_path[self.current_path_index]
                    self.get_logger().info(
                        f'다음 웨이포인트로 이동: ({next_target[0]:.2f}, {next_target[1]:.2f})'
                    )

    def get_current_linear_velocity(self):
        return self.current_linear_velocity if self.current_linear_velocity is not None else 0.0

    def stop_navigation(self):
        self.is_navigating = False
        self.planned_path = []
        self.current_path_index = 0
        
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.get_logger().info('목표 도달! 네비게이션 완료')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 신호')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()