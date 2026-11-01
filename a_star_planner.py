
#!/usr/bin/env python3
import math
import heapq

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import TransformException


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')

        # Destination in map coordinates (metres)
        self.goal_x = -1.0
        self.goal_y = 1.3

        self.robot_radius = 0.18
        self.margin = 0.05
        self.threshold = 50
        self.goal_tolerance = 0.15
        self.stop_distance = 0.30
        self.linear_speed = 0.10
        self.angular_speed = 0.60

        self.map_msg = None
        self.grid = None
        self.blocked = None
        self.scan = None
        self.path_cells = []
        self.path_index = 0
        self.last_plan = 0.0
        self.map_changed = False
        self.last_warning = 0.0

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('A* planner node started.')

    def map_callback(self, msg):
        self.map_msg = msg
        info = msg.info
        self.width = info.width
        self.height = info.height
        self.res = info.resolution
        self.ox = info.origin.position.x
        self.oy = info.origin.position.y

        q = info.origin.orientation
        self.origin_yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))

        self.frame = msg.header.frame_id or 'map'
        self.grid = [
            list(msg.data[y*self.width:(y+1)*self.width])
            for y in range(self.height)]

        self.blocked = [
            [v < 0 or v >= self.threshold for v in row]
            for row in self.grid]

        # Inflate obstacles by robot radius plus safety margin.
        radius = int(math.ceil(
            (self.robot_radius + self.margin) / self.res))
        obstacles = [
            (x, y) for y in range(self.height)
            for x in range(self.width) if self.blocked[y][x]]

        for ox, oy in obstacles:
            for dy in range(-radius, radius+1):
                for dx in range(-radius, radius+1):
                    if dx*dx + dy*dy > radius*radius:
                        continue
                    x, y = ox+dx, oy+dy
                    if 0 <= x < self.width and 0 <= y < self.height:
                        self.blocked[y][x] = True

        self.map_changed = True
        self.get_logger().info(
            f'Map received: {self.width}x{self.height}, '
            f'{self.res:.3f} m/cell, frame={self.frame}')

    def scan_callback(self, msg):
        self.scan = msg

    def in_bounds(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def world_to_grid(self, wx, wy):
        dx, dy = wx-self.ox, wy-self.oy
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        lx, ly = c*dx+s*dy, -s*dx+c*dy
        return (math.floor(lx/self.res), math.floor(ly/self.res))

    def grid_to_world(self, cell):
        x, y = cell
        lx, ly = (x+0.5)*self.res, (y+0.5)*self.res
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (self.ox+c*lx-s*ly, self.oy+s*lx+c*ly)

    def is_blocked(self, cell):
        if not self.in_bounds(cell):
            return True
        x, y = cell
        return self.blocked[y][x]

    def nearest_free_goal(self, goal, radius=20):
        if not self.in_bounds(goal):
            return None
        if not self.is_blocked(goal):
            return goal

        gx, gy = goal
        candidates = []
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                cell = (gx+dx, gy+dy)
                if not self.in_bounds(cell) or self.is_blocked(cell):
                    continue
                x, y = cell
                if self.grid[y][x] != 0:
                    continue
                candidates.append((dx*dx+dy*dy, cell))

        return min(candidates)[1] if candidates else None

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame, 'base_footprint', Time()).transform
            q = t.rotation
            yaw = math.atan2(
                2*(q.w*q.z+q.x*q.y),
                1-2*(q.y*q.y+q.z*q.z))
            return t.translation.x, t.translation.y, yaw
        except TransformException as e:
            now = self.get_clock().now().nanoseconds / 1e9
            if now-self.last_warning > 3:
                self.get_logger().warning(f'Waiting for TF: {e}')
                self.last_warning = now
            return None

    @staticmethod
    def heuristic(a, b):
        dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
        return max(dx, dy)+(math.sqrt(2)-1)*min(dx, dy)

    def neighbors(self, cell):
        x, y = cell
        for dx, dy, cost in [
            (1,0,1), (-1,0,1), (0,1,1), (0,-1,1),
            (1,1,math.sqrt(2)), (1,-1,math.sqrt(2)),
            (-1,1,math.sqrt(2)), (-1,-1,math.sqrt(2))]:
            nxt = (x+dx, y+dy)
            if self.is_blocked(nxt):
                continue
            if dx and dy and (
                self.is_blocked((x+dx, y)) or
                self.is_blocked((x, y+dy))):
                continue
            yield nxt, cost

    def astar(self, start, goal):
        if self.is_blocked(start) or self.is_blocked(goal):
            return []

        queue = [(self.heuristic(start, goal), 0, start)]
        came = {}
        costs = {start: 0}
        visited = set()

        while queue:
            _, cost, current = heapq.heappop(queue)
            if current in visited:
                continue
            if current == goal:
                result = [current]
                while current in came:
                    current = came[current]
                    result.append(current)
                return result[::-1]

            visited.add(current)
            for nxt, step in self.neighbors(current):
                new_cost = cost + step
                if new_cost < costs.get(nxt, float('inf')):
                    costs[nxt] = new_cost
                    came[nxt] = current
                    heapq.heappush(
                        queue,
                        (new_cost+self.heuristic(nxt, goal),
                         new_cost, nxt))
        return []

    def publish_path(self, cells):
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for cell in cells:
            x, y = self.grid_to_world(cell)
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def plan(self, x, y):
        start = self.world_to_grid(x, y)
        requested = self.world_to_grid(self.goal_x, self.goal_y)

        if not self.in_bounds(start) or not self.in_bounds(requested):
            self.get_logger().error(
                f'Start {start} or goal {requested} is outside the map.')
            return False

        goal = self.nearest_free_goal(requested)
        if goal is None:
            self.get_logger().error(
                'No safe free goal near requested destination. '
                'Choose a mapped open area.')
            return False

        if goal != requested:
            self.get_logger().warning(
                f'Goal adjusted to nearby safe cell {goal}, '
                f'world position {self.grid_to_world(goal)}')

        path = self.astar(start, goal)
        if not path:
            self.get_logger().error(
                'A* found no route. Check map coverage and connectivity.')
            return False

        self.path_cells = path
        self.path_index = min(1, len(path)-1)
        self.publish_path(path)
        self.get_logger().info(f'Path found: {len(path)} cells.')
        return True

    def stop(self):
        self.cmd_pub.publish(Twist())

    def obstacle_ahead(self):
        if self.scan is None:
            return False
        scan = self.scan
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i*scan.angle_increment
            if abs(angle) <= math.radians(20):
                if math.isfinite(distance) and (
                    scan.range_min <= distance <= self.stop_distance):
                    return True
        return False

    def control_loop(self):
        if self.map_msg is None:
            return

        pose = self.robot_pose()
        if pose is None:
            self.stop()
            return

        x, y, yaw = pose
        now = self.get_clock().now().nanoseconds / 1e9

        if (not self.path_cells or self.map_changed or
                now-self.last_plan >= 2.0):
            self.stop()
            ok = self.plan(x, y)
            self.last_plan = now
            self.map_changed = False
            if not ok:
                return

        if self.obstacle_ahead():
            self.stop()
            if now-self.last_warning > 2:
                self.get_logger().warning(
                    'Close obstacle ahead: stopped for safety.')
                self.last_warning = now
            return

        while self.path_index < len(self.path_cells):
            tx, ty = self.grid_to_world(self.path_cells[self.path_index])
            if math.hypot(tx-x, ty-y) > max(0.08, self.res*0.8):
                break
            self.path_index += 1

        if self.path_index >= len(self.path_cells):
            gx, gy = self.grid_to_world(self.path_cells[-1])
            if math.hypot(gx-x, gy-y) <= self.goal_tolerance:
                self.stop()
                self.get_logger().info('Goal reached.')
                self.path_cells = []
            else:
                self.path_cells = []
                self.stop()
            return

        tx, ty = self.grid_to_world(self.path_cells[self.path_index])
        desired = math.atan2(ty-y, tx-x)
        error = math.atan2(math.sin(desired-yaw), math.cos(desired-yaw))

        cmd = Twist()
        if abs(error) > 0.45:
            cmd.angular.z = max(
                -self.angular_speed,
                min(self.angular_speed, 1.5*error))
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = max(
                -self.angular_speed,
                min(self.angular_speed, 1.5*error))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
