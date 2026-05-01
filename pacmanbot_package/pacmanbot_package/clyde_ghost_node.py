#!/usr/bin/env python3

import heapq
import math
import os
import random
import struct

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


class ClydeGhostNode(Node):
    def __init__(self):
        super().__init__('clyde_ghost_node')

        self.robot_ns = '/robot_15'
        self.map_frame = 'map'
        self.map_package = 'pacmanbot_package'
        self.map_yaml_name = 'map_01.yaml'

        self.free_pixel_threshold = 250
        self.wall_clearance_m = 0.25
        self.speed_mps = 0.25
        self.update_period_s = 0.2
        self.obstacle_radius_m = 0.05
        self.obstacle_z_m = 0.25
        self.global_costmap_clear_period_s = 1.0
        self.global_costmap_clear_move_m = self.obstacle_radius_m * 2.0

        self.resolution = None
        self.origin = None
        self.height = None
        self.width = None
        self.free_mask = None
        self.free_cells = []

        self.current_x = None
        self.current_y = None
        self.path = []
        self.path_index = 0
        self.game_state = 'menu'
        self.selected_start_cell = None
        self.last_global_costmap_clear_time = None
        self.last_global_costmap_clear_position = None
        self.global_costmap_clear_in_progress = False

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'{self.robot_ns}/clyde_pose',
            10
        )
        self.marker_pub = self.create_publisher(
            Marker,
            f'{self.robot_ns}/clyde_marker',
            10
        )
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            f'{self.robot_ns}/clyde_obstacle',
            10
        )
        self.game_state_sub = self.create_subscription(
            String,
            f'{self.robot_ns}/game_state',
            self.game_state_callback,
            latched_qos
        )
        self.speed_sub = self.create_subscription(
            Float32,
            f'{self.robot_ns}/clyde/speed',
            self.speed_callback,
            latched_qos
        )
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            f'{self.robot_ns}/clicked_point',
            self.clicked_point_callback,
            10
        )
        self.global_clicked_point_sub = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.clicked_point_callback,
            10
        )
        self.reset_srv = self.create_service(
            Trigger,
            f'{self.robot_ns}/reset_clyde',
            self.reset_clyde_callback
        )
        self.global_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            f'{self.robot_ns}/global_costmap/clear_entirely_global_costmap'
        )

        if self.load_map():
            self.initialize_position()
            self.pick_new_destination()
            self.timer = self.create_timer(self.update_period_s, self.update_clyde)
            self.get_logger().info(
                f'Clyde ghost started with {len(self.free_cells)} valid free cells'
            )
        else:
            self.timer = self.create_timer(1.0, self.publish_current_state)
            self.get_logger().error('Clyde ghost started without a valid map')

    def game_state_callback(self, msg):
        self.game_state = msg.data.strip().lower()

    def speed_callback(self, msg):
        speed = float(msg.data)
        if speed <= 0.0:
            self.get_logger().warn(f'Ignoring invalid Clyde speed: {speed:.2f}')
            return

        self.speed_mps = speed
        self.get_logger().info(f'Clyde speed set to {self.speed_mps:.2f} m/s')

    def clicked_point_callback(self, msg):
        if not self.free_cells:
            self.get_logger().warn('Ignoring Clyde spawn point; map is not ready')
            return

        if self.game_state == 'running':
            self.get_logger().warn('Ignoring Clyde spawn point while game is running')
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'Ignoring Clyde spawn point in frame "{msg.header.frame_id}"; '
                f'expected "{self.map_frame}"'
            )
            return

        cell = self.world_to_grid(msg.point.x, msg.point.y)
        if not self.is_free_cell(cell):
            self.get_logger().warn(
                f'Clicked Clyde spawn ({msg.point.x:.2f}, {msg.point.y:.2f}) '
                'is not a valid safe map cell'
            )
            return

        self.selected_start_cell = cell
        self.current_x, self.current_y = self.grid_to_world(*cell)
        self.pick_new_destination()
        self.publish_current_state()
        self.get_logger().info(
            f'Selected Clyde spawn: ({self.current_x:.2f}, {self.current_y:.2f})'
        )

    def reset_clyde_callback(self, request, response):
        del request

        if not self.free_cells:
            response.success = False
            response.message = 'Cannot reset Clyde; no valid free cells'
            return response

        self.initialize_position()
        self.pick_new_destination()
        self.publish_current_state()

        response.success = True
        response.message = 'Clyde respawned'
        self.get_logger().info(response.message)
        return response

    def load_map(self):
        pkg_share = get_package_share_directory(self.map_package)
        yaml_path = os.path.join(pkg_share, 'maps', self.map_yaml_name)

        try:
            with open(yaml_path, 'r') as f:
                map_yaml = yaml.safe_load(f)
        except OSError as exc:
            self.get_logger().error(f'Failed to read map yaml {yaml_path}: {exc}')
            return False

        image_path = os.path.join(pkg_share, 'maps', map_yaml['image'])
        map_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            self.get_logger().error(f'Failed to load map image: {image_path}')
            return False

        self.resolution = float(map_yaml['resolution'])
        self.origin = map_yaml['origin']
        self.height, self.width = map_img.shape

        free_mask = map_img > self.free_pixel_threshold
        clearance_px = max(1, int(round(self.wall_clearance_m / self.resolution)))

        safe_mask = free_mask.copy()
        for y in range(self.height):
            for x in range(self.width):
                if not free_mask[y, x]:
                    safe_mask[y, x] = False
                    continue

                if (
                    x < clearance_px or
                    x >= self.width - clearance_px or
                    y < clearance_px or
                    y >= self.height - clearance_px
                ):
                    safe_mask[y, x] = False
                    continue

                neighborhood = free_mask[
                    y - clearance_px:y + clearance_px + 1,
                    x - clearance_px:x + clearance_px + 1
                ]
                if not neighborhood.all():
                    safe_mask[y, x] = False

        self.free_mask = safe_mask
        self.free_cells = [
            (x, y)
            for y in range(self.height)
            for x in range(self.width)
            if self.free_mask[y, x]
        ]

        if not self.free_cells:
            self.get_logger().error('No valid free cells found for Clyde')
            return False

        self.get_logger().info(
            f'Loaded Clyde map: {self.width}x{self.height}, '
            f'resolution={self.resolution:.3f} m/px'
        )
        return True

    def initialize_position(self):
        start_cell = self.selected_start_cell
        if start_cell is None:
            start_cell = random.choice(self.free_cells)

        self.current_x, self.current_y = self.grid_to_world(*start_cell)
        self.get_logger().info(
            f'Clyde initial position: ({self.current_x:.2f}, {self.current_y:.2f})'
        )

    def pick_new_destination(self):
        if self.current_x is None or self.current_y is None:
            return

        start = self.world_to_grid(self.current_x, self.current_y)
        for _ in range(20):
            goal = random.choice(self.free_cells)
            if goal == start:
                continue

            path = self.plan_path(start, goal)
            if path:
                self.path = path
                self.path_index = 0
                goal_x, goal_y = self.grid_to_world(*goal)
                self.get_logger().info(
                    f'Clyde path planned to ({goal_x:.2f}, {goal_y:.2f}) '
                    f'with {len(path)} cells'
                )
                return

        self.get_logger().warn('Clyde could not find a new reachable destination')
        self.path = []
        self.path_index = 0

    def plan_path(self, start, goal):
        if not self.is_free_cell(start) or not self.is_free_cell(goal):
            return []

        frontier = []
        heapq.heappush(frontier, (0.0, start))
        came_from = {start: None}
        cost_so_far = {start: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break

            for neighbor, step_cost in self.neighbors(current):
                new_cost = cost_so_far[current] + step_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        if goal not in came_from:
            return []

        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def neighbors(self, cell):
        x, y = cell
        candidates = [
            (x + 1, y, 1.0),
            (x - 1, y, 1.0),
            (x, y + 1, 1.0),
            (x, y - 1, 1.0),
            (x + 1, y + 1, math.sqrt(2.0)),
            (x - 1, y + 1, math.sqrt(2.0)),
            (x + 1, y - 1, math.sqrt(2.0)),
            (x - 1, y - 1, math.sqrt(2.0)),
        ]

        for nx, ny, cost in candidates:
            neighbor = (nx, ny)
            if not self.is_free_cell(neighbor):
                continue

            # Prevent diagonal paths from squeezing through wall corners.
            if nx != x and ny != y:
                if not self.is_free_cell((nx, y)) or not self.is_free_cell((x, ny)):
                    continue

            yield neighbor, cost

    def heuristic(self, cell, goal):
        return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

    def is_free_cell(self, cell):
        x, y = cell
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False
        return bool(self.free_mask[y, x])

    def update_clyde(self):
        if self.current_x is None or self.current_y is None:
            return

        if self.game_state != 'running':
            self.publish_current_state()
            return

        if not self.path or self.path_index >= len(self.path):
            self.pick_new_destination()
            self.publish_current_state()
            return

        step_distance = self.speed_mps * self.update_period_s
        remaining_step = step_distance

        while remaining_step > 0.0 and self.path_index < len(self.path):
            target_x, target_y = self.grid_to_world(*self.path[self.path_index])
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.hypot(dx, dy)

            if distance <= 1e-9:
                self.path_index += 1
                continue

            if distance <= remaining_step:
                self.current_x = target_x
                self.current_y = target_y
                self.path_index += 1
                remaining_step -= distance
                continue

            self.current_x += (dx / distance) * remaining_step
            self.current_y += (dy / distance) * remaining_step
            remaining_step = 0.0

        if self.path_index >= len(self.path):
            self.pick_new_destination()

        self.publish_current_state()

    def publish_current_state(self):
        if self.current_x is None or self.current_y is None:
            return

        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.map_frame
        pose_msg.header.stamp = stamp
        pose_msg.pose.position.x = self.current_x
        pose_msg.pose.position.y = self.current_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'ghosts'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.pose.position.z = 0.18
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.31
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)
        self.publish_obstacle_cloud(stamp)
        self.maybe_clear_global_costmap()

    def maybe_clear_global_costmap(self):
        if self.current_x is None or self.current_y is None:
            return

        if self.global_costmap_clear_in_progress:
            return

        now = self.get_clock().now()
        moved_far_enough = True
        if self.last_global_costmap_clear_position is not None:
            last_x, last_y = self.last_global_costmap_clear_position
            moved_far_enough = (
                math.hypot(self.current_x - last_x, self.current_y - last_y) >=
                self.global_costmap_clear_move_m
            )

        timed_out = True
        if self.last_global_costmap_clear_time is not None:
            elapsed = (
                now - self.last_global_costmap_clear_time
            ).nanoseconds / 1e9
            timed_out = elapsed >= self.global_costmap_clear_period_s

        if not moved_far_enough and not timed_out:
            return

        if not self.global_costmap_clear_client.service_is_ready():
            return

        self.global_costmap_clear_in_progress = True
        future = self.global_costmap_clear_client.call_async(
            ClearEntireCostmap.Request()
        )
        future.add_done_callback(self.global_costmap_clear_callback)

    def global_costmap_clear_callback(self, future):
        self.global_costmap_clear_in_progress = False
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn(f'Failed to clear global costmap: {exc}')
            return

        self.last_global_costmap_clear_time = self.get_clock().now()
        self.last_global_costmap_clear_position = (self.current_x, self.current_y)
        self.publish_obstacle_cloud(self.get_clock().now().to_msg())

    def publish_obstacle_cloud(self, stamp):
        offsets = [
            (0.0, 0.0),
            (self.obstacle_radius_m, 0.0),
            (-self.obstacle_radius_m, 0.0),
            (0.0, self.obstacle_radius_m),
            (0.0, -self.obstacle_radius_m),
            (self.obstacle_radius_m * 0.7, self.obstacle_radius_m * 0.7),
            (-self.obstacle_radius_m * 0.7, self.obstacle_radius_m * 0.7),
            (self.obstacle_radius_m * 0.7, -self.obstacle_radius_m * 0.7),
            (-self.obstacle_radius_m * 0.7, -self.obstacle_radius_m * 0.7),
        ]

        cloud = PointCloud2()
        cloud.header.frame_id = self.map_frame
        cloud.header.stamp = stamp
        cloud.height = 1
        cloud.width = len(offsets)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(
            struct.pack(
                '<fff',
                self.current_x + dx,
                self.current_y + dy,
                self.obstacle_z_m
            )
            for dx, dy in offsets
        )
        self.obstacle_pub.publish(cloud)

    def grid_to_world(self, x, y):
        world_x = x * self.resolution + self.origin[0]
        world_y = (self.height - y) * self.resolution + self.origin[1]
        return world_x, world_y

    def world_to_grid(self, world_x, world_y):
        x = int(round((world_x - self.origin[0]) / self.resolution))
        y = int(round(self.height - ((world_y - self.origin[1]) / self.resolution)))
        x = min(max(x, 0), self.width - 1)
        y = min(max(y, 0), self.height - 1)
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = ClydeGhostNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
