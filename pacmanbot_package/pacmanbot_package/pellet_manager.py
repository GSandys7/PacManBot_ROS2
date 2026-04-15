#!/usr/bin/env python3

import os
import cv2
import yaml
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


class PelletManager(Node):
    def __init__(self):
        super().__init__('pellet_manager')

        self.robot_ns = '/robot_15'

        # Defines TF frame for pellet markers
        self.map_frame = 'map'

        # Publishes remaining pellets as markers for RViz and planner consumption
        self.marker_pub = self.create_publisher(
            MarkerArray,
            f'{self.robot_ns}/pellet_markers',
            10
        )

        # Subscribes to pellet removal requests by pellet ID
        self.remove_sub = self.create_subscription(
            Int32,
            f'{self.robot_ns}/remove_pellet',
            self.remove_pellet_callback,
            10
        )

        # Loads map and generates pellet list
        self.world_pellets = self.load_and_generate_pellets()

        # Republishes pellet markers periodically
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(
            f'Generated {len(self.world_pellets)} pellets in frame "{self.map_frame}"'
        )

    # Loads occupancy map and samples valid free-space pellet positions
    def load_and_generate_pellets(self):
        pkg_share = get_package_share_directory('pacmanbot_package')
        yaml_path = os.path.join(pkg_share, 'maps', 'map_01.yaml')

        with open(yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)

        image_path = os.path.join(pkg_share, 'maps', map_yaml['image'])
        resolution = map_yaml['resolution']
        origin = map_yaml['origin']

        map_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if map_img is None:
            self.get_logger().error('Failed to load map image')
            return []

        height, width = map_img.shape
        free_mask = map_img > 250

        # Controls distance between pellets in meters
        spacing_m = 3.0
        step = max(1, int(spacing_m / resolution))

        pellets_pixels = []

        for y in range(0, height, step):
            for x in range(0, width, step):
                # Skips occupied cells
                if not free_mask[y, x]:
                    continue

                # Skips boundary cells where neighborhood checks would be unsafe
                if y < 2 or y >= height - 2 or x < 2 or x >= width - 2:
                    continue

                # Skips cells too close to walls
                if not np.all(free_mask[y - 2:y + 2, x - 2:x + 2]):
                    continue

                pellets_pixels.append((x, y))

        world_pellets = []
        pellet_id = 0

        for px, py in pellets_pixels:
            wx, wy = self.pixel_to_world(px, py, resolution, origin, height)
            world_pellets.append({
                'id': pellet_id,
                'x': wx,
                'y': wy
            })
            pellet_id += 1

        return world_pellets

    # Converts map pixel coordinates into world coordinates in the map frame
    def pixel_to_world(self, x, y, resolution, origin, height):
        world_x = x * resolution + origin[0]
        world_y = (height - y) * resolution + origin[1]
        return world_x, world_y

    # Removes a pellet from the source-of-truth list by pellet ID
    def remove_pellet_callback(self, msg):
        pellet_id = msg.data

        before = len(self.world_pellets)
        self.world_pellets = [
            pellet for pellet in self.world_pellets
            if pellet['id'] != pellet_id
        ]
        after = len(self.world_pellets)

        if after < before:
            self.get_logger().info(f'Removed pellet {pellet_id}')
        else:
            self.get_logger().warn(f'Pellet {pellet_id} not found')

    # Publishes all remaining pellets as a MarkerArray with stable IDs
    def publish_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Clears old markers so removed pellets disappear in RViz
        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        # Publishes one marker per remaining pellet
        for pellet in self.world_pellets:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp

            marker.ns = 'pellets'
            marker.id = pellet['id']

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = pellet['x']
            marker.pose.position.y = pellet['y']
            marker.pose.position.z = 0.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PelletManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()