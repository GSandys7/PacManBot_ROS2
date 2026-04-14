import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray

import cv2
import numpy as np
import yaml
import os

from ament_index_python.packages import get_package_share_directory


class PelletManager(Node):

    def __init__(self):
        super().__init__('pellet_manager')

        # Publisher
        self.publisher = self.create_publisher(
            MarkerArray,
            '/pellet_markers',
            10
        )

        # Load map and generate pellets
        self.world_pellets = self.load_and_generate_pellets()

        # Timer to publish markers
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(f"Generated {len(self.world_pellets)} pellets")

    # =========================
    # MAP LOADING + PROCESSING
    # =========================
    def load_and_generate_pellets(self):
        pkg_share = get_package_share_directory('pacmanbot_package')

        yaml_path = os.path.join(pkg_share, 'maps', 'map_01.yaml')

        with open(yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)

        image_path = os.path.join(pkg_share, 'maps', map_yaml['image'])
        resolution = map_yaml['resolution']
        origin = map_yaml['origin']

        # Load grayscale image
        map_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if map_img is None:
            self.get_logger().error("Failed to load map image")
            return []

        height, width = map_img.shape

        # Free space mask (white pixels)
        free_mask = map_img > 250

        # =========================
        # SAMPLING PARAMETERS
        # =========================
        spacing_m = 0.5  # distance between pellets (meters)
        step = int(spacing_m / resolution)

        pellets_pixels = []

        for y in range(0, height, step):
            for x in range(0, width, step):

                if not free_mask[y, x]:
                    continue

                # Optional: avoid walls (small neighborhood check)
                if y < 2 or y >= height-2 or x < 2 or x >= width-2:
                    continue

                if not np.all(free_mask[y-2:y+2, x-2:x+2]):
                    continue

                pellets_pixels.append((x, y))

        # Convert to world coordinates
        world_pellets = []

        for px, py in pellets_pixels:
            wx, wy = self.pixel_to_world(px, py, resolution, origin, height)
            world_pellets.append((wx, wy))

        return world_pellets

    # =========================
    # PIXEL → WORLD
    # =========================
    def pixel_to_world(self, x, y, resolution, origin, height):
        world_x = x * resolution + origin[0]
        world_y = (height - y) * resolution + origin[1]
        return world_x, world_y

    # =========================
    # MARKER PUBLISHING
    # =========================
    def publish_markers(self):
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(self.world_pellets):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "pellets"
            marker.id = i

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1

            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PelletManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()