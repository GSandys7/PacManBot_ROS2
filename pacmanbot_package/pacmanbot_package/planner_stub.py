#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class PlannerStub(Node):
    def __init__(self):
        super().__init__('planner_stub')

        self.robot_ns = '/robot_15'

        self.map_frame = 'map'

        # Matches AMCL latched pose behavior so the node receives the latest stored pose
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribes to remaining pellets from pellet_manager
        self.pellet_sub = self.create_subscription(
            MarkerArray,
            f'{self.robot_ns}/pellet_markers',
            self.pellet_callback,
            10
        )

        # Subscribes to AMCL pose for robot localization
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'{self.robot_ns}/amcl_pose',
            self.pose_callback,
            amcl_qos
        )

        # Publishes pellet IDs that should be removed by pellet_manager
        self.remove_pub = self.create_publisher(
            Int32,
            f'{self.robot_ns}/remove_pellet',
            10
        )

        # Sends goal requests to Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f'{self.robot_ns}/navigate_to_pose'
        )

        # Stores current robot pose
        self.current_pose = None

        # Stores currently known pellets as dictionaries
        self.pellets = []

        # Tracks whether a Nav2 goal is currently active
        self.goal_in_progress = False

        # Tracks the pellet currently being attempted
        self.current_target = None

        # Periodically checks whether a new pellet should be sent as a goal
        self.timer = self.create_timer(1.0, self.try_send_next_goal)

        self.get_logger().info('Planner stub started')

    # Updates the latest robot pose from AMCL
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().info(
            f'Received AMCL pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})'
        )

    # Extracts pellet IDs and positions from MarkerArray
    def pellet_callback(self, msg):
        pellets = []

        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue

            if marker.ns != 'pellets':
                continue

            pellets.append({
                'id': marker.id,
                'x': marker.pose.position.x,
                'y': marker.pose.position.y
            })

        self.pellets = pellets

    # Sends the nearest remaining pellet only when no goal is active
    def try_send_next_goal(self):
        if self.goal_in_progress:
            return

        if self.current_pose is None:
            self.get_logger().info('Waiting for AMCL pose...')
            return

        if not self.pellets:
            return

        nearest = self.find_nearest_pellet()
        if nearest is None:
            return

        self.send_nav_goal(nearest)

    # Finds the nearest pellet by Euclidean distance from the current robot pose
    def find_nearest_pellet(self):
        rx = self.current_pose.position.x
        ry = self.current_pose.position.y

        best = None
        best_dist = float('inf')

        for pellet in self.pellets:
            dist = math.hypot(pellet['x'] - rx, pellet['y'] - ry)

            if dist < best_dist:
                best_dist = dist
                best = pellet

        if best is not None:
            self.get_logger().info(
                f"Nearest pellet {best['id']} at "
                f"({best['x']:.2f}, {best['y']:.2f}), dist={best_dist:.2f}"
            )

        return best

    # Sends the selected pellet to Nav2 as a NavigateToPose goal
    def send_nav_goal(self, pellet):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = pellet['x']
        goal_msg.pose.pose.position.y = pellet['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.current_target = pellet
        self.goal_in_progress = True

        self.get_logger().info(
            f"Sending pellet {pellet['id']} as goal: "
            f"({pellet['x']:.2f}, {pellet['y']:.2f})"
        )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # Handles Nav2 accepting or rejecting the goal
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')

            if self.current_target is not None:
                self.publish_remove_pellet(self.current_target['id'])

            self.goal_in_progress = False
            self.current_target = None
            return

        self.get_logger().info('Goal accepted by Nav2')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    # Removes pellets on both success and failure so unreachable pellets are discarded
    def goal_result_callback(self, future):
        result_wrapper = future.result()
        status = result_wrapper.status

        if self.current_target is None:
            self.goal_in_progress = False
            return

        pellet_id = self.current_target['id']

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Pellet {pellet_id} reached successfully')
        else:
            self.get_logger().warn(
                f'Pellet {pellet_id} unreachable or failed, status={status}'
            )

        self.publish_remove_pellet(pellet_id)

        self.goal_in_progress = False
        self.current_target = None

    # Logs distance remaining during navigation
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} m'
        )

    # Publishes a pellet ID removal request to pellet_manager
    def publish_remove_pellet(self, pellet_id):
        msg = Int32()
        msg.data = pellet_id
        self.remove_pub.publish(msg)
        self.get_logger().info(f'Published remove request for pellet {pellet_id}')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerStub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()