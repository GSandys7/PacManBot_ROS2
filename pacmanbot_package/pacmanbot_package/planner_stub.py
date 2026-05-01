#!/usr/bin/env python3

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class PlannerStub(Node):
    def __init__(self):
        super().__init__('planner_stub')

        self.robot_ns = '/robot_15'

        self.map_frame = 'map'

        self.clyde_death_radius_m = 0.25
        self.ghost_risk_radius_m = 2.0
        self.ghost_risk_weight = 3.0
        self.replan_score_margin = 0.3
        self.escape_warning_radius_m = 3.0
        self.escape_emergency_radius_m = 2.0
        self.escape_approach_speed_mps = 0.03
        self.ghost_direction_penalty = 5.0
        self.pellet_pickup_radius_m = 0.25

        # Matches AMCL latched pose behavior so the node receives the latest stored pose
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        latched_qos = QoSProfile(
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

        self.clyde_sub = self.create_subscription(
            PoseStamped,
            f'{self.robot_ns}/clyde_pose',
            self.clyde_pose_callback,
            10
        )
        self.game_command_sub = self.create_subscription(
            String,
            f'{self.robot_ns}/game_command',
            self.game_command_callback,
            latched_qos
        )

        # Publishes pellet IDs that should be removed by pellet_manager
        self.remove_pub = self.create_publisher(
            Int32,
            f'{self.robot_ns}/remove_pellet',
            10
        )
        self.sound_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_sound',
            10
        )
        self.event_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_event',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_status',
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
        self.home_pose = None
        self.clyde_pose = None
        self.previous_clyde_distance = None
        self.previous_clyde_distance_time = None
        self.clyde_closing_speed_mps = 0.0

        # Stores currently known pellets as dictionaries
        self.pellets = []
        self.received_pellet_markers = False
        self.collected_pellet_ids = set()

        # Tracks whether a Nav2 goal is currently active
        self.autonomy_enabled = False
        self.goal_in_progress = False
        self.active_goal_type = None
        self.last_goal_rejected = False

        # Tracks the pellet currently being attempted
        self.current_target = None
        self.current_goal_handle = None
        self.active_goal_id = 0
        self.death_sequence_active = False
        self.returning_home = False
        self.all_pellets_collected = False

        # Periodically checks whether a new pellet should be sent as a goal
        self.timer = self.create_timer(0.5, self.try_send_next_goal)

        self.get_logger().info('Planner stub started and waiting for game start')

    # Updates the latest robot pose from AMCL
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.home_pose is None:
            self.home_pose = copy.deepcopy(msg.pose.pose)
            self.get_logger().info(
                f'Saved home pose from first AMCL pose: '
                f'({self.home_pose.position.x:.2f}, {self.home_pose.position.y:.2f})'
            )

        self.check_clyde_collision()
        self.collect_nearby_pellets()
        self.get_logger().info(
            f'Received AMCL pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})'
        )

    def clyde_pose_callback(self, msg):
        self.clyde_pose = msg.pose
        self.update_clyde_closing_speed()
        self.check_clyde_collision()

    def game_command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received game command: {command}')

        if command == 'enable':
            self.autonomy_enabled = True
            self.death_sequence_active = False
            self.returning_home = False
            self.all_pellets_collected = False

        elif command == 'disable':
            self.autonomy_enabled = False
            self.cancel_current_goal()
            if not self.returning_home:
                self.clear_active_goal_state()

        elif command == 'reset_round':
            self.autonomy_enabled = False
            self.cancel_current_goal()
            self.clear_active_goal_state()
            self.death_sequence_active = False
            self.returning_home = False
            self.all_pellets_collected = False
            self.received_pellet_markers = False
            self.collected_pellet_ids.clear()
            self.previous_clyde_distance = None
            self.previous_clyde_distance_time = None
            self.clyde_closing_speed_mps = 0.0

        elif command == 'return_home':
            self.autonomy_enabled = False
            self.death_sequence_active = False
            if not self.send_home_goal():
                self.publish_game_status('home_failed')

    # Extracts pellet IDs and positions from MarkerArray
    def pellet_callback(self, msg):
        pellets = []
        self.received_pellet_markers = True

        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue

            if marker.ns != 'pellets':
                continue

            if marker.id in self.collected_pellet_ids:
                continue

            pellets.append({
                'id': marker.id,
                'x': marker.pose.position.x,
                'y': marker.pose.position.y
            })

        self.pellets = pellets

    # Sends or replans pellet goals only when no conflicting goal is active
    def try_send_next_goal(self):
        if not self.autonomy_enabled:
            return

        if self.death_sequence_active or self.returning_home:
            return

        self.check_clyde_collision()
        if self.death_sequence_active:
            return

        self.collect_nearby_pellets()

        if self.goal_in_progress and self.active_goal_type == 'pellet':
            self.try_replan_current_goal()
            return

        if self.goal_in_progress:
            return

        if self.current_pose is None:
            self.get_logger().info('Waiting for AMCL pose...')
            return

        if not self.received_pellet_markers:
            self.get_logger().info('Waiting for pellet markers...')
            return

        if not self.pellets:
            if not self.all_pellets_collected:
                self.get_logger().info('All pellets collected')
                self.all_pellets_collected = True
                self.autonomy_enabled = False
                self.publish_game_status('game_complete')
            return

        best = self.find_best_pellet()
        if best is None:
            return

        self.send_pellet_goal(best)

    # Finds the best pellet by distance, Clyde risk, and threat direction.
    def find_best_pellet(self):
        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        threat_active = self.is_clyde_threatening()

        best = None
        best_score = float('inf')
        fallback = None
        fallback_score = float('inf')
        safe_best = None
        safe_best_score = float('inf')

        for pellet in self.pellets:
            if pellet['id'] in self.collected_pellet_ids:
                continue

            score = self.score_pellet(pellet, rx, ry, threat_active)
            clyde_distance = self.distance_to_clyde(pellet['x'], pellet['y'])
            safe_direction = self.is_pellet_safe_direction(pellet, rx, ry)

            if score < fallback_score:
                fallback_score = score
                fallback = pellet

            if (
                clyde_distance is not None and
                clyde_distance <= self.clyde_death_radius_m
            ):
                continue

            if score < best_score:
                best_score = score
                best = pellet

            if threat_active and safe_direction and score < safe_best_score:
                safe_best_score = score
                safe_best = pellet

        if safe_best is not None:
            best = safe_best
            best_score = safe_best_score

        if best is None:
            best = fallback
            best_score = fallback_score

        if best is not None:
            mode = 'threat-aware' if threat_active else 'best'
            self.get_logger().info(
                f"{mode} pellet {best['id']} at "
                f"({best['x']:.2f}, {best['y']:.2f}), score={best_score:.2f}"
            )

        return best

    def score_pellet(self, pellet, rx=None, ry=None, threat_active=None):
        if rx is None:
            rx = self.current_pose.position.x
        if ry is None:
            ry = self.current_pose.position.y
        if threat_active is None:
            threat_active = self.is_clyde_threatening()

        robot_distance = math.hypot(pellet['x'] - rx, pellet['y'] - ry)
        clyde_distance = self.distance_to_clyde(pellet['x'], pellet['y'])
        risk_penalty = 0.0

        if clyde_distance is not None:
            risk_penalty = max(
                0.0,
                self.ghost_risk_radius_m - clyde_distance
            ) * self.ghost_risk_weight

        direction_penalty = 0.0
        if threat_active and not self.is_pellet_safe_direction(pellet, rx, ry):
            alignment = self.pellet_ghost_alignment(pellet, rx, ry)
            direction_penalty = self.ghost_direction_penalty * (1.0 + alignment)

        return robot_distance + risk_penalty + direction_penalty

    def distance_to_clyde(self, x, y):
        if self.clyde_pose is None:
            return None
        return math.hypot(
            x - self.clyde_pose.position.x,
            y - self.clyde_pose.position.y
        )

    def update_clyde_closing_speed(self):
        if self.current_pose is None or self.clyde_pose is None:
            return

        distance = self.distance_to_clyde(
            self.current_pose.position.x,
            self.current_pose.position.y
        )
        now = self.get_clock().now()

        if (
            self.previous_clyde_distance is not None and
            self.previous_clyde_distance_time is not None
        ):
            elapsed = (
                now - self.previous_clyde_distance_time
            ).nanoseconds / 1e9
            if elapsed > 0.0:
                self.clyde_closing_speed_mps = (
                    self.previous_clyde_distance - distance
                ) / elapsed

        self.previous_clyde_distance = distance
        self.previous_clyde_distance_time = now

    def is_clyde_threatening(self):
        if (
            self.current_pose is None or
            self.clyde_pose is None or
            self.death_sequence_active or
            self.returning_home
        ):
            return False

        distance = self.distance_to_clyde(
            self.current_pose.position.x,
            self.current_pose.position.y
        )
        if distance is None or distance <= self.clyde_death_radius_m:
            return False

        if distance <= self.escape_emergency_radius_m:
            return True

        return (
            distance <= self.escape_warning_radius_m and
            self.clyde_closing_speed_mps >= self.escape_approach_speed_mps
        )

    def pellet_ghost_alignment(self, pellet, rx=None, ry=None):
        if self.clyde_pose is None:
            return 0.0
        if rx is None:
            rx = self.current_pose.position.x
        if ry is None:
            ry = self.current_pose.position.y

        ghost_x = self.clyde_pose.position.x - rx
        ghost_y = self.clyde_pose.position.y - ry
        pellet_x = pellet['x'] - rx
        pellet_y = pellet['y'] - ry
        ghost_distance = math.hypot(ghost_x, ghost_y)
        pellet_distance = math.hypot(pellet_x, pellet_y)
        if ghost_distance <= 1e-9 or pellet_distance <= 1e-9:
            return 0.0

        return (
            (ghost_x / ghost_distance) * (pellet_x / pellet_distance) +
            (ghost_y / ghost_distance) * (pellet_y / pellet_distance)
        )

    def is_pellet_safe_direction(self, pellet, rx=None, ry=None):
        if self.clyde_pose is None:
            return True
        if rx is None:
            rx = self.current_pose.position.x
        if ry is None:
            ry = self.current_pose.position.y

        current_clyde_distance = self.distance_to_clyde(rx, ry)
        pellet_clyde_distance = self.distance_to_clyde(pellet['x'], pellet['y'])
        if (
            current_clyde_distance is not None and
            pellet_clyde_distance is not None and
            pellet_clyde_distance > current_clyde_distance
        ):
            return True

        return self.pellet_ghost_alignment(pellet, rx, ry) <= 0.0

    def try_replan_current_goal(self):
        if self.current_target is None or self.current_pose is None:
            return

        best = self.find_best_pellet()
        if best is None or best['id'] == self.current_target['id']:
            return

        threat_active = self.is_clyde_threatening()
        current_score = self.score_pellet(
            self.current_target,
            threat_active=threat_active
        )
        best_score = self.score_pellet(best, threat_active=threat_active)
        if best_score + self.replan_score_margin >= current_score:
            return

        self.get_logger().info(
            f"Replanning from pellet {self.current_target['id']} "
            f"(score={current_score:.2f}) to pellet {best['id']} "
            f"(score={best_score:.2f})"
        )
        self.cancel_current_goal()
        self.goal_in_progress = False
        self.active_goal_type = None
        self.current_target = None
        self.current_goal_handle = None
        self.active_goal_id += 1
        self.send_pellet_goal(best)

    # Sends the selected pellet to Nav2 as a NavigateToPose goal
    def send_pellet_goal(self, pellet):
        self.send_nav_goal(pellet['x'], pellet['y'], 'pellet', pellet)

    def send_home_goal(self):
        if self.home_pose is None:
            self.get_logger().warn('Cannot return home; home pose not saved yet')
            return False

        self.returning_home = True
        sent = self.send_nav_goal(
            self.home_pose.position.x,
            self.home_pose.position.y,
            'home',
            None,
            self.home_pose
        )
        if not sent:
            self.returning_home = False
        return sent

    def send_nav_goal(self, x, y, goal_type, pellet=None, pose=None):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        if pose is not None:
            goal_msg.pose.pose.orientation = pose.orientation
        else:
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = 0.0
            goal_msg.pose.pose.orientation.w = 1.0

        self.current_target = pellet
        self.active_goal_type = goal_type
        self.goal_in_progress = True
        self.last_goal_rejected = False
        self.active_goal_id += 1
        goal_id = self.active_goal_id

        if goal_type == 'pellet':
            self.get_logger().info(
                f"Sending pellet {pellet['id']} as goal: "
                f"({x:.2f}, {y:.2f})"
            )
        elif goal_type == 'home':
            self.get_logger().info(f'Sending return-home goal: ({x:.2f}, {y:.2f})')
        else:
            self.get_logger().info(
                f'Sending {goal_type} goal: ({x:.2f}, {y:.2f})'
            )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(
            lambda future, sent_goal_id=goal_id:
            self.goal_response_callback(future, sent_goal_id)
        )
        return True

    # Handles Nav2 accepting or rejecting the goal
    def goal_response_callback(self, future, goal_id):
        if goal_id != self.active_goal_id:
            self.get_logger().info('Ignoring stale Nav2 goal response')
            return

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(
                'Goal rejected by Nav2; keeping target available and retrying later'
            )

            self.goal_in_progress = False
            self.last_goal_rejected = True
            if self.active_goal_type == 'home':
                self.returning_home = False
                self.publish_game_status('home_failed')
            self.active_goal_type = None
            self.current_target = None
            self.current_goal_handle = None
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, sent_goal_id=goal_id:
            self.goal_result_callback(future, sent_goal_id)
        )

    # Removes pellets on both success and failure so unreachable pellets are discarded
    def goal_result_callback(self, future, goal_id):
        if goal_id != self.active_goal_id:
            self.get_logger().info('Ignoring stale Nav2 goal result')
            return

        result_wrapper = future.result()
        status = result_wrapper.status

        if self.active_goal_type == 'home':
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Returned home successfully')
                self.publish_game_status('home_reached')
            else:
                self.get_logger().warn(f'Return-home goal failed, status={status}')
                self.publish_game_status('home_failed')

            self.goal_in_progress = False
            self.active_goal_type = None
            self.current_goal_handle = None
            self.current_target = None
            self.returning_home = False
            return

        if self.current_target is None:
            self.goal_in_progress = False
            self.active_goal_type = None
            self.current_goal_handle = None
            return

        pellet_id = self.current_target['id']

        if status == GoalStatus.STATUS_SUCCEEDED:
            if pellet_id not in self.collected_pellet_ids:
                self.collect_pellet(self.current_target, 'nav2 result')
            else:
                self.get_logger().info(
                    f'Pellet {pellet_id} was already collected by proximity'
                )
        else:
            self.get_logger().warn(
                f'Pellet {pellet_id} unreachable or failed, status={status}'
            )
            if pellet_id not in self.collected_pellet_ids:
                self.publish_remove_pellet(pellet_id)

        self.goal_in_progress = False
        self.active_goal_type = None
        self.current_target = None
        self.current_goal_handle = None
        self.active_goal_id += 1

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

    def collect_nearby_pellets(self):
        if self.current_pose is None or not self.pellets:
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        collected_active_target = False

        for pellet in list(self.pellets):
            if pellet['id'] in self.collected_pellet_ids:
                continue

            distance = math.hypot(pellet['x'] - rx, pellet['y'] - ry)
            if distance > self.pellet_pickup_radius_m:
                continue

            self.collect_pellet(pellet, 'proximity')
            if (
                self.current_target is not None and
                pellet['id'] == self.current_target['id']
            ):
                collected_active_target = True

        if collected_active_target:
            self.cancel_current_goal()
            self.goal_in_progress = False
            self.active_goal_type = None
            self.current_target = None
            self.current_goal_handle = None
            self.active_goal_id += 1

    def collect_pellet(self, pellet, reason):
        pellet_id = pellet['id']
        if pellet_id in self.collected_pellet_ids:
            return

        self.collected_pellet_ids.add(pellet_id)
        self.pellets = [
            remaining
            for remaining in self.pellets
            if remaining['id'] != pellet_id
        ]
        self.get_logger().info(
            f"Pellet {pellet_id} collected by {reason}"
        )
        self.publish_game_status(f'pellet_collected:{pellet_id}')
        self.publish_remove_pellet(pellet_id)

    def check_clyde_collision(self):
        if (
            not self.autonomy_enabled or
            self.current_pose is None or
            self.clyde_pose is None or
            self.death_sequence_active or
            self.returning_home
        ):
            return

        distance = math.hypot(
            self.current_pose.position.x - self.clyde_pose.position.x,
            self.current_pose.position.y - self.clyde_pose.position.y
        )
        if distance > self.clyde_death_radius_m:
            return

        self.get_logger().warn(
            f'Clyde killed Pac-Man at distance {distance:.2f} m'
        )
        self.death_sequence_active = True
        self.autonomy_enabled = False
        self.cancel_current_goal()
        self.goal_in_progress = False
        self.active_goal_type = None
        self.current_target = None
        self.current_goal_handle = None
        self.active_goal_id += 1

        event_msg = String()
        event_msg.data = 'clyde killed'
        self.event_pub.publish(event_msg)
        self.publish_game_status('death_detected')

    def cancel_current_goal(self):
        if self.current_goal_handle is None:
            return

        try:
            self.current_goal_handle.cancel_goal_async()
            self.get_logger().info('Requested cancellation of current Nav2 goal')
        except Exception as exc:
            self.get_logger().warn(f'Failed to cancel current Nav2 goal: {exc}')

    def clear_active_goal_state(self):
        self.goal_in_progress = False
        self.active_goal_type = None
        self.current_target = None
        self.current_goal_handle = None
        self.active_goal_id += 1

    def publish_game_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Published game status: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = PlannerStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
