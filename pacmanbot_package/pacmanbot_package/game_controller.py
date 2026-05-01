#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')

        self.robot_ns = '/robot_15'
        self.start_theatrical_duration_s = 8.0
        self.death_feedback_duration_s = 5.0

        self.state = 'menu'
        self.final_result = ''
        self.pellets_spawned = 0
        self.pellets_collected = 0
        self.round_number = 1
        self.round_score = 0
        self.total_score = 0
        self.score = 0
        self.base_clyde_speed_mps = 0.25
        self.round_speed_step_mps = 0.05
        self.awaiting_new_spawn = False
        self.collected_pellet_ids = set()
        self.pending_start_after_home = False
        self.pending_completed_round = False

        self.start_timer = None
        self.death_timer = None
        self.completion_reset_timer = None
        self.nav_wait_timer = None

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.state_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_state',
            latched_qos
        )
        self.command_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_command',
            latched_qos
        )
        self.result_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_result',
            latched_qos
        )
        self.spawned_pub = self.create_publisher(
            Int32,
            f'{self.robot_ns}/game/pellets_spawned',
            latched_qos
        )
        self.collected_pub = self.create_publisher(
            Int32,
            f'{self.robot_ns}/game/pellets_collected',
            latched_qos
        )
        self.score_pub = self.create_publisher(
            Int32,
            f'{self.robot_ns}/game/score',
            latched_qos
        )
        self.round_pub = self.create_publisher(
            Int32,
            f'{self.robot_ns}/game/round',
            latched_qos
        )
        self.clyde_speed_pub = self.create_publisher(
            Float32,
            f'{self.robot_ns}/clyde/speed',
            latched_qos
        )
        self.event_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_event',
            10
        )

        self.status_sub = self.create_subscription(
            String,
            f'{self.robot_ns}/game_status',
            self.status_callback,
            10
        )
        self.marker_sub = self.create_subscription(
            MarkerArray,
            f'{self.robot_ns}/pellet_markers',
            self.marker_callback,
            10
        )
        self.base_clyde_speed_sub = self.create_subscription(
            Float32,
            f'{self.robot_ns}/game/base_clyde_speed',
            self.base_clyde_speed_callback,
            latched_qos
        )

        self.start_srv = self.create_service(
            Trigger,
            f'{self.robot_ns}/game/start',
            self.start_callback
        )
        self.reset_srv = self.create_service(
            Trigger,
            f'{self.robot_ns}/game/reset',
            self.reset_callback
        )
        self.reset_pellets_client = self.create_client(
            Trigger,
            f'{self.robot_ns}/reset_pellets'
        )
        self.reset_clyde_client = self.create_client(
            Trigger,
            f'{self.robot_ns}/reset_clyde'
        )
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f'{self.robot_ns}/navigate_to_pose'
        )

        self.publish_command('disable')
        self.publish_all()
        self.get_logger().info('Game controller ready')

    def start_callback(self, request, response):
        del request

        if self.state != 'menu':
            response.success = False
            response.message = f'Cannot start from state "{self.state}"'
            return response

        self.cancel_timer('completion_reset_timer')
        self.cancel_timer('nav_wait_timer')
        self.reset_round(
            clear_result=True,
            reset_game=False,
            publish_menu=False
        )
        self.state = 'returning_home'
        self.pending_start_after_home = True
        self.pending_completed_round = False
        self.publish_command('return_home')
        self.publish_all()

        response.success = True
        response.message = 'Returning home before starting PacMan round'
        return response

    def reset_callback(self, request, response):
        del request

        self.cancel_timer('start_timer')
        self.cancel_timer('death_timer')
        self.cancel_timer('completion_reset_timer')
        self.cancel_timer('nav_wait_timer')
        self.pending_start_after_home = False
        self.pending_completed_round = False
        self.reset_round(clear_result=True, reset_game=True, publish_menu=True)

        response.success = True
        response.message = 'PacMan round reset'
        return response

    def status_callback(self, msg):
        status = msg.data.strip().lower()
        self.get_logger().info(f'Received game status: {status}')

        if status == 'pellet_collected' or status.startswith('pellet_collected:'):
            if self.state != 'running':
                return

            pellet_id = self.parse_pellet_id(status)
            if pellet_id is not None:
                if pellet_id in self.collected_pellet_ids:
                    self.get_logger().info(
                        f'Ignoring duplicate collection for pellet {pellet_id}'
                    )
                    return
                self.collected_pellet_ids.add(pellet_id)

            self.pellets_collected += 1
            self.round_score += 1
            self.total_score += 1
            self.score = self.total_score
            self.publish_all()
            self.publish_event('pellet')
            if (
                self.pellets_spawned > 0 and
                self.pellets_collected >= self.pellets_spawned
            ):
                self.handle_completion()

        elif status == 'death_detected':
            self.handle_death()

        elif status in ['home_reached', 'home_failed']:
            if self.state != 'returning_home':
                return

            if self.pending_start_after_home:
                self.pending_start_after_home = False
                if status == 'home_reached':
                    self.begin_start_sequence()
                else:
                    self.final_result = 'Could not return home before start'
                    self.reset_round(
                        clear_result=False,
                        reset_game=True,
                        publish_menu=True
                    )
            elif self.pending_completed_round:
                self.prepare_next_round(status)
            else:
                self.reset_round(
                    clear_result=False,
                    reset_game=True,
                    publish_menu=True
                )

        elif status == 'game_complete':
            self.handle_completion()

    def marker_callback(self, msg):
        remaining = sum(
            1
            for marker in msg.markers
            if marker.action == Marker.ADD and marker.ns == 'pellets'
        )

        if self.awaiting_new_spawn:
            self.pellets_spawned = remaining
            self.awaiting_new_spawn = False
        else:
            self.pellets_spawned = max(
                self.pellets_spawned,
                remaining + self.pellets_collected
            )

        self.publish_counts()

    def base_clyde_speed_callback(self, msg):
        speed = float(msg.data)
        if speed <= 0.0:
            self.get_logger().warn(f'Ignoring invalid base Clyde speed: {speed:.2f}')
            return

        self.base_clyde_speed_mps = speed
        self.get_logger().info(
            f'Base Clyde speed set to {self.base_clyde_speed_mps:.2f} m/s'
        )
        if self.state == 'menu':
            self.publish_round_clyde_speed()

    def parse_pellet_id(self, status):
        parts = status.split(':', 1)
        if len(parts) != 2:
            return None

        try:
            return int(parts[1])
        except ValueError:
            self.get_logger().warn(f'Invalid pellet collection status: {status}')
            return None

    def begin_start_sequence(self):
        self.state = 'starting'
        self.publish_event('start theatrical')
        self.publish_command('disable')
        self.publish_all()
        self.start_timer = self.create_timer(
            self.start_theatrical_duration_s,
            self.finish_start_sequence
        )

    def finish_start_sequence(self):
        self.cancel_timer('start_timer')

        if self.state != 'starting':
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn(
                'Start sequence finished, but NavigateToPose action server is not ready yet; waiting'
            )
            self.nav_wait_timer = self.create_timer(1.0, self.enable_when_nav_ready)
            return

        self.enable_round()

    def enable_when_nav_ready(self):
        if self.state != 'starting':
            self.cancel_timer('nav_wait_timer')
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(
                'Waiting for NavigateToPose action server before enabling game'
            )
            return

        self.cancel_timer('nav_wait_timer')
        self.enable_round()

    def enable_round(self):
        if self.state != 'starting':
            return

        self.state = 'running'
        self.publish_round_clyde_speed()
        self.publish_command('enable')
        self.publish_all()
        self.get_logger().info(
            f'NavigateToPose is ready; PacMan round {self.round_number} is running'
        )

    def handle_death(self):
        if self.state not in ['starting', 'running']:
            return

        self.cancel_timer('start_timer')
        self.cancel_timer('nav_wait_timer')
        self.state = 'dying'
        self.pending_start_after_home = False
        self.pending_completed_round = False
        self.final_result = f'Death - Score: {self.score}'
        self.publish_command('disable')
        self.publish_all()
        self.death_timer = self.create_timer(
            self.death_feedback_duration_s,
            self.command_return_home
        )

    def command_return_home(self):
        self.cancel_timer('death_timer')

        if self.state != 'dying':
            return

        self.state = 'returning_home'
        self.pending_start_after_home = False
        self.publish_command('return_home')
        self.publish_all()

    def handle_completion(self):
        if self.state not in ['running', 'starting']:
            return

        self.cancel_timer('start_timer')
        self.cancel_timer('nav_wait_timer')
        self.state = 'complete'
        self.pending_start_after_home = False
        self.pending_completed_round = True
        self.score = self.total_score
        self.final_result = f'Complete - Score: {self.score}'
        self.publish_command('disable')
        self.publish_event('win')
        self.publish_command('return_home')
        self.publish_all()
        self.state = 'returning_home'
        self.publish_all()

    def reset_completed_round(self):
        self.cancel_timer('completion_reset_timer')
        self.prepare_next_round('home_reached')

    def prepare_next_round(self, home_status: str):
        self.pending_completed_round = False
        self.round_number += 1
        self.reset_round(
            clear_result=False,
            reset_game=False,
            publish_menu=True
        )
        if home_status == 'home_failed':
            self.final_result = (
                f'Complete - Score: {self.score}; return home failed'
            )
        self.publish_all()

    def reset_round(self, clear_result: bool, reset_game: bool, publish_menu: bool):
        self.publish_command('disable')
        self.publish_command('reset_round')
        self.publish_event('reset')
        self.call_trigger(self.reset_pellets_client, 'reset_pellets')
        self.call_trigger(self.reset_clyde_client, 'reset_clyde')

        if reset_game:
            self.round_number = 1
            self.round_score = 0
            self.total_score = 0
            self.score = 0

        self.pellets_collected = 0
        self.round_score = 0
        self.score = self.total_score
        self.pellets_spawned = 0
        self.awaiting_new_spawn = True
        self.collected_pellet_ids.clear()

        if clear_result:
            self.final_result = ''

        if publish_menu:
            self.state = 'menu'

        self.publish_round_clyde_speed()
        self.publish_all()

    def current_round_clyde_speed(self):
        return (
            self.base_clyde_speed_mps +
            self.round_speed_step_mps * (self.round_number - 1)
        )

    def publish_round_clyde_speed(self):
        msg = Float32()
        msg.data = float(self.current_round_clyde_speed())
        self.clyde_speed_pub.publish(msg)
        self.get_logger().info(
            f'Published Clyde speed for round {self.round_number}: '
            f'{msg.data:.2f} m/s'
        )

    def call_trigger(self, client, service_name):
        if not client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(f'Service {service_name} is not ready')
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda future, name=service_name:
            self.trigger_done_callback(future, name)
        )

    def trigger_done_callback(self, future, service_name):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'{service_name} call failed: {exc}')
            return

        if response is not None and not response.success:
            self.get_logger().warn(f'{service_name} failed: {response.message}')

    def publish_all(self):
        self.publish_string(self.state_pub, self.state)
        self.publish_string(self.result_pub, self.final_result)
        self.publish_counts()

    def publish_counts(self):
        self.publish_int(self.spawned_pub, self.pellets_spawned)
        self.publish_int(self.collected_pub, self.pellets_collected)
        self.publish_int(self.score_pub, self.score)
        self.publish_int(self.round_pub, self.round_number)

    def publish_command(self, command):
        self.publish_string(self.command_pub, command)
        self.get_logger().info(f'Published game command: {command}')

    def publish_event(self, event):
        self.publish_string(self.event_pub, event)
        self.get_logger().info(f'Published game event: {event}')

    def publish_string(self, publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def publish_int(self, publisher, value):
        msg = Int32()
        msg.data = int(value)
        publisher.publish(msg)

    def cancel_timer(self, attr_name):
        timer = getattr(self, attr_name)
        if timer is not None:
            timer.cancel()
            setattr(self, attr_name, None)


def main(args=None):
    rclpy.init(args=args)
    node = GameController()
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
