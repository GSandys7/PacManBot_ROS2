#!/usr/bin/env python3

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GameEventMapper(Node):
    def __init__(self):
        super().__init__('game_event_mapper')

        # Namespace configuration
        self.robot_ns = '/robot_15'

        # Shared motion timer configuration
        self.motion_publish_period_s = 0.1

        # Death shake configuration
        self.death_motion_duration_s = 5.0
        self.shake_angular_z = 1.5
        self.shake_direction_interval_s = 0.25

        # Theatrical start spin configuration
        self.start_theatrical_duration_s = 8.0
        self.start_spin_angular_z = 1.5
        self.start_light_restart_period_s = 3.0

        # Tracks active motion state
        self.motion_active = False
        self.motion_time_remaining = 0.0
        self.active_motion_mode = None

        # Tracks shake state
        self.current_shake_direction = 1.0
        self.time_until_direction_flip = self.shake_direction_interval_s

        # Tracks theatrical start light looping
        self.time_until_light_restart = self.start_light_restart_period_s

        # Listens for high-level game events
        self.sub = self.create_subscription(
            String,
            f'{self.robot_ns}/game_event',
            self.event_callback,
            10
        )

        # Publishes mapped light commands
        self.light_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_light',
            10
        )

        # Publishes mapped sound commands
        self.sound_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/game_sound',
            10
        )

        # Publishes TwistStamped commands for motion behaviors
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            f'{self.robot_ns}/cmd_vel',
            10
        )

        # Timer used to drive active motion behaviors
        self.motion_timer = self.create_timer(
            self.motion_publish_period_s,
            self.motion_timer_callback
        )
        self.motion_timer.cancel()

        self.get_logger().info('Game Event Mapper Node Started')

    def event_callback(self, msg: String):
        event = msg.data.strip().lower()
        self.get_logger().info(f'Received event: {event}')

        if event == 'start':
            self.send_light('start')
            self.send_sound('start')

        elif event == 'start theatrical':
            self.handle_start_theatrical_event()

        elif event == 'pellet':
            self.send_light('pellet')
            self.send_sound('pellet')

        elif event in ['power pellet', 'powerup']:
            self.send_light('power pellet')
            self.send_sound('power pellet')

        elif event == 'blinky caught':
            self.send_light('blinky caught')
            self.send_sound('ghost')

        elif event == 'pinky caught':
            self.send_light('pinky caught')
            self.send_sound('ghost')

        elif event == 'inky caught':
            self.send_light('inky caught')
            self.send_sound('ghost')

        elif event == 'clyde caught':
            self.send_light('clyde caught')
            self.send_sound('ghost')

        elif event in ['blinky killed', 'pinky killed', 'inky killed', 'clyde killed']:
            self.handle_death_event(event)

        elif event in ['killed', 'death']:
            self.handle_death_event()

        elif event == 'game over':
            self.send_light('game over')
            self.send_sound('death')
            self.start_motion('death', self.death_motion_duration_s)

        elif event == 'win':
            self.send_light('win')
            self.send_sound('win')

        elif event == 'reset':
            self.stop_motion()

        else:
            self.get_logger().warning(f'Unknown event: {event}')

    def handle_death_event(self, light_command='game over'):
        self.get_logger().info('Handling death event')
        self.send_light(light_command)
        self.send_sound('death')
        self.start_motion('death', self.death_motion_duration_s)

    def handle_start_theatrical_event(self):
        self.get_logger().info('Handling theatrical start event')

        # Starts the theme once
        self.send_sound('pacman_theme')

        # Starts the startup lights immediately
        self.send_light('start')

        # Starts spin and periodically restarts startup animation
        self.start_motion('start_theatrical', self.start_theatrical_duration_s)

    def send_light(self, command: str):
        msg = String()
        msg.data = command
        self.light_pub.publish(msg)
        self.get_logger().info(f'Published light command: {command}')

    def send_sound(self, command: str):
        msg = String()
        msg.data = command
        self.sound_pub.publish(msg)
        self.get_logger().info(f'Published sound command: {command}')

    def start_motion(self, mode: str, duration_s: float):
        self.motion_active = True
        self.active_motion_mode = mode
        self.motion_time_remaining = duration_s
        self.current_shake_direction = 1.0
        self.time_until_direction_flip = self.shake_direction_interval_s
        self.time_until_light_restart = self.start_light_restart_period_s
        self.motion_timer.reset()

        self.get_logger().info(
            f'Started motion mode "{mode}" for {duration_s:.1f} seconds'
        )

    def stop_motion(self):
        self.motion_active = False
        self.active_motion_mode = None
        self.motion_time_remaining = 0.0
        self.current_shake_direction = 1.0
        self.time_until_direction_flip = self.shake_direction_interval_s
        self.time_until_light_restart = self.start_light_restart_period_s
        self.motion_timer.cancel()
        self.publish_stop()
        self.get_logger().info('Stopped active motion')

    def motion_timer_callback(self):
        if not self.motion_active:
            self.motion_timer.cancel()
            return

        if self.motion_time_remaining <= 0.0:
            self.stop_motion()
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0

        # Death uses equal left-right shake
        if self.active_motion_mode == 'death':
            msg.twist.angular.z = (
                self.current_shake_direction * self.shake_angular_z
            )

            self.time_until_direction_flip -= self.motion_publish_period_s
            if self.time_until_direction_flip <= 0.0:
                self.current_shake_direction *= -1.0
                self.time_until_direction_flip = (
                    self.shake_direction_interval_s
                )

        # Theatrical start uses continuous spin
        elif self.active_motion_mode == 'start_theatrical':
            msg.twist.angular.z = self.start_spin_angular_z

            self.time_until_light_restart -= self.motion_publish_period_s
            if self.time_until_light_restart <= 0.0:
                self.send_light('start')
                self.time_until_light_restart = (
                    self.start_light_restart_period_s
                )

        else:
            msg.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)
        self.motion_time_remaining -= self.motion_publish_period_s

    def publish_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GameEventMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.publish_stop()
            except Exception as exc:
                node.get_logger().warn(
                    f'Could not publish stop during shutdown: {exc}'
                )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
