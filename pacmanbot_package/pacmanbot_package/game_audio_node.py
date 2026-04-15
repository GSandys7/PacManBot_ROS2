import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GameEventMapper(Node):
    def __init__(self):
        super().__init__('game_event_mapper')

        # Listen for one game event
        self.sub = self.create_subscription(
            String,
            '/robot_15/game_event',
            self.event_callback,
            10
        )

        # Publish to light node
        self.light_pub = self.create_publisher(
            String,
            '/robot_15/game_light',
            10
        )

        # Publish to audio node
        self.sound_pub = self.create_publisher(
            String,
            '/robot_15/game_sound',
            10
        )

        self.get_logger().info('Game Event Mapper Node Started')

    def event_callback(self, msg: String):
        event = msg.data.strip().lower()
        self.get_logger().info(f'Received event: {event}')

        if event == 'start':
            self.send_light('start')
            self.send_sound('start')
        else:
            self.get_logger().warning(f'Unknown event: {event}')

    def send_light(self, command):
        msg = String()
        msg.data = command
        self.light_pub.publish(msg)

    def send_sound(self, command):
        msg = String()
        msg.data = command
        self.sound_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GameEventMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()