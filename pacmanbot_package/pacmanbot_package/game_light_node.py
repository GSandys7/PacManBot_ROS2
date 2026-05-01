from irobot_create_msgs.msg import LedColor, LightringLeds
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GameLightNode(Node):
    def __init__(self):
        super().__init__('game_light_node')

        # Subscribes to game light commands
        self.game_light_sub = self.create_subscription(
            String,
            '/robot_15/game_light',
            self.game_light_callback,
            10
        )

        # Publishes actual Create 3 ring light messages
        self.lightring_pub = self.create_publisher(
            LightringLeds,
            '/robot_15/cmd_lightring',
            10
        )

        # Timer used to drive animations
        self.anim_timer = self.create_timer(0.15, self.animation_step_callback)
        self.anim_timer.cancel()

        # Stores current animation state
        self.current_animation = None
        self.animation_frames = []
        self.animation_index = 0
        self.loop_animation = False

        self.get_logger().info(
            'Game light node started. Listening on /robot_15/game_light'
        )

    def game_light_callback(self, msg: String):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received game light command: {command}')

        if command == 'start':
            self.start_startup_animation()

        elif command == 'game over':
            self.start_game_over_animation()

        elif command == 'pellet':
            self.start_pellet_animation()

        elif command in ['power pellet', 'powerup']:
            self.start_power_pellet_animation()

        elif command == 'pacman':
            self.start_pacman_animation()

        elif command == 'win':
            self.start_win_animation()

        elif command == 'blinky caught':
            self.start_ghost_caught_animation((255, 0, 0))

        elif command == 'pinky caught':
            self.start_ghost_caught_animation((255, 60, 180))

        elif command == 'inky caught':
            self.start_ghost_caught_animation((0, 0, 255))

        elif command == 'clyde caught':
            self.start_ghost_caught_animation((255, 80, 0))

        elif command == 'blinky killed':
            self.start_ghost_killed_animation((255, 0, 0))

        elif command == 'pinky killed':
            self.start_ghost_killed_animation((255, 60, 180))

        elif command == 'inky killed':
            self.start_ghost_killed_animation((0, 0, 255))

        elif command == 'clyde killed':
            self.start_ghost_killed_animation((255, 80, 0))

        elif command == 'off':
            self.stop_animation()
            self.publish_all_off()

        elif command == 'yellow':
            self.stop_animation()
            self.publish_solid_color(255, 180, 0)

        elif command == 'red':
            self.stop_animation()
            self.publish_solid_color(255, 0, 0)

        elif command == 'blue':
            self.stop_animation()
            self.publish_solid_color(0, 0, 255)

        elif command == 'pink':
            self.stop_animation()
            self.publish_solid_color(255, 60, 180)

        elif command == 'orange':
            self.stop_animation()
            self.publish_solid_color(255, 80, 0)

        elif command == 'white':
            self.stop_animation()
            self.publish_solid_color(255, 255, 255)

        else:
            self.get_logger().warn(f'Unknown game light command: {command}')

    def start_startup_animation(self):
        self.stop_animation()

        yellow = (255, 180, 0)
        red = (255, 0, 0)
        blue = (0, 0, 255)
        orange = (255, 80, 0)
        pink = (255, 60, 180)
        off = (0, 0, 0)

        base_pattern = [yellow, red, blue, orange, pink, off]
        frames = []

        # Rotates the multicolor pattern around the ring
        for step in range(30):
            frame = [base_pattern[(i - step) % 6] for i in range(6)]
            frames.append(frame)

        # Holds all LEDs off briefly
        for _ in range(20):
            frames.append([off] * 6)

        # Ends on solid yellow
        frames.append([yellow] * 6)

        self.animation_frames = frames
        self.animation_index = 0
        self.current_animation = 'startup'
        self.loop_animation = False
        self.anim_timer.reset()

        self.get_logger().info('Started startup chase animation')

    def start_game_over_animation(self):
        self.stop_animation()

        red = (255, 0, 0)
        dim_red = (60, 0, 0)
        mid_red = (140, 0, 0)

        frames = []

        # Rotates a red wave around the ring
        for step in range(6):
            frame = []
            for i in range(6):
                if i == step:
                    frame.append(red)
                elif i == (step - 1) % 6 or i == (step + 1) % 6:
                    frame.append(mid_red)
                else:
                    frame.append(dim_red)
            frames.append(frame)

        self.animation_frames = frames
        self.animation_index = 0
        self.current_animation = 'game_over'
        self.loop_animation = True
        self.anim_timer.reset()

        self.get_logger().info('Started game over animation')

    def start_power_pellet_animation(self):
        self.stop_animation()

        yellow = (255, 180, 0)
        white = (255, 255, 255)

        frames = [
            [yellow] * 6,
            [white] * 6,
        ]

        self.animation_frames = frames
        self.animation_index = 0
        self.current_animation = 'power_pellet'
        self.loop_animation = True
        self.anim_timer.reset()

        self.get_logger().info('Started power pellet animation')

    def start_pellet_animation(self):
        self.stop_animation()

        yellow = (255, 180, 0)
        dim_yellow = (80, 55, 0)
        off = (0, 0, 0)

        frames = [
            [yellow, off, off, off, yellow, yellow],
            [yellow, yellow, off, yellow, yellow, yellow],
            [dim_yellow] * 6,
            [off] * 6,
        ]

        self.animation_frames = frames
        self.animation_index = 0
        self.current_animation = 'pellet'
        self.loop_animation = False
        self.anim_timer.reset()

        self.get_logger().info('Started pellet chomp animation')

    def start_pacman_animation(self):
        self.stop_animation()

        yellow = (255, 180, 0)
        off = (0, 0, 0)

        frames = [
            [yellow, off, off, off, yellow, yellow],
            [yellow, yellow, off, yellow, yellow, yellow],
        ]

        self.animation_frames = frames
        self.animation_index = 0
        self.current_animation = 'pacman'
        self.loop_animation = True
        self.anim_timer.reset()

        self.get_logger().info('Started Pac-Man chomp animation')

    def start_win_animation(self):
        self.stop_animation()

        yellow = (255, 180, 0)
        white = (255, 255, 255)
        blue = (0, 80, 255)
        pink = (255, 60, 180)
        orange = (255, 80, 0)
        off = (0, 0, 0)

        frames = [
            [yellow, white, blue, pink, orange, white],
            [white, yellow, white, blue, pink, orange],
            [orange, white, yellow, white, blue, pink],
            [pink, orange, white, yellow, white, blue],
            [off] * 6,
            [white] * 6,
        ]

        self.animation_frames = frames
        self.animation_index = 0
        self.current_animation = 'win'
        self.loop_animation = True
        self.anim_timer.reset()

        self.get_logger().info('Started win celebration animation')

    def start_ghost_caught_animation(self, ghost_color):
        self.stop_animation()

        yellow = (255, 180, 0)

        frames = [
            [ghost_color] * 6,
            [yellow] * 6,
        ]

        self.animation_frames = frames
        self.animation_index = 0
        self.current_animation = 'ghost_caught'
        self.loop_animation = True
        self.anim_timer.reset()

        self.get_logger().info(
            f'Started ghost caught animation with color {ghost_color}'
        )

    def start_ghost_killed_animation(self, ghost_color):
        self.stop_animation()

        red = (255, 0, 0)
        dim_red = (90, 0, 0)
        off = (0, 0, 0)

        frames = [
            [ghost_color] * 6,
            [red] * 6,
            [ghost_color, red, ghost_color, red, ghost_color, red],
            [red, ghost_color, red, ghost_color, red, ghost_color],
            [dim_red] * 6,
            [off] * 6,
        ]

        self.animation_frames = frames
        self.animation_index = 0
        self.current_animation = 'ghost_killed'
        self.loop_animation = True
        self.anim_timer.reset()

        self.get_logger().info(
            f'Started ghost killed blink with color {ghost_color}'
        )

    def animation_step_callback(self):
        if not self.animation_frames:
            self.anim_timer.cancel()
            return

        frame = self.animation_frames[self.animation_index]
        self.publish_led_frame(frame)

        self.animation_index += 1

        if self.animation_index >= len(self.animation_frames):
            if self.loop_animation:
                self.animation_index = 0
            else:
                # Leaves startup animation on solid yellow when finished
                if self.current_animation == 'startup':
                    self.publish_solid_color(255, 180, 0)

                self.stop_animation()

    def stop_animation(self):
        self.current_animation = None
        self.animation_frames = []
        self.animation_index = 0
        self.loop_animation = False
        self.anim_timer.cancel()

    def publish_all_off(self):
        self.publish_solid_color(0, 0, 0)

    def publish_solid_color(self, red: int, green: int, blue: int):
        frame = [(red, green, blue)] * 6
        self.publish_led_frame(frame)

    def publish_led_frame(self, frame):
        msg = LightringLeds()
        msg.override_system = True
        msg.leds = []

        for red, green, blue in frame:
            led = LedColor()
            led.red = int(red)
            led.green = int(green)
            led.blue = int(blue)
            msg.leds.append(led)

        self.lightring_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GameLightNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.publish_all_off()
            except Exception as exc:
                node.get_logger().warn(
                    f'Could not turn lights off during shutdown: {exc}'
                )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
