# Hello! This is just a sample stub node atm!
import rclpy
from rclpy.node import Node


class PlannerStub(Node):
    def __init__(self):
        super().__init__('planner_stub')
        self.get_logger().info('PacManBot planning stub node started')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerStub()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()