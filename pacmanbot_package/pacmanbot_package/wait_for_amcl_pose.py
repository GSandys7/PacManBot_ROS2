#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class WaitForAMCLPose(Node):
    def __init__(self):
        super().__init__('wait_for_amcl_pose')

        self.received_pose = False
        self.robot_ns = '/robot_15'

        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'{self.robot_ns}/amcl_pose',
            self.pose_callback,
            amcl_qos
        )

        self.get_logger().info(
            f'Waiting for first AMCL pose on {self.robot_ns}/amcl_pose'
        )

    def pose_callback(self, msg):
        self.received_pose = True
        self.get_logger().info(
            f'AMCL pose received: '
            f'({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaitForAMCLPose()

    try:
        while rclpy.ok() and not node.received_pose:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
