#!/usr/bin/env python3

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Convert quaternion to yaw (theta) in radians.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AMCLLogger(Node):
    def __init__(self) -> None:
        super().__init__('amcl_logger')

        # Parameters so you can override them later if needed
        self.declare_parameter('amcl_topic', '/robot_15/amcl_pose')
        self.declare_parameter('output_dir', os.path.expanduser('~/ros2_ws/amcl_logs'))
        self.declare_parameter('file_prefix', 'amcl_pose_log')

        self.amcl_topic = self.get_parameter('amcl_topic').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.file_prefix = self.get_parameter('file_prefix').get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'{self.file_prefix}_{timestamp}.csv')

        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)

        # CSV header
        self.writer.writerow([
            'stamp_sec',
            'stamp_nanosec',
            'elapsed_time_sec',
            'x',
            'y',
            'theta_rad',
            'theta_deg',
            'cov_xx',
            'cov_xy',
            'cov_yy',
            'cov_yawyaw'
        ])

        self.start_time_ns = self.get_clock().now().nanoseconds
        self.msg_count = 0

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.listener_callback,
            10
        )

        self.get_logger().info('AMCL logger started.')
        self.get_logger().info(f'Subscribing to: {self.amcl_topic}')
        self.get_logger().info(f'Writing CSV to: {self.csv_path}')
        self.get_logger().info('Keep the robot stationary during the measurement.')
        self.get_logger().info('Press Ctrl+C when done.')

    def listener_callback(self, msg: PoseWithCovarianceStamped) -> None:
        try:
            # ROS message timestamp
            stamp_sec = msg.header.stamp.sec
            stamp_nanosec = msg.header.stamp.nanosec

            # Elapsed wall time since node start
            now_ns = self.get_clock().now().nanoseconds
            elapsed_time_sec = (now_ns - self.start_time_ns) / 1e9

            # Pose
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            theta_rad = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            theta_deg = math.degrees(theta_rad)

            # Pose covariance is a 6x6 flattened row-major matrix:
            # [x, y, z, roll, pitch, yaw]
            cov = msg.pose.covariance
            cov_xx = cov[0]    # variance of x
            cov_xy = cov[1]    # covariance x-y
            cov_yy = cov[7]    # variance of y
            cov_yawyaw = cov[35]  # variance of yaw

            self.writer.writerow([
                stamp_sec,
                stamp_nanosec,
                f'{elapsed_time_sec:.6f}',
                f'{x:.6f}',
                f'{y:.6f}',
                f'{theta_rad:.6f}',
                f'{theta_deg:.6f}',
                f'{cov_xx:.8f}',
                f'{cov_xy:.8f}',
                f'{cov_yy:.8f}',
                f'{cov_yawyaw:.8f}'
            ])
            self.file.flush()

            self.msg_count += 1

            if self.msg_count % 20 == 0:
                self.get_logger().info(
                    f'Logged {self.msg_count} samples | '
                    f'x={x:.3f}, y={y:.3f}, theta={theta_deg:.2f} deg'
                )

        except Exception as e:
            self.get_logger().error(f'Error while logging AMCL pose: {e}')

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, 'file') and self.file and not self.file.closed:
                self.file.close()
                self.get_logger().info(f'CSV saved to: {self.csv_path}')
                self.get_logger().info(f'Total samples logged: {self.msg_count}')
        except Exception as e:
            self.get_logger().error(f'Error closing CSV file: {e}')

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AMCLLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping AMCL logger...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()