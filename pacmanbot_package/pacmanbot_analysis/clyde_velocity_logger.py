#!/usr/bin/env python3

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class ClydeVelocityLogger(Node):
    def __init__(self) -> None:
        super().__init__('clyde_velocity_logger')

        self.declare_parameter('pose_topic', '/robot_15/clyde_pose')
        self.declare_parameter('speed_topic', '/robot_15/clyde/speed')
        self.declare_parameter('output_dir', os.path.expanduser('~/ros2_ws/clyde_logs'))

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value

        os.makedirs(output_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # --- pose / velocity log ---
        self._pose_csv_path = os.path.join(output_dir, f'clyde_velocity_{ts}.csv')
        self._pose_file = open(self._pose_csv_path, 'w', newline='')
        self._pose_writer = csv.writer(self._pose_file)
        self._pose_writer.writerow([
            'stamp_sec', 'stamp_nanosec',
            'x', 'y',
            'dx', 'dy', 'dt_sec',
            'vx', 'vy', 'v_abs',
        ])
        self._pose_file.flush()

        # --- speed log (Float64 has no header, so we attach wall-clock time) ---
        self._speed_csv_path = os.path.join(output_dir, f'clyde_speed_{ts}.csv')
        self._speed_file = open(self._speed_csv_path, 'w', newline='')
        self._speed_writer = csv.writer(self._speed_file)
        self._speed_writer.writerow(['wall_time_sec', 'speed'])
        self._speed_file.flush()

        self._start_ns = self.get_clock().now().nanoseconds
        self._prev_x: float | None = None
        self._prev_y: float | None = None
        self._prev_stamp_ns: int | None = None
        self._pose_count = 0
        self._speed_count = 0

        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        self.create_subscription(Float64, speed_topic, self._speed_cb, 10)

        self.get_logger().info(f'Subscribing to pose : {pose_topic}')
        self.get_logger().info(f'Subscribing to speed: {speed_topic}')
        self.get_logger().info(f'Velocity log : {self._pose_csv_path}')
        self.get_logger().info(f'Speed log    : {self._speed_csv_path}')
        self.get_logger().info('Waiting for publishers… press Ctrl+C to stop.')

    # ------------------------------------------------------------------

    def _speed_cb(self, msg: Float64) -> None:
        wall_sec = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9
        self._speed_writer.writerow([f'{wall_sec:.6f}', f'{msg.data:.6f}'])
        self._speed_file.flush()
        self._speed_count += 1

    def _pose_cb(self, msg: PoseStamped) -> None:
        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec
        stamp_ns = stamp_sec * 10**9 + stamp_nanosec

        x = msg.pose.position.x
        y = msg.pose.position.y

        # Seed state on the first message, nothing to log yet
        if self._prev_x is None:
            self._prev_x, self._prev_y, self._prev_stamp_ns = x, y, stamp_ns
            return

        dt_ns = stamp_ns - self._prev_stamp_ns
        dt_sec = dt_ns / 1e9

        if dt_sec <= 0.0:
            return

        dx = x - self._prev_x
        dy = y - self._prev_y
        vx = dx / dt_sec
        vy = dy / dt_sec
        v_abs = math.hypot(vx, vy)

        self._pose_writer.writerow([
            stamp_sec, stamp_nanosec,
            f'{x:.6f}', f'{y:.6f}',
            f'{dx:.6f}', f'{dy:.6f}', f'{dt_sec:.6f}',
            f'{vx:.6f}', f'{vy:.6f}', f'{v_abs:.6f}',
        ])
        self._pose_file.flush()

        self._prev_x, self._prev_y, self._prev_stamp_ns = x, y, stamp_ns
        self._pose_count += 1

        if self._pose_count % 20 == 0:
            self.get_logger().info(
                f'[{self._pose_count}] x={x:.3f} y={y:.3f} '
                f'vx={vx:.3f} vy={vy:.3f} |v|={v_abs:.3f} m/s'
            )

    # ------------------------------------------------------------------

    def destroy_node(self) -> bool:
        for f, path, count, label in [
            (self._pose_file,  self._pose_csv_path,  self._pose_count,  'velocity'),
            (self._speed_file, self._speed_csv_path, self._speed_count, 'speed'),
        ]:
            try:
                if not f.closed:
                    f.close()
                    self.get_logger().info(
                        f'Saved {count} {label} samples → {path}'
                    )
            except Exception as e:
                self.get_logger().error(f'Error closing {label} file: {e}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClydeVelocityLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping…')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
