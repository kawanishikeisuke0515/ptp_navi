import csv
import math
import os
from pathlib import Path
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from zed_interfaces.msg import ZedHeading


def normalize_to_180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def quaternion_to_yaw_rad(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class VioMagHeadingComparator(Node):
    def __init__(self):
        super().__init__('compare_vio_heading')

        self.declare_parameter('heading_topic', '/zed/heading')
        self.declare_parameter('odom_topic', '/zed2i/zed_node/odom')
        self.declare_parameter('csv_path', '/tmp/zed_vio_mag_compare.csv')
        self.declare_parameter('log_rate_hz', 1.0)
        self.declare_parameter('write_rate_hz', 20.0)

        self.heading_topic = str(self.get_parameter('heading_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.csv_path = str(self.get_parameter('csv_path').value)
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)
        self.write_rate_hz = float(self.get_parameter('write_rate_hz').value)
        self._validate_parameters()

        self.latest_heading: Optional[ZedHeading] = None
        self.latest_heading_time: Optional[Time] = None
        self.latest_vio_yaw_deg: Optional[float] = None
        self.latest_odom_time: Optional[Time] = None
        self.latest_odom_stamp_sec: Optional[float] = None
        self.mag_zero_deg: Optional[float] = None
        self.vio_zero_deg: Optional[float] = None
        self.last_log_time: Optional[Time] = None

        csv_file_path = Path(self.csv_path)
        csv_file_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = open(csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_sec',
            'mag_stamp_sec',
            'vio_stamp_sec',
            'mag_yaw_deg',
            'vio_yaw_deg',
            'mag_relative_yaw_deg',
            'vio_relative_yaw_deg',
            'yaw_diff_deg',
            'mag_age_sec',
            'vio_age_sec',
        ])
        self.csv_file.flush()

        self.create_subscription(
            ZedHeading,
            self.heading_topic,
            self.heading_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.write_rate_hz, self.timer_callback)

        self.get_logger().info(
            'comparing MAG heading %s with VIO odom %s; writing %s'
            % (self.heading_topic, self.odom_topic, self.csv_path)
        )

    def _validate_parameters(self):
        if self.heading_topic == '':
            raise ValueError('heading_topic must not be empty')
        if self.odom_topic == '':
            raise ValueError('odom_topic must not be empty')
        if self.csv_path == '':
            raise ValueError('csv_path must not be empty')
        if self.log_rate_hz <= 0.0 or not math.isfinite(self.log_rate_hz):
            raise ValueError('log_rate_hz must be a finite value greater than 0')
        if self.write_rate_hz <= 0.0 or not math.isfinite(self.write_rate_hz):
            raise ValueError('write_rate_hz must be a finite value greater than 0')

    def heading_callback(self, msg: ZedHeading):
        if not msg.valid:
            return
        self.latest_heading = msg
        self.latest_heading_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        self.latest_vio_yaw_deg = math.degrees(
            quaternion_to_yaw_rad(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
        )
        self.latest_odom_time = self.get_clock().now()
        self.latest_odom_stamp_sec = stamp_to_sec(msg.header.stamp)

    def timer_callback(self):
        if (
            self.latest_heading is None
            or self.latest_heading_time is None
            or self.latest_vio_yaw_deg is None
            or self.latest_odom_time is None
        ):
            return

        mag_yaw_deg = float(self.latest_heading.robot_yaw_deg)
        vio_yaw_deg = float(self.latest_vio_yaw_deg)
        if self.mag_zero_deg is None or self.vio_zero_deg is None:
            self.mag_zero_deg = mag_yaw_deg
            self.vio_zero_deg = vio_yaw_deg
            self.get_logger().info(
                'zero reference set: mag=%.3f deg vio=%.3f deg'
                % (self.mag_zero_deg, self.vio_zero_deg)
            )

        mag_relative_yaw_deg = normalize_to_180(mag_yaw_deg - self.mag_zero_deg)
        vio_relative_yaw_deg = normalize_to_180(vio_yaw_deg - self.vio_zero_deg)
        yaw_diff_deg = normalize_to_180(mag_relative_yaw_deg - vio_relative_yaw_deg)

        now = self.get_clock().now()
        time_sec = now.nanoseconds * 1e-9
        mag_age_sec = (now - self.latest_heading_time).nanoseconds * 1e-9
        vio_age_sec = (now - self.latest_odom_time).nanoseconds * 1e-9
        mag_stamp_sec = stamp_to_sec(self.latest_heading.header.stamp)
        vio_stamp_sec = (
            self.latest_odom_stamp_sec
            if self.latest_odom_stamp_sec is not None
            else ''
        )

        self.csv_writer.writerow([
            '%.6f' % time_sec,
            '%.6f' % mag_stamp_sec,
            '%.6f' % vio_stamp_sec if vio_stamp_sec != '' else '',
            '%.6f' % mag_yaw_deg,
            '%.6f' % vio_yaw_deg,
            '%.6f' % mag_relative_yaw_deg,
            '%.6f' % vio_relative_yaw_deg,
            '%.6f' % yaw_diff_deg,
            '%.6f' % mag_age_sec,
            '%.6f' % vio_age_sec,
        ])
        self.csv_file.flush()
        self._log_periodic(mag_relative_yaw_deg, vio_relative_yaw_deg, yaw_diff_deg)

    def _log_periodic(
        self,
        mag_relative_yaw_deg: float,
        vio_relative_yaw_deg: float,
        yaw_diff_deg: float,
    ):
        now = self.get_clock().now()
        if self.last_log_time is not None:
            elapsed = (now - self.last_log_time).nanoseconds * 1e-9
            if elapsed < 1.0 / self.log_rate_hz:
                return

        self.last_log_time = now
        self.get_logger().info(
            'relative_yaw_deg mag=%.3f vio=%.3f diff=%.3f'
            % (mag_relative_yaw_deg, vio_relative_yaw_deg, yaw_diff_deg)
        )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def main(args: Optional[list] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = VioMagHeadingComparator()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
