"""Collect compass calibration while commanding an in-place turn."""

import math
import os
import signal
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.signals import SignalHandlerOptions

from .calibrate_zed_heading import ZedHeadingCalibrator


class AutoZedHeadingCalibrator(ZedHeadingCalibrator):
    def __init__(self):
        super().__init__()
        self.declare_parameter('command_topic', '/rov_cmd_vel')
        self.declare_parameter('angular_speed_rad_sec', 0.15)
        self.declare_parameter('startup_timeout_sec', 60.0)
        self.declare_parameter('sensor_timeout_sec', 1.0)
        self.speed = float(self.get_parameter('angular_speed_rad_sec').value)
        self.startup_timeout = float(self.get_parameter('startup_timeout_sec').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout_sec').value)
        if not math.isfinite(self.speed) or not 0.0 < abs(self.speed) <= 0.3:
            raise ValueError('angular_speed_rad_sec must have magnitude in (0, 0.3]')
        for value in (self.startup_timeout, self.sensor_timeout):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('timeouts must be positive and finite')
        if self.input_csv_path or not self.use_vio_turn_detection:
            raise ValueError('automatic rotation requires live VIO turn detection')
        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter('command_topic').value), 1)
        self.wait_started = time.monotonic()
        self.turn_started = None
        self.last_mag = None
        self.last_odom = None
        self.create_timer(0.05, self.control_callback)
        self.get_logger().info('Waiting for magnetic field and VIO before rotating')

    def mag_callback(self, msg):
        previous_count = self.sample_count
        super().mag_callback(msg)
        if self.sample_count > previous_count:
            self.last_mag = time.monotonic()

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            return
        if sum(v * v for v in (q.x, q.y, q.z, q.w)) < 1e-12:
            return
        super().odom_callback(msg)
        self.last_odom = time.monotonic()

    def stop(self):
        self.command_publisher.publish(Twist())

    def abort(self, reason):
        self.stop()
        self.finished = True
        self.get_logger().error(reason + '; stopped without saving calibration')

    def control_callback(self):
        if self.finished:
            self.stop()
            return
        now = time.monotonic()
        fresh = all(stamp is not None and now - stamp < self.sensor_timeout
                    for stamp in (self.last_mag, self.last_odom))
        if self.turn_started is None:
            self.stop()
            if fresh:
                self.turn_started = now
                self.start_time = self.get_clock().now()
                self.last_progress_log_time = self.start_time
                self.get_logger().info('Starting in-place calibration turn')
            elif now - self.wait_started >= self.startup_timeout:
                self.abort('Timed out waiting for magnetic field / VIO')
            return
        if not fresh:
            self.abort('Magnetic field / VIO reception lost')
            return
        if now - self.turn_started >= self.max_duration_sec:
            self.abort('Maximum rotation duration reached')
            return
        command = Twist()
        command.angular.z = self.speed
        self.command_publisher.publish(command)

    def timer_callback(self):
        # The parent timer must not consume the camera startup time.
        if getattr(self, 'turn_started', None) is not None:
            super().timer_callback()

    def _finish(self):
        self.stop()
        super()._finish()


def main(args=None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_term = signal.getsignal(signal.SIGTERM)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        node = AutoZedHeadingCalibrator()
        # Leave the executor callback before shutting down the ROS context.
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)
