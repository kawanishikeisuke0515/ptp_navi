"""Publish Jetson module power reported by an owned tegrastats process."""
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from jetson_interfaces.msg import JetsonPower
from .core import Monitor


class JetsonPowerPublisher(Node):
    def __init__(self):
        super().__init__('jetson_power_publisher')
        self.monitor = None
        try:
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Real-time measurement requires use_sim_time=false')
            defaults = dict(tegrastats_path='tegrastats', power_topic='/jetson/power',
                            total_power_rail='VDD_IN', interval_ms=1000, output_timeout_sec=3.0)
            p = {k: self.declare_parameter(k, v, ParameterDescriptor(read_only=True)).value
                 for k, v in defaults.items()}
            for key in ('tegrastats_path', 'power_topic', 'total_power_rail'):
                if not p[key].strip():
                    raise ValueError(f'{key} must not be empty')
            if any(c.isspace() for c in p['total_power_rail']):
                raise ValueError('total_power_rail must not contain whitespace')
            if type(p['interval_ms']) is not int or p['interval_ms'] <= 0:
                raise ValueError('interval_ms must be a positive integer')
            timeout = p['output_timeout_sec']
            if not math.isfinite(timeout) or timeout <= p['interval_ms'] / 1000:
                raise ValueError('output_timeout_sec must exceed interval_ms / 1000')
            self.rail = p['total_power_rail']
            self.publisher = self.create_publisher(JetsonPower, p['power_topic'], 10)
            self.monitor = Monitor([p['tegrastats_path'], '--interval', str(p['interval_ms'])],
                                   self.rail, timeout, p['interval_ms'] / 1000, self.publish)
        except BaseException:
            self.destroy_node()
            raise

    def publish(self, reading):
        if not self.context.ok():
            return
        msg = JetsonPower()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.total_power_rail = self.rail
        msg.power_w, msg.average_power_w = reading.power_w, reading.average_power_w
        msg.valid, msg.status, msg.raw_line = reading.valid, reading.status, reading.raw_line
        self.publisher.publish(msg)
        if not reading.valid:
            self.get_logger().warning('Power telemetry: ' + reading.status, throttle_duration_sec=5.)

    def destroy_node(self):
        if self.monitor is not None:
            self.monitor.close()
            self.monitor = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JetsonPowerPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
