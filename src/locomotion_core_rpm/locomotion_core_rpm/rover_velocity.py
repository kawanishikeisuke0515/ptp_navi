"""Twist to motor RPM publisher."""
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from .RoverDescription import RoverDescription
from .control import FreshCommand, positive, rpm_limits


class RoverVelocity(Node):
    def __init__(self):
        super().__init__('rover_velocity_rpm')
        self.declare_parameters('', [
            ('command_topic', '/rov_cmd_vel'), ('motor_topic', '/rov/motor_rpm'),
            ('wheel_radius_m', 0.0635), ('half_length_m', 0.205),
            ('half_width_m', 0.170), ('gear_ratios', [1.0] * 4),
            ('max_motor_rpm', [0.0] * 4), ('command_rate_hz', 20.0),
            ('twist_timeout_sec', 0.5)])
        p = lambda name: self.get_parameter(name).value
        self.limits = rpm_limits(p('max_motor_rpm'))
        self.rover = RoverDescription(p('wheel_radius_m'), p('half_length_m'),
                                      p('half_width_m'), p('gear_ratios'))
        self.command = FreshCommand(p('twist_timeout_sec'))
        self.pub = self.create_publisher(Float32MultiArray, p('motor_topic'), 1)
        self.sub = self.create_subscription(Twist, p('command_topic'), self.receive, 1)
        self.timer = self.create_timer(
            1.0 / positive(p('command_rate_hz'), 'command_rate_hz'), self.publish,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        try:
            self.command.set(self.rover.motor_rpm(
                [msg.linear.x, msg.linear.y, msg.angular.z], self.limits))
        except (ValueError, OverflowError):
            self.command.clear()
            self.pub.publish(Float32MultiArray(data=[0.0] * 4))
            self.get_logger().error('Invalid Twist; publishing zero RPM')

    def publish(self):
        self.pub.publish(Float32MultiArray(data=self.command.get() or [0.0] * 4))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoverVelocity()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
