"""ROS interface for RPM commands."""
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Float32MultiArray
from .control import RoboteqDriver, positive


class MotorDriver(Node):
    def __init__(self):
        super().__init__('cmd_roboteq_rpm')
        self.declare_parameters('', [
            ('motor_topic', '/rov/motor_rpm'),
            ('front_port', '/dev/roboteq_1'), ('back_port', '/dev/roboteq_2'),
            ('baudrate', 115200), ('max_motor_rpm', [0.0] * 4),
            ('motor_directions', [-1, -1, -1, -1]),
            ('command_rate_hz', 20.0), ('rpm_timeout_sec', 0.5)])
        p = lambda name: self.get_parameter(name).value
        period = 1.0 / positive(p('command_rate_hz'), 'command_rate_hz')
        self.driver = RoboteqDriver(
            p('front_port'), p('back_port'), p('baudrate'), p('max_motor_rpm'),
            p('motor_directions'), p('rpm_timeout_sec'))
        try:
            self.sub = self.create_subscription(
                Float32MultiArray, p('motor_topic'), self.receive, 1)
            self.timer = self.create_timer(
                period, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        except Exception:
            self.driver.close()
            raise
        self.get_logger().info('RPM driver ready; waiting for fresh RPM input')

    def receive(self, msg):
        try:
            self.driver.receive(msg.data)
        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().error(f'Invalid RPM input; stopped: {exc}')

    def tick(self):
        try:
            self.driver.tick()
        except Exception as exc:
            self.get_logger().error(f'Drive stopped: {exc}')

    def destroy_node(self):
        for error in self.driver.close():
            self.get_logger().error(f'Shutdown stop/close failed: {error}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
