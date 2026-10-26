"""ROS adapter for autonomous straight path control."""
from dataclasses import fields
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from .core import Config, Controller


class PathControl(Node):
    def __init__(self):
        super().__init__('path_control')
        parameters = {f.name: f.default for f in fields(Config) if f.name not in ('goal_x', 'goal_y')}
        parameters.update(goal_x=float('nan'), goal_y=float('nan'),
                          pose_topic='/uwb/robot_pose', command_topic='/rov_cmd_vel',
                          status_topic='/path_control/status', world_frame_id='world')
        for key, value in parameters.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        values = {key: self.get_parameter(key).value for key in parameters}
        for key in ('pose_topic', 'command_topic', 'status_topic', 'world_frame_id'):
            if not values[key].strip():
                raise ValueError(f'{key} must not be empty')
        self.world_frame = values['world_frame_id']
        config = Config(**{f.name: values[f.name] for f in fields(Config)})
        self.control = Controller(config, self.now())
        self.command_pub = self.create_publisher(Twist, values['command_topic'], 1)
        self.status_pub = self.create_publisher(String, values['status_topic'], 10)
        self.create_subscription(PoseStamped, values['pose_topic'], self.receive, 10)
        self.create_timer(1.0 / config.control_rate_hz, self.update,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Path control ready: automatically starts after initialization')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def receive(self, msg):
        now = self.now()
        q = msg.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        norm = math.hypot(*values)
        if msg.header.frame_id != self.world_frame:
            self.control.fail('frame_mismatch')
        elif not all(math.isfinite(v) for v in values) or abs(norm - 1.) > 0.01:
            self.control.fail('invalid_quaternion')
        elif not math.isfinite(msg.pose.position.z):
            self.control.fail('nonfinite_pose')
        else:
            x, y, z, w = (v / norm for v in values)
            yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
            self.control.receive(msg.pose.position.x, msg.pose.position.y, yaw,
                                 msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9, now)
        # Publish immediately on invalid input or transition into a stopping state.
        if self.control.state != 'DRIVE':
            self.command_pub.publish(Twist())

    def update(self):
        vx, vy, omega = self.control.tick(self.now())
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, omega
        self.command_pub.publish(msg)
        self.status_pub.publish(String(data=json.dumps({
            'state': self.control.state, 'reason': self.control.reason,
            'error': self.control.state == 'ERROR',
            'init_samples': len(self.control.samples), 'start': self.control.a,
            'goal': [self.control.c.goal_x, self.control.c.goal_y],
        }, allow_nan=False)))

    def destroy_node(self):
        if hasattr(self, 'command_pub'):
            self.command_pub.publish(Twist())
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
