"""ROS adapter for UWB-derived robot center poses."""
from dataclasses import fields
import math
import os

from geometry_msgs.msg import PoseStamped
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from uwb_interfaces.msg import UwbPosition
from zed_interfaces.msg import ZedHeading

from uwb_robot_pose_publisher.pose_core import Config, NSEC, PoseSynchronizer


class UwbRobotPosePublisher(Node):
    def __init__(self):
        super().__init__('uwb_robot_pose_publisher')
        defaults = Config()
        parameters = {
            'uwb_position_topic': '/uwb/position',
            'zed_heading_topic': '/zed/heading',
            'robot_pose_topic': '/uwb/robot_pose',
            'world_frame_id': 'world',
        }
        parameters.update({f.name: getattr(defaults, f.name) for f in fields(defaults)})
        for name, value in parameters.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        values = {name: self.get_parameter(name).value for name in parameters}
        for name, default in parameters.items():
            if isinstance(default, str) and not values[name].strip():
                raise ValueError(f'{name} must not be empty')
        config = Config(**{f.name: values[f.name] for f in fields(defaults)})
        self.world_frame = values['world_frame_id']
        self.sync = PoseSynchronizer(config, self._warn)
        self.publisher = self.create_publisher(PoseStamped, values['robot_pose_topic'], 10)
        self.create_subscription(UwbPosition, values['uwb_position_topic'],
                                 self.position_callback, 10)
        self.create_subscription(ZedHeading, values['zed_heading_topic'],
                                 self.heading_callback, 10)
        period = min(0.02, config.heading_wait_timeout_sec / 2,
                     config.position_timeout_sec / 2, config.heading_timeout_sec / 2)
        self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f"Publishing {values['robot_pose_topic']} in {self.world_frame}; "
            f'tag forward={config.tag_offset_forward_m} m, left={config.tag_offset_left_m} m')

    def _warn(self, reason):
        self.get_logger().warning(reason, throttle_duration_sec=2.0)

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec * NSEC + msg.header.stamp.nanosec

    def position_callback(self, msg):
        self._publish(self.sync.position(self._stamp(msg), msg.x_m, msg.y_m, msg.valid,
                                         self.get_clock().now().nanoseconds))

    def heading_callback(self, msg):
        self._publish(self.sync.heading(self._stamp(msg), msg.robot_yaw_rad, msg.valid,
                                        self.get_clock().now().nanoseconds))

    def timer_callback(self):
        self._publish(self.sync.poll(self.get_clock().now().nanoseconds))

    def _publish(self, poses):
        for pose in poses:
            msg = PoseStamped()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(pose.stamp, NSEC)
            msg.header.frame_id = self.world_frame
            msg.pose.position.x = pose.x
            msg.pose.position.y = pose.y
            msg.pose.orientation.z = math.sin(pose.yaw / 2)
            msg.pose.orientation.w = math.cos(pose.yaw / 2)
            self.publisher.publish(msg)


def main(args=None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init(args=args)
    node = None
    try:
        node = UwbRobotPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
