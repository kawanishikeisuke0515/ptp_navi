"""Publish robot center positions from OptiTrack rigid-body poses."""
from copy import deepcopy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from .optitrack_geometry import center_position


class OptitrackCenter(Node):
    def __init__(self):
        super().__init__('optitrack_center')
        defaults = dict(input_topic='/vrpn_mocap/RigidBody_1/pose',
                        output_topic='/optitrack/robot_pose',
                        center_to_rigid_body_x_m=0.0,
                        center_to_rigid_body_y_m=0.0,
                        center_to_rigid_body_z_m=0.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        config = {key: self.get_parameter(key).value for key in defaults}
        if any(not config[key].strip() for key in ('input_topic', 'output_topic')):
            raise ValueError('Topic names must not be empty')
        self.offset = tuple(config[f'center_to_rigid_body_{axis}_m'] for axis in 'xyz')
        if not all(math.isfinite(v) for v in self.offset):
            raise ValueError('Offsets must be finite')
        self.publisher = self.create_publisher(PoseStamped, config['output_topic'], 10)
        # Compare resolved names to prevent a remapping from creating a feedback loop.
        if self.resolve_topic_name(config['input_topic']) == self.publisher.topic_name:
            raise ValueError('Input and output topics must differ')
        self.create_subscription(PoseStamped, config['input_topic'], self.receive,
                                 qos_profile_sensor_data)

    def receive(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        try:
            xyz = center_position((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.offset)
            if not msg.header.frame_id:
                raise ValueError('Missing coordinate frame')
        except ValueError as exc:
            self.get_logger().warning(f'Ignoring invalid OptiTrack pose: {exc}',
                                      throttle_duration_sec=5.0)
            return
        result = deepcopy(msg)
        result.pose.position.x, result.pose.position.y, result.pose.position.z = xyz
        # Preserve measurement timestamp, mocap frame and rigid-body orientation.
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OptitrackCenter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
