"""Passive logger for path-control experiments."""
from datetime import datetime, timezone
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from uwb_interfaces.msg import UwbPosition
from .log_store import LogStore


def finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


class PathControlLogger(Node):
    def __init__(self):
        super().__init__('path_control_logger')
        defaults = dict(output_dir='~/path_control_logs', timeline_rate_hz=20.0,
                        stale_after_sec=0.6, status_topic='/path_control/status',
                        command_topic='/rov_cmd_vel',
                        optitrack_pose_topic='/vrpn_mocap/RigidBody_1/pose',
                        uwb_position_topic='/uwb/position', robot_pose_topic='/uwb/robot_pose')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        config = {key: self.get_parameter(key).value for key in defaults}
        for key in ('timeline_rate_hz', 'stale_after_sec'):
            if not finite(config[key]) or config[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if any(not str(value).strip() for key, value in config.items()
               if key.endswith('_topic') or key == 'output_dir'):
            raise ValueError('Output directory and topic names must not be empty')
        self.store = LogStore(config['output_dir'], {'logger_config': config,
            'units': {'position': 'm', 'linear_velocity': 'm/s', 'angular_velocity': 'rad/s'},
            'note': 'Raw coordinate frames retained; no sensor alignment applied.'},
            config['stale_after_sec'])
        self.create_subscription(String, config['status_topic'], self.status, 100)
        self.create_subscription(Twist, config['command_topic'], self.command, 100)
        self.create_subscription(UwbPosition, config['uwb_position_topic'], self.position, 100)
        self.create_subscription(PoseStamped, config['robot_pose_topic'],
                                 lambda msg: self.pose('robot_pose', msg), 100)
        self.create_subscription(PoseStamped, config['optitrack_pose_topic'],
                                 lambda msg: self.pose('optitrack', msg), qos_profile_sensor_data)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1 / config['timeline_rate_hz'], self.timeline, clock=steady)
        self.create_timer(1.0, self.store.flush, clock=steady)
        self.parameter_clients = {name: AsyncParameterClient(self, name) for name in (
            '/path_control', '/uwb_position_publisher', '/uwb_robot_pose_publisher',
            '/zed_heading_publisher', '/rover_velocity_rpm')}
        self.pending = {}
        self.create_timer(1.0, self.snapshot_parameters, clock=steady)
        self.get_logger().info(f'Experiment logs: {self.store.directory}')

    def record(self, stream, values, msg=None, valid=True):
        stamp = msg.header.stamp if msg is not None else None
        self.store.record(stream, values, self.get_clock().now().nanoseconds,
                          time.monotonic_ns(),
                          stamp.sec * 10**9 + stamp.nanosec if stamp else None,
                          msg.header.frame_id if msg is not None else '', valid)

    def status(self, msg):
        values = dict(raw_json=msg.data)
        valid = False
        try:
            data = json.loads(msg.data)
            values.update({key: data.get(key) for key in ('state', 'reason', 'error', 'init_samples')})
            for key, prefix in (('start', 'start'), ('goal', 'goal')):
                point = data.get(key)
                if point is not None:
                    if len(point) != 2 or not finite(*point):
                        raise ValueError('Invalid path point')
                    values[f'{prefix}_x'], values[f'{prefix}_y'] = point
            valid = isinstance(data.get('state'), str)
        except (ValueError, TypeError, AttributeError, KeyError):
            pass
        self.record('status', values, valid=valid)

    def command(self, msg):
        values = dict(linear_x=msg.linear.x, linear_y=msg.linear.y, angular_z=msg.angular.z)
        self.record('cmd_vel', values, valid=finite(*values.values()))

    def position(self, msg):
        self.record('uwb_position', dict(x=msg.x_m, y=msg.y_m, valid=msg.valid,
                    device_time_ms=msg.device_time_ms), msg,
                    msg.valid and finite(msg.x_m, msg.y_m))

    def pose(self, stream, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = dict(x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w)
        self.record(stream, values, msg,
                    finite(*values.values()) and math.hypot(q.x, q.y, q.z, q.w) > 0)

    def timeline(self):
        self.store.timeline(self.get_clock().now().nanoseconds, time.monotonic_ns())

    def snapshot_parameters(self):
        # Retry until each node's parameter services become available. Never block logging.
        for name, client in self.parameter_clients.items():
            if name in self.store.metadata['parameter_snapshots'] or not client.services_are_ready():
                continue
            pending = self.pending.get(name)
            if pending is None:
                self.pending[name] = ('list', client.list_parameters(), [], time.monotonic())
                continue
            stage, future, names, started = pending
            if not future.done():
                if time.monotonic() - started > 10:
                    future.cancel()
                    del self.pending[name]
                continue
            try:
                result = future.result()
                if stage == 'list':
                    names = result.result.names
                    self.pending[name] = ('get', client.get_parameters(names), names, time.monotonic())
                else:
                    from rclpy.parameter import parameter_value_to_python
                    self.store.metadata['parameter_snapshots'][name] = {
                        'captured_ros_ns': self.get_clock().now().nanoseconds,
                        'parameters': {key: parameter_value_to_python(value)
                                       for key, value in zip(names, result.values)}}
                    self.store.save_metadata()
                    del self.pending[name]
            except Exception as exc:
                self.get_logger().warning(f'Parameter snapshot retry for {name}: {exc}')
                self.pending.pop(name, None)

    def destroy_node(self):
        if hasattr(self, 'store'):
            self.store.metadata['closed_utc'] = datetime.now(timezone.utc).isoformat()
            self.store.metadata['missing_parameter_snapshots'] = [
                name for name in self.parameter_clients if name not in self.store.metadata['parameter_snapshots']]
            self.store.save_metadata()
            self.store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathControlLogger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
