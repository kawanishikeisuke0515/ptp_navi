"""CSV logger for UWB distances, trilateration, and OptiTrack pose."""

import csv
import math
import os
import time
from datetime import datetime
from typing import Optional
from typing import Sequence

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from uwb_interfaces.msg import UwbDistances

from uwb_position_publisher.trilateration import trilaterate_2d


def stamp_to_sec(stamp) -> float:
    """Convert a ROS time stamp message to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class UwbOptitrackLogger(Node):
    """Log UWB distances, UWB estimated position, and OptiTrack pose."""

    def __init__(self):
        """Initialize subscriptions, CSV output, and periodic logging."""
        super().__init__('uwb_optitrack_logger')

        self.declare_parameter('output_dir', '/tmp/uwb_optitrack_logs')
        self.declare_parameter('log_rate', 20.0)
        self.declare_parameter('flush_every_rows', 20)
        self.declare_parameter('uwb_topic', '/uwb/distances')
        self.declare_parameter(
            'optitrack_pose_topic',
            '/vrpn_mocap/RigidBody_1/pose',
        )
        self.declare_parameter('anchor_1_x', 0.0)
        self.declare_parameter('anchor_1_y', 0.0)
        self.declare_parameter('anchor_2_x', 1.0)
        self.declare_parameter('anchor_2_y', 0.0)
        self.declare_parameter('anchor_3_x', 0.0)
        self.declare_parameter('anchor_3_y', 1.0)
        for i in range(1, 4):
            self.declare_parameter(f'anchor_{i}_height_difference_m', 0.0)
        self.declare_parameter('min_anchor_determinant', 1.0e-9)

        self.output_dir = str(self.get_parameter('output_dir').value)
        self.log_rate = float(self.get_parameter('log_rate').value)
        self.flush_every_rows = int(
            self.get_parameter('flush_every_rows').value
        )
        self.uwb_topic = str(self.get_parameter('uwb_topic').value)
        self.optitrack_pose_topic = str(
            self.get_parameter('optitrack_pose_topic').value
        )
        self.anchor_1 = (
            float(self.get_parameter('anchor_1_x').value),
            float(self.get_parameter('anchor_1_y').value),
        )
        self.anchor_2 = (
            float(self.get_parameter('anchor_2_x').value),
            float(self.get_parameter('anchor_2_y').value),
        )
        self.anchor_3 = (
            float(self.get_parameter('anchor_3_x').value),
            float(self.get_parameter('anchor_3_y').value),
        )
        self.height_differences = tuple(
            float(self.get_parameter(f'anchor_{i}_height_difference_m').value)
            for i in range(1, 4)
        )
        self.min_anchor_determinant = float(
            self.get_parameter('min_anchor_determinant').value
        )
        self._validate_parameters()

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'uwb_optitrack_log_{stamp}.csv',
        )

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'elapsed_sec',
            'uwb_stamp_sec',
            'uwb_device_time_ms',
            'anchor_1_distance_m',
            'anchor_2_distance_m',
            'anchor_3_distance_m',
            'anchor_1_valid',
            'anchor_2_valid',
            'anchor_3_valid',
            'uwb_raw_line',
            'uwb_position_valid',
            'uwb_estimated_x',
            'uwb_estimated_y',
            'optitrack_stamp_sec',
            'optitrack_x',
            'optitrack_y',
            'optitrack_z',
            'optitrack_qx',
            'optitrack_qy',
            'optitrack_qz',
            'optitrack_qw',
        ])

        self.start_time = time.perf_counter()
        self.row_count = 0
        self.latest_uwb: Optional[UwbDistances] = None
        self.latest_optitrack_pose: Optional[PoseStamped] = None

        optitrack_best_effort_qos = QoSProfile(depth=10)
        optitrack_best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            UwbDistances,
            self.uwb_topic,
            self.uwb_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.optitrack_pose_topic,
            self.optitrack_pose_callback,
            optitrack_best_effort_qos,
        )
        self.create_timer(1.0 / self.log_rate, self.log_callback)

        self.get_logger().info(
            'logging %s, %s, and UWB trilateration to %s'
            % (self.uwb_topic, self.optitrack_pose_topic, self.csv_path)
        )

    def _validate_parameters(self):
        if not all(math.isfinite(h) for h in self.height_differences):
            raise ValueError('anchor height differences must be finite')
        if self.log_rate <= 0.0:
            raise ValueError('log_rate must be greater than 0')
        if self.flush_every_rows <= 0:
            raise ValueError('flush_every_rows must be greater than 0')
        if self.output_dir == '':
            raise ValueError('output_dir must not be empty')
        if self.uwb_topic == '':
            raise ValueError('uwb_topic must not be empty')
        if self.optitrack_pose_topic == '':
            raise ValueError('optitrack_pose_topic must not be empty')
        if self.min_anchor_determinant <= 0.0:
            raise ValueError('min_anchor_determinant must be greater than 0')

    def uwb_callback(self, msg: UwbDistances):
        """Store the latest UWB distance message."""
        self.latest_uwb = msg

    def optitrack_pose_callback(self, msg: PoseStamped):
        """Store the latest OptiTrack pose message."""
        self.latest_optitrack_pose = msg

    def log_callback(self):
        """Write one CSV row with the latest available samples."""
        elapsed_sec = time.perf_counter() - self.start_time
        uwb = self.latest_uwb
        optitrack_pose = self.latest_optitrack_pose

        estimated_position = self._estimate_uwb_position(uwb)
        uwb_position_valid = estimated_position is not None
        if estimated_position is not None:
            estimated_x, estimated_y = estimated_position
        else:
            estimated_x = None
            estimated_y = None

        optitrack_position = (
            optitrack_pose.pose.position
            if optitrack_pose is not None
            else None
        )
        optitrack_orientation = (
            optitrack_pose.pose.orientation
            if optitrack_pose is not None
            else None
        )
        uwb_stamp = uwb.header.stamp if uwb is not None else None
        optitrack_stamp = (
            optitrack_pose.header.stamp
            if optitrack_pose is not None
            else None
        )
        optitrack_x = (
            optitrack_position.x if optitrack_position is not None else None
        )
        optitrack_y = (
            optitrack_position.y if optitrack_position is not None else None
        )
        optitrack_z = (
            optitrack_position.z if optitrack_position is not None else None
        )
        optitrack_qx = (
            optitrack_orientation.x
            if optitrack_orientation is not None
            else None
        )
        optitrack_qy = (
            optitrack_orientation.y
            if optitrack_orientation is not None
            else None
        )
        optitrack_qz = (
            optitrack_orientation.z
            if optitrack_orientation is not None
            else None
        )
        optitrack_qw = (
            optitrack_orientation.w
            if optitrack_orientation is not None
            else None
        )

        self.writer.writerow([
            '%.4f' % elapsed_sec,
            self._format_optional_stamp(uwb_stamp),
            str(uwb.device_time_ms) if uwb is not None else '',
            self._format_optional(
                uwb.anchor_1_distance_m if uwb is not None else None
            ),
            self._format_optional(
                uwb.anchor_2_distance_m if uwb is not None else None
            ),
            self._format_optional(
                uwb.anchor_3_distance_m if uwb is not None else None
            ),
            self._format_optional_bool(
                uwb.anchor_1_valid if uwb is not None else None
            ),
            self._format_optional_bool(
                uwb.anchor_2_valid if uwb is not None else None
            ),
            self._format_optional_bool(
                uwb.anchor_3_valid if uwb is not None else None
            ),
            uwb.raw_line if uwb is not None else '',
            self._format_optional_bool(
                uwb_position_valid if uwb is not None else None
            ),
            self._format_optional(estimated_x),
            self._format_optional(estimated_y),
            self._format_optional_stamp(optitrack_stamp),
            self._format_optional(optitrack_x),
            self._format_optional(optitrack_y),
            self._format_optional(optitrack_z),
            self._format_optional(optitrack_qx),
            self._format_optional(optitrack_qy),
            self._format_optional(optitrack_qz),
            self._format_optional(optitrack_qw),
        ])

        self.row_count += 1
        if self.row_count % self.flush_every_rows == 0:
            self.csv_file.flush()

    def _estimate_uwb_position(
        self,
        uwb: Optional[UwbDistances],
    ) -> Optional[tuple[float, float]]:
        if uwb is None:
            return None
        anchors_valid = (
            uwb.anchor_1_valid
            and uwb.anchor_2_valid
            and uwb.anchor_3_valid
        )
        if not anchors_valid:
            return None

        return trilaterate_2d(
            self.anchor_1,
            self.anchor_2,
            self.anchor_3,
            float(uwb.anchor_1_distance_m),
            float(uwb.anchor_2_distance_m),
            float(uwb.anchor_3_distance_m),
            self.min_anchor_determinant,
            self.height_differences,
        )

    def _format_optional_stamp(self, stamp) -> str:
        if stamp is None:
            return ''
        return self._format_optional(stamp_to_sec(stamp))

    def _format_optional(self, value: Optional[float]) -> str:
        if value is None:
            return ''
        return '%.6f' % float(value)

    def _format_optional_bool(self, value: Optional[bool]) -> str:
        if value is None:
            return ''
        return 'true' if value else 'false'

    def close(self):
        """Flush and close the CSV file."""
        self.csv_file.flush()
        self.csv_file.close()
        print('saved log to %s' % self.csv_path)


def main(args: Optional[Sequence[str]] = None):
    """Run the UWB OptiTrack logger node."""
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = UwbOptitrackLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
