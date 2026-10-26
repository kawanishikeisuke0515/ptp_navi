from dataclasses import dataclass
import math
import os
from typing import Optional

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import MagneticField
from zed_interfaces.msg import ZedHeading


def normalize_to_180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class HeadingSample:
    raw_x: float
    raw_z: float
    corrected_x: float
    corrected_z: float
    magnetic_heading_deg: float
    robot_yaw_deg: float
    robot_yaw_rad: float


def calculate_heading(
    raw_x: float,
    raw_z: float,
    center_x: float,
    center_z: float,
    zero_heading_deg: float,
    invert_yaw: bool = False,
    soft_iron_matrix_00: float = 1.0,
    soft_iron_matrix_01: float = 0.0,
    soft_iron_matrix_10: float = 0.0,
    soft_iron_matrix_11: float = 1.0,
) -> HeadingSample:
    shifted_x = raw_x - center_x
    shifted_z = raw_z - center_z
    corrected_x = soft_iron_matrix_00 * shifted_x + soft_iron_matrix_01 * shifted_z
    corrected_z = soft_iron_matrix_10 * shifted_x + soft_iron_matrix_11 * shifted_z

    magnetic_heading_deg = normalize_to_180(
        math.degrees(math.atan2(corrected_x, corrected_z))
    )
    robot_yaw_deg = magnetic_heading_deg - zero_heading_deg
    if invert_yaw:
        robot_yaw_deg = -robot_yaw_deg
    robot_yaw_deg = normalize_to_180(robot_yaw_deg)
    robot_yaw_rad = math.radians(robot_yaw_deg)

    return HeadingSample(
        raw_x=raw_x,
        raw_z=raw_z,
        corrected_x=corrected_x,
        corrected_z=corrected_z,
        magnetic_heading_deg=magnetic_heading_deg,
        robot_yaw_deg=robot_yaw_deg,
        robot_yaw_rad=robot_yaw_rad,
    )


def is_finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def get_bool_parameter(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('true', '1', 'yes', 'on'):
            return True
        if normalized in ('false', '0', 'no', 'off'):
            return False
    raise ValueError(f'invalid bool parameter value: {value}')


def get_axis_value(magnetic_field, axis: str) -> float:
    if axis == 'x':
        return float(magnetic_field.x)
    if axis == 'y':
        return float(magnetic_field.y)
    if axis == 'z':
        return float(magnetic_field.z)
    raise ValueError(f'invalid magnetic field axis: {axis}')


def get_axis_parameter(value) -> str:
    if isinstance(value, bool):
        if value:
            return 'y'
        raise ValueError('bool false is not a valid magnetic field axis')
    axis = str(value).strip().lower()
    if axis in ('x', 'y', 'z'):
        return axis
    raise ValueError(f'axis parameter must be one of x, y, or z: {value}')


class ZedHeadingPublisher(Node):
    def __init__(self):
        super().__init__('zed_heading_publisher')

        self.declare_parameter('mag_topic', '/zed2i/zed_node/imu/mag')
        self.declare_parameter('center_x', -0.629279)
        self.declare_parameter('center_z', -0.874388)
        self.declare_parameter('zero_heading_deg', 39.7)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'zed2i_mag')
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('magnetic_field_scale', 1000000.0)
        self.declare_parameter('diagnostic_log_interval_sec', 1.0)
        axis_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('raw_x_axis', 'y', axis_descriptor)
        self.declare_parameter('raw_x_sign', -1.0)
        self.declare_parameter('raw_z_axis', 'x', axis_descriptor)
        self.declare_parameter('raw_z_sign', 1.0)
        self.declare_parameter('soft_iron_matrix_00', 1.0)
        self.declare_parameter('soft_iron_matrix_01', 0.0)
        self.declare_parameter('soft_iron_matrix_10', 0.0)
        self.declare_parameter('soft_iron_matrix_11', 1.0)

        self.mag_topic = str(self.get_parameter('mag_topic').value)
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_z = float(self.get_parameter('center_z').value)
        self.zero_heading_deg = float(self.get_parameter('zero_heading_deg').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.invert_yaw = get_bool_parameter(self.get_parameter('invert_yaw').value)
        self.magnetic_field_scale = float(
            self.get_parameter('magnetic_field_scale').value
        )
        self.diagnostic_log_interval_sec = float(
            self.get_parameter('diagnostic_log_interval_sec').value
        )
        self.raw_x_axis = get_axis_parameter(self.get_parameter('raw_x_axis').value)
        self.raw_x_sign = float(self.get_parameter('raw_x_sign').value)
        self.raw_z_axis = get_axis_parameter(self.get_parameter('raw_z_axis').value)
        self.raw_z_sign = float(self.get_parameter('raw_z_sign').value)
        self.soft_iron_matrix_00 = float(
            self.get_parameter('soft_iron_matrix_00').value
        )
        self.soft_iron_matrix_01 = float(
            self.get_parameter('soft_iron_matrix_01').value
        )
        self.soft_iron_matrix_10 = float(
            self.get_parameter('soft_iron_matrix_10').value
        )
        self.soft_iron_matrix_11 = float(
            self.get_parameter('soft_iron_matrix_11').value
        )
        self._validate_parameters()

        self.publisher_ = self.create_publisher(ZedHeading, '/zed/heading', 10)
        self.last_sensor_warn_time = None
        self.last_publish_time = None
        self.last_diagnostic_log_time = None

        self.mag_subscription = self.create_subscription(
            MagneticField,
            self.mag_topic,
            self.mag_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'mag_topic=%s center_x=%.5f center_z=%.5f zero_heading_deg=%.5f '
            'publish_rate_hz=%.3f frame_id=%s invert_yaw=%s '
            'magnetic_field_scale=%.5f diagnostic_log_interval_sec=%.3f '
            'raw_x=%+.1f*%s raw_z=%+.1f*%s '
            'soft_iron=[[%.6f, %.6f], [%.6f, %.6f]]'
            % (
                self.mag_topic,
                self.center_x,
                self.center_z,
                self.zero_heading_deg,
                self.publish_rate_hz,
                self.frame_id,
                self.invert_yaw,
                self.magnetic_field_scale,
                self.diagnostic_log_interval_sec,
                self.raw_x_sign,
                self.raw_x_axis,
                self.raw_z_sign,
                self.raw_z_axis,
                self.soft_iron_matrix_00,
                self.soft_iron_matrix_01,
                self.soft_iron_matrix_10,
                self.soft_iron_matrix_11,
            )
        )
        self.get_logger().info('publishing ZED2i heading on /zed/heading')

    def _validate_parameters(self):
        if self.mag_topic == '':
            raise ValueError('mag_topic must not be empty')
        if not is_finite(self.center_x, self.center_z, self.zero_heading_deg):
            raise ValueError(
                'center_x, center_z, and zero_heading_deg must be finite values'
            )
        if self.publish_rate_hz <= 0.0 or not math.isfinite(self.publish_rate_hz):
            raise ValueError('publish_rate_hz must be a finite value greater than 0')
        if self.frame_id == '':
            raise ValueError('frame_id must not be empty')
        if not math.isfinite(self.magnetic_field_scale):
            raise ValueError('magnetic_field_scale must be finite')
        if (
            self.diagnostic_log_interval_sec < 0.0
            or not math.isfinite(self.diagnostic_log_interval_sec)
        ):
            raise ValueError(
                'diagnostic_log_interval_sec must be a finite value greater than or equal to 0'
            )
        if self.raw_x_axis not in ('x', 'y', 'z'):
            raise ValueError("raw_x_axis must be one of 'x', 'y', or 'z'")
        if self.raw_z_axis not in ('x', 'y', 'z'):
            raise ValueError("raw_z_axis must be one of 'x', 'y', or 'z'")
        if not is_finite(self.raw_x_sign, self.raw_z_sign):
            raise ValueError('raw_x_sign and raw_z_sign must be finite')
        if self.raw_x_sign == 0.0 or self.raw_z_sign == 0.0:
            raise ValueError('raw_x_sign and raw_z_sign must be non-zero')
        if not is_finite(
            self.soft_iron_matrix_00,
            self.soft_iron_matrix_01,
            self.soft_iron_matrix_10,
            self.soft_iron_matrix_11,
        ):
            raise ValueError('soft-iron matrix parameters must be finite')

    def mag_callback(self, msg: MagneticField):
        if not self._should_publish():
            return

        raw_x = (
            self.raw_x_sign
            * get_axis_value(msg.magnetic_field, self.raw_x_axis)
            * self.magnetic_field_scale
        )
        raw_z = (
            self.raw_z_sign
            * get_axis_value(msg.magnetic_field, self.raw_z_axis)
            * self.magnetic_field_scale
        )

        if not is_finite(raw_x, raw_z):
            self._warn_throttled(
                f'invalid magnetometer data: raw_x={raw_x} raw_z={raw_z}'
            )
            return

        sample = calculate_heading(
            raw_x=raw_x,
            raw_z=raw_z,
            center_x=self.center_x,
            center_z=self.center_z,
            zero_heading_deg=self.zero_heading_deg,
            invert_yaw=self.invert_yaw,
            soft_iron_matrix_00=self.soft_iron_matrix_00,
            soft_iron_matrix_01=self.soft_iron_matrix_01,
            soft_iron_matrix_10=self.soft_iron_matrix_10,
            soft_iron_matrix_11=self.soft_iron_matrix_11,
        )
        self._log_diagnostic_sample(msg, sample)
        self.publisher_.publish(self._build_message(sample))
        self.last_publish_time = self.get_clock().now()

    def _should_publish(self) -> bool:
        if self.last_publish_time is None:
            return True
        elapsed = (
            self.get_clock().now() - self.last_publish_time
        ).nanoseconds / 1e9
        return elapsed >= 1.0 / self.publish_rate_hz

    def _build_message(self, sample: HeadingSample) -> ZedHeading:
        msg = ZedHeading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.raw_x = sample.raw_x
        msg.raw_z = sample.raw_z
        msg.corrected_x = sample.corrected_x
        msg.corrected_z = sample.corrected_z
        msg.magnetic_heading_deg = sample.magnetic_heading_deg
        msg.robot_yaw_deg = sample.robot_yaw_deg
        msg.robot_yaw_rad = sample.robot_yaw_rad
        msg.valid = True
        return msg

    def _log_diagnostic_sample(
        self,
        source_msg: MagneticField,
        sample: HeadingSample,
    ):
        if self.diagnostic_log_interval_sec <= 0.0:
            return

        now = self.get_clock().now()
        if self.last_diagnostic_log_time is not None:
            elapsed = (now - self.last_diagnostic_log_time).nanoseconds / 1e9
            if elapsed < self.diagnostic_log_interval_sec:
                return

        self.last_diagnostic_log_time = now
        field = source_msg.magnetic_field
        self.get_logger().info(
            'mag_sensor=(x=%.8g y=%.8g z=%.8g) mapped=(raw_x=%.5f raw_z=%.5f) '
            'corrected=(x=%.5f z=%.5f) robot_yaw_deg=%.3f'
            % (
                float(field.x),
                float(field.y),
                float(field.z),
                sample.raw_x,
                sample.raw_z,
                sample.corrected_x,
                sample.corrected_z,
                sample.robot_yaw_deg,
            )
        )

    def _warn_throttled(self, message: str, interval_sec: float = 2.0):
        now = self.get_clock().now()
        if self.last_sensor_warn_time is not None:
            elapsed = (now - self.last_sensor_warn_time).nanoseconds / 1e9
            if elapsed < interval_sec:
                return

        self.last_sensor_warn_time = now
        self.get_logger().warn(message)


def main(args: Optional[list] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = ZedHeadingPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
