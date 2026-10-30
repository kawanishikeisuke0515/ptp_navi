import csv
from dataclasses import dataclass
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from typing import Optional

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import MagneticField

from .zed_heading_publisher import calculate_heading
from .zed_heading_publisher import get_axis_parameter
from .zed_heading_publisher import get_axis_value
from .zed_heading_publisher import get_bool_parameter
from .zed_heading_publisher import is_finite
from .zed_heading_publisher import normalize_to_180


def default_output_config_path() -> Path:
    # Resolving __file__ also finds the source tree in a symlink install.
    source_config = Path(__file__).resolve().parent.parent / 'config' / 'zed_heading_publisher.yaml'
    if source_config.is_file():
        return source_config
    return Path(get_package_share_directory('zed_heading_publisher')) / 'config' / 'zed_heading_publisher.yaml'


def update_calibration_config(config_path: Path, parameters: dict) -> None:
    # Resolve symlinks so atomic replacement updates the actual source config.
    config_path = config_path.expanduser().resolve(strict=True)
    data = yaml.safe_load(config_path.read_text())
    existing = data['zed_heading_publisher']['ros__parameters']
    if not isinstance(existing, dict):
        raise ValueError('zed_heading_publisher.ros__parameters must be a mapping')
    existing.update(parameters)
    content = yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=config_path.parent,
                                         prefix=config_path.name + '.', delete=False) as stream:
            temporary_path = Path(stream.name)
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.chmod(config_path.stat().st_mode & 0o777)
        os.replace(temporary_path, config_path)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


@dataclass(frozen=True)
class SoftIronCalibration:
    center_x: float
    center_z: float
    x_radius: float
    z_radius: float
    covariance_major: float
    covariance_minor: float
    soft_iron_matrix_00: float
    soft_iron_matrix_01: float
    soft_iron_matrix_10: float
    soft_iron_matrix_11: float

    @property
    def covariance_ratio_major_over_minor(self) -> float:
        if self.covariance_minor == 0.0:
            return math.inf
        return self.covariance_major / self.covariance_minor


@dataclass(frozen=True)
class CalibrationSample:
    stamp_sec: float
    raw_x: float
    raw_z: float
    accumulated_vio_yaw_deg: Optional[float] = None


def _normalize_vector(x_value: float, z_value: float) -> tuple[float, float]:
    norm = math.hypot(x_value, z_value)
    if norm == 0.0:
        raise ValueError('zero-length eigenvector')
    return x_value / norm, z_value / norm


def _eigenvector_for_symmetric_2x2(
    xx_value: float,
    xz_value: float,
    zz_value: float,
    eigenvalue: float,
) -> tuple[float, float]:
    if abs(xz_value) > 1e-12:
        return _normalize_vector(xz_value, eigenvalue - xx_value)
    if xx_value >= zz_value:
        return (1.0, 0.0)
    return (0.0, 1.0)


def estimate_soft_iron_calibration(
    raw_samples: list[tuple[float, float]],
) -> SoftIronCalibration:
    if len(raw_samples) < 2:
        raise ValueError('at least 2 samples are required for soft-iron calibration')

    raw_x_values = [sample[0] for sample in raw_samples]
    raw_z_values = [sample[1] for sample in raw_samples]
    min_x = min(raw_x_values)
    max_x = max(raw_x_values)
    min_z = min(raw_z_values)
    max_z = max(raw_z_values)
    center_x = (min_x + max_x) / 2.0
    center_z = (min_z + max_z) / 2.0
    x_radius = (max_x - min_x) / 2.0
    z_radius = (max_z - min_z) / 2.0

    if x_radius <= 0.0 or z_radius <= 0.0:
        raise ValueError('sample range must be non-zero on both raw_x and raw_z')

    shifted_samples = [
        (raw_x - center_x, raw_z - center_z) for raw_x, raw_z in raw_samples
    ]
    cov_xx = sum(raw_x * raw_x for raw_x, _ in shifted_samples) / len(shifted_samples)
    cov_xz = (
        sum(raw_x * raw_z for raw_x, raw_z in shifted_samples) / len(shifted_samples)
    )
    cov_zz = sum(raw_z * raw_z for _, raw_z in shifted_samples) / len(shifted_samples)

    trace = cov_xx + cov_zz
    discriminant = math.sqrt((cov_xx - cov_zz) ** 2 + 4.0 * cov_xz * cov_xz)
    major = (trace + discriminant) / 2.0
    minor = (trace - discriminant) / 2.0
    if minor <= 0.0 or not is_finite(major, minor):
        raise ValueError('sample covariance is degenerate')

    major_vec_x, major_vec_z = _eigenvector_for_symmetric_2x2(
        cov_xx, cov_xz, cov_zz, major
    )
    minor_vec_x = -major_vec_z
    minor_vec_z = major_vec_x
    target_variance = math.sqrt(major * minor)
    major_scale = math.sqrt(target_variance / major)
    minor_scale = math.sqrt(target_variance / minor)

    matrix_00 = (
        major_scale * major_vec_x * major_vec_x
        + minor_scale * minor_vec_x * minor_vec_x
    )
    matrix_01 = (
        major_scale * major_vec_x * major_vec_z
        + minor_scale * minor_vec_x * minor_vec_z
    )
    matrix_10 = (
        major_scale * major_vec_z * major_vec_x
        + minor_scale * minor_vec_z * minor_vec_x
    )
    matrix_11 = (
        major_scale * major_vec_z * major_vec_z
        + minor_scale * minor_vec_z * minor_vec_z
    )

    return SoftIronCalibration(
        center_x=center_x,
        center_z=center_z,
        x_radius=x_radius,
        z_radius=z_radius,
        covariance_major=major,
        covariance_minor=minor,
        soft_iron_matrix_00=matrix_00,
        soft_iron_matrix_01=matrix_01,
        soft_iron_matrix_10=matrix_10,
        soft_iron_matrix_11=matrix_11,
    )


def _optional_float(value: Optional[str]) -> Optional[float]:
    if value is None or value == '':
        return None
    return float(value)


def read_calibration_samples_csv(csv_path: Path) -> list[CalibrationSample]:
    samples: list[CalibrationSample] = []
    with csv_path.open(newline='') as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError('CSV file has no header')
        missing_fields = {'raw_x', 'raw_z'} - set(reader.fieldnames)
        if missing_fields:
            raise ValueError(
                'CSV file must contain raw_x and raw_z columns: missing %s'
                % ', '.join(sorted(missing_fields))
            )
        for index, row in enumerate(reader):
            raw_x = float(row['raw_x'])
            raw_z = float(row['raw_z'])
            if is_finite(raw_x, raw_z):
                stamp_sec = _optional_float(row.get('stamp_sec'))
                accumulated_yaw = _optional_float(row.get('accumulated_vio_yaw_deg'))
                samples.append(
                    CalibrationSample(
                        float(index) if stamp_sec is None else stamp_sec,
                        raw_x,
                        raw_z,
                        accumulated_yaw,
                    )
                )
    return samples


def read_raw_samples_csv(csv_path: Path) -> list[tuple[float, float]]:
    return [
        (sample.raw_x, sample.raw_z)
        for sample in read_calibration_samples_csv(csv_path)
    ]


def quaternion_to_yaw_rad(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def get_angle_bin_index(accumulated_yaw_deg: float, bin_size_deg: float) -> int:
    wrapped_yaw_deg = abs(accumulated_yaw_deg) % 360.0
    return int(wrapped_yaw_deg // bin_size_deg)


class ZedHeadingCalibrator(Node):
    def __init__(self):
        super().__init__('calibrate_zed_heading')

        self.declare_parameter('mag_topic', '/zed2i/zed_node/imu/mag')
        self.declare_parameter('odom_topic', '/zed2i/zed_node/odom')
        self.declare_parameter('use_vio_turn_detection', True)
        self.declare_parameter('duration_sec', 30.0)
        self.declare_parameter('max_duration_sec', 90.0)
        self.declare_parameter('target_rotation_deg', 360.0)
        self.declare_parameter('angle_bin_size_deg', 2.0)
        self.declare_parameter('min_bin_coverage_ratio', 0.7)
        self.declare_parameter('min_samples', 20)
        self.declare_parameter('magnetic_field_scale', 1000000.0)
        axis_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('raw_x_axis', 'y', axis_descriptor)
        self.declare_parameter('raw_x_sign', -1.0)
        self.declare_parameter('raw_z_axis', 'x', axis_descriptor)
        self.declare_parameter('raw_z_sign', 1.0)
        self.declare_parameter('progress_log_interval_sec', 2.0)
        self.declare_parameter('sample_csv_path', '')
        self.declare_parameter('input_csv_path', '')
        self.declare_parameter('save_config', True)
        self.declare_parameter('output_config_path', '')

        self.mag_topic = str(self.get_parameter('mag_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.use_vio_turn_detection = get_bool_parameter(
            self.get_parameter('use_vio_turn_detection').value
        )
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.max_duration_sec = float(self.get_parameter('max_duration_sec').value)
        self.target_rotation_deg = float(
            self.get_parameter('target_rotation_deg').value
        )
        self.angle_bin_size_deg = float(
            self.get_parameter('angle_bin_size_deg').value
        )
        self.min_bin_coverage_ratio = float(
            self.get_parameter('min_bin_coverage_ratio').value
        )
        self.min_samples = int(self.get_parameter('min_samples').value)
        self.magnetic_field_scale = float(
            self.get_parameter('magnetic_field_scale').value
        )
        self.raw_x_axis = get_axis_parameter(self.get_parameter('raw_x_axis').value)
        self.raw_x_sign = float(self.get_parameter('raw_x_sign').value)
        self.raw_z_axis = get_axis_parameter(self.get_parameter('raw_z_axis').value)
        self.raw_z_sign = float(self.get_parameter('raw_z_sign').value)
        self.progress_log_interval_sec = float(
            self.get_parameter('progress_log_interval_sec').value
        )
        self.sample_csv_path = str(self.get_parameter('sample_csv_path').value)
        self.input_csv_path = str(self.get_parameter('input_csv_path').value)
        self.save_config = get_bool_parameter(self.get_parameter('save_config').value)
        self.output_config_path = str(self.get_parameter('output_config_path').value)
        self._validate_parameters()

        self.sample_count = 0
        self.min_x = math.inf
        self.max_x = -math.inf
        self.min_z = math.inf
        self.max_z = -math.inf
        self.last_raw_x: Optional[float] = None
        self.last_raw_z: Optional[float] = None
        self.zero_raw_x: Optional[float] = None
        self.zero_raw_z: Optional[float] = None
        self.samples: list[CalibrationSample] = []
        self.angle_bin_samples: dict[int, tuple[float, float]] = {}
        self.vio_zero_yaw_deg: Optional[float] = None
        self.previous_vio_yaw_deg: Optional[float] = None
        self.accumulated_vio_yaw_deg = 0.0
        self.finish_reason = ''
        self.start_time = self.get_clock().now()
        self.last_progress_log_time = self.start_time
        self.finished = False

        if self.input_csv_path:
            self.create_timer(0.1, self.input_csv_timer_callback)
            self.get_logger().info(
                'loading heading calibration samples from %s' % self.input_csv_path
            )
            return

        if self.use_vio_turn_detection:
            self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                qos_profile_sensor_data,
            )

        self.create_subscription(
            MagneticField,
            self.mag_topic,
            self.mag_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'collecting heading calibration samples from %s; '
            'mode=%s max_duration=%.1f sec scale=%.5f '
            'raw_x=%+.1f*%s raw_z=%+.1f*%s'
            % (
                self.mag_topic,
                'vio_turn' if self.use_vio_turn_detection else 'duration',
                self.max_duration_sec
                if self.use_vio_turn_detection
                else self.duration_sec,
                self.magnetic_field_scale,
                self.raw_x_sign,
                self.raw_x_axis,
                self.raw_z_sign,
                self.raw_z_axis,
            )
        )
        if self.use_vio_turn_detection:
            self.get_logger().info(
                'start at robot yaw=0, then rotate slowly until %.1f deg is reached'
                % self.target_rotation_deg
            )
        else:
            self.get_logger().info(
                'rotate the robot slowly through 360 degrees; keep it at yaw=0 near the end'
            )

    def _validate_parameters(self):
        if self.mag_topic == '':
            raise ValueError('mag_topic must not be empty')
        if self.odom_topic == '':
            raise ValueError('odom_topic must not be empty')
        if self.duration_sec <= 0.0 or not math.isfinite(self.duration_sec):
            raise ValueError('duration_sec must be a finite value greater than 0')
        if self.max_duration_sec <= 0.0 or not math.isfinite(self.max_duration_sec):
            raise ValueError('max_duration_sec must be a finite value greater than 0')
        if self.target_rotation_deg <= 0.0 or not math.isfinite(
            self.target_rotation_deg
        ):
            raise ValueError(
                'target_rotation_deg must be a finite value greater than 0'
            )
        if (
            self.angle_bin_size_deg <= 0.0
            or self.angle_bin_size_deg > 360.0
            or not math.isfinite(self.angle_bin_size_deg)
        ):
            raise ValueError('angle_bin_size_deg must be in the range (0, 360]')
        if (
            self.min_bin_coverage_ratio <= 0.0
            or self.min_bin_coverage_ratio > 1.0
            or not math.isfinite(self.min_bin_coverage_ratio)
        ):
            raise ValueError(
                'min_bin_coverage_ratio must be a finite value in the range (0, 1]'
            )
        if self.min_samples < 1:
            raise ValueError('min_samples must be greater than or equal to 1')
        if not math.isfinite(self.magnetic_field_scale):
            raise ValueError('magnetic_field_scale must be finite')
        if not is_finite(
            self.raw_x_sign,
            self.raw_z_sign,
            self.progress_log_interval_sec,
        ):
            raise ValueError('numeric parameters must be finite')
        if self.raw_x_sign == 0.0 or self.raw_z_sign == 0.0:
            raise ValueError('raw_x_sign and raw_z_sign must be non-zero')
        if self.progress_log_interval_sec < 0.0:
            raise ValueError(
                'progress_log_interval_sec must be greater than or equal to 0'
            )
        if self.sample_csv_path and self.input_csv_path:
            raise ValueError('sample_csv_path and input_csv_path cannot both be set')

    def odom_callback(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        vio_yaw_deg = math.degrees(
            quaternion_to_yaw_rad(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
        )
        if self.vio_zero_yaw_deg is None or self.previous_vio_yaw_deg is None:
            self.vio_zero_yaw_deg = vio_yaw_deg
            self.previous_vio_yaw_deg = vio_yaw_deg
            self.get_logger().info(
                'VIO zero reference set: vio=%.3f deg' % self.vio_zero_yaw_deg
            )
            return

        yaw_delta_deg = normalize_to_180(vio_yaw_deg - self.previous_vio_yaw_deg)
        self.accumulated_vio_yaw_deg += yaw_delta_deg
        self.previous_vio_yaw_deg = vio_yaw_deg

    def mag_callback(self, msg: MagneticField):
        if self.finished:
            return
        if self.use_vio_turn_detection and self.vio_zero_yaw_deg is None:
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
            self.get_logger().warn(
                'skipping invalid sample: raw_x=%s raw_z=%s' % (raw_x, raw_z)
            )
            return

        self.sample_count += 1
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        accumulated_yaw = (
            self.accumulated_vio_yaw_deg if self.use_vio_turn_detection else None
        )
        self.samples.append(CalibrationSample(stamp_sec, raw_x, raw_z, accumulated_yaw))
        if self.zero_raw_x is None or self.zero_raw_z is None:
            self.zero_raw_x = raw_x
            self.zero_raw_z = raw_z
        if accumulated_yaw is not None:
            self.angle_bin_samples[
                get_angle_bin_index(accumulated_yaw, self.angle_bin_size_deg)
            ] = (raw_x, raw_z)
        self.min_x = min(self.min_x, raw_x)
        self.max_x = max(self.max_x, raw_x)
        self.min_z = min(self.min_z, raw_z)
        self.max_z = max(self.max_z, raw_z)
        self.last_raw_x = raw_x
        self.last_raw_z = raw_z

    def timer_callback(self):
        if self.finished:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if self.progress_log_interval_sec > 0.0:
            progress_elapsed = (now - self.last_progress_log_time).nanoseconds / 1e9
            if progress_elapsed >= self.progress_log_interval_sec:
                self.last_progress_log_time = now
                self._log_progress(elapsed)

        if self.use_vio_turn_detection:
            if self._has_reached_target_rotation():
                self.finished = True
                self.finish_reason = 'target rotation reached'
                self._finish()
                return
            if elapsed >= self.max_duration_sec:
                self.finished = True
                self.finish_reason = 'max duration reached'
                self.get_logger().warn(
                    'max_duration_sec reached before target rotation/coverage'
                )
                self._finish()
                return

        if not self.use_vio_turn_detection and elapsed >= self.duration_sec:
            self.finished = True
            self.finish_reason = 'duration reached'
            self._finish()

    def input_csv_timer_callback(self):
        if self.finished:
            return
        self.finished = True
        try:
            self.samples = read_calibration_samples_csv(Path(self.input_csv_path))
            self.sample_count = len(self.samples)
            if self.samples:
                self.last_raw_x = self.samples[-1].raw_x
                self.last_raw_z = self.samples[-1].raw_z
                self.zero_raw_x = self.samples[0].raw_x
                self.zero_raw_z = self.samples[0].raw_z
                self.min_x = min(sample.raw_x for sample in self.samples)
                self.max_x = max(sample.raw_x for sample in self.samples)
                self.min_z = min(sample.raw_z for sample in self.samples)
                self.max_z = max(sample.raw_z for sample in self.samples)
                for sample in self.samples:
                    if sample.accumulated_vio_yaw_deg is not None:
                        self.angle_bin_samples[
                            get_angle_bin_index(
                                sample.accumulated_vio_yaw_deg,
                                self.angle_bin_size_deg,
                            )
                        ] = (sample.raw_x, sample.raw_z)
            self._finish()
        except Exception as exc:
            self.get_logger().error(
                'failed to load calibration CSV %s: %s'
                % (self.input_csv_path, exc)
            )

    def _has_reached_target_rotation(self) -> bool:
        return (
            abs(self.accumulated_vio_yaw_deg) >= self.target_rotation_deg
            and self._bin_coverage_ratio() >= self.min_bin_coverage_ratio
        )

    def _total_bin_count(self) -> int:
        return max(1, math.ceil(360.0 / self.angle_bin_size_deg))

    def _bin_coverage_ratio(self) -> float:
        return len(self.angle_bin_samples) / self._total_bin_count()

    def _estimation_raw_samples(self) -> list[tuple[float, float]]:
        if self.use_vio_turn_detection and self.angle_bin_samples:
            return list(self.angle_bin_samples.values())
        return [(sample.raw_x, sample.raw_z) for sample in self.samples]

    def _log_progress(self, elapsed: float):
        max_duration = (
            self.max_duration_sec if self.use_vio_turn_detection else self.duration_sec
        )
        remaining = max(0.0, max_duration - elapsed)
        if self.sample_count == 0:
            self.get_logger().info(
                'collecting... samples=0 remaining=%.1f sec' % remaining
            )
            return

        if self.use_vio_turn_detection:
            self.get_logger().info(
                'collecting... samples=%d rotation=%.1f/%.1f deg '
                'bins=%d/%d coverage=%.1f%% remaining=%.1f sec '
                'raw_x=[%.5f, %.5f] raw_z=[%.5f, %.5f]'
                % (
                    self.sample_count,
                    self.accumulated_vio_yaw_deg,
                    self.target_rotation_deg,
                    len(self.angle_bin_samples),
                    self._total_bin_count(),
                    self._bin_coverage_ratio() * 100.0,
                    remaining,
                    self.min_x,
                    self.max_x,
                    self.min_z,
                    self.max_z,
                )
            )
            return

        self.get_logger().info(
            'collecting... samples=%d remaining=%.1f sec '
            'raw_x=[%.5f, %.5f] raw_z=[%.5f, %.5f]'
            % (
                self.sample_count,
                remaining,
                self.min_x,
                self.max_x,
                self.min_z,
                self.max_z,
            )
        )

    def _finish(self):
        if (
            self.sample_count == 0
            or self.zero_raw_x is None
            or self.zero_raw_z is None
            or self.last_raw_x is None
            or self.last_raw_z is None
        ):
            self.get_logger().error(
                'no samples received for calibration; check mag_topic and ZED data hub'
            )
            return
        if self.sample_count < self.min_samples:
            self.get_logger().warn(
                'sample count is lower than recommended: samples=%d min_samples=%d'
                % (self.sample_count, self.min_samples)
            )

        if self.sample_csv_path:
            self._write_samples_csv(Path(self.sample_csv_path))

        raw_samples = self._estimation_raw_samples()
        try:
            calibration = estimate_soft_iron_calibration(raw_samples)
        except ValueError as exc:
            self.get_logger().error('failed to estimate soft-iron calibration: %s' % exc)
            return

        zero_sample = calculate_heading(
            raw_x=self.zero_raw_x,
            raw_z=self.zero_raw_z,
            center_x=calibration.center_x,
            center_z=calibration.center_z,
            zero_heading_deg=0.0,
            soft_iron_matrix_00=calibration.soft_iron_matrix_00,
            soft_iron_matrix_01=calibration.soft_iron_matrix_01,
            soft_iron_matrix_10=calibration.soft_iron_matrix_10,
            soft_iron_matrix_11=calibration.soft_iron_matrix_11,
        )
        zero_heading_deg = zero_sample.magnetic_heading_deg

        if self.save_config:
            if self.sample_count < self.min_samples or self.finish_reason == 'max duration reached':
                self.get_logger().warn(
                    'config not updated: insufficient samples or incomplete VIO rotation'
                )
            else:
                parameters = {
                    'mag_topic': self.mag_topic,
                    'center_x': calibration.center_x,
                    'center_z': calibration.center_z,
                    'soft_iron_matrix_00': calibration.soft_iron_matrix_00,
                    'soft_iron_matrix_01': calibration.soft_iron_matrix_01,
                    'soft_iron_matrix_10': calibration.soft_iron_matrix_10,
                    'soft_iron_matrix_11': calibration.soft_iron_matrix_11,
                    'zero_heading_deg': zero_heading_deg,
                    'magnetic_field_scale': self.magnetic_field_scale,
                    'raw_x_axis': self.raw_x_axis,
                    'raw_x_sign': self.raw_x_sign,
                    'raw_z_axis': self.raw_z_axis,
                    'raw_z_sign': self.raw_z_sign,
                }
                try:
                    config_path = (Path(self.output_config_path) if self.output_config_path
                                   else default_output_config_path())
                    update_calibration_config(config_path, parameters)
                    self.get_logger().info('updated calibration config: %s' % config_path)
                except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
                    self.get_logger().error('failed to save calibration config: %s' % exc)

        self.get_logger().info('calibration complete')
        print('', flush=True)
        print('zed_heading_publisher:', flush=True)
        print('  ros__parameters:', flush=True)
        print(f'    mag_topic: {self.mag_topic}', flush=True)
        print(f'    center_x: {calibration.center_x:.6f}', flush=True)
        print(f'    center_z: {calibration.center_z:.6f}', flush=True)
        print(
            f'    soft_iron_matrix_00: {calibration.soft_iron_matrix_00:.9f}',
            flush=True,
        )
        print(
            f'    soft_iron_matrix_01: {calibration.soft_iron_matrix_01:.9f}',
            flush=True,
        )
        print(
            f'    soft_iron_matrix_10: {calibration.soft_iron_matrix_10:.9f}',
            flush=True,
        )
        print(
            f'    soft_iron_matrix_11: {calibration.soft_iron_matrix_11:.9f}',
            flush=True,
        )
        print(f'    zero_heading_deg: {zero_heading_deg:.6f}', flush=True)
        print('    publish_rate_hz: 20.0', flush=True)
        print('    frame_id: "zed2i_mag"', flush=True)
        print('    invert_yaw: false', flush=True)
        print(f'    magnetic_field_scale: {self.magnetic_field_scale:.6f}', flush=True)
        print('    diagnostic_log_interval_sec: 1.0', flush=True)
        print(f'    raw_x_axis: "{self.raw_x_axis}"', flush=True)
        print(f'    raw_x_sign: {self.raw_x_sign:.1f}', flush=True)
        print(f'    raw_z_axis: "{self.raw_z_axis}"', flush=True)
        print(f'    raw_z_sign: {self.raw_z_sign:.1f}', flush=True)
        print('', flush=True)
        print('# diagnostics:', flush=True)
        print(f'# samples: {self.sample_count}', flush=True)
        print(f'# estimation_samples: {len(raw_samples)}', flush=True)
        if self.finish_reason:
            print(f'# finish_reason: {self.finish_reason}', flush=True)
        if self.use_vio_turn_detection:
            print(f'# odom_topic: {self.odom_topic}', flush=True)
            print(
                f'# accumulated_vio_yaw_deg: {self.accumulated_vio_yaw_deg:.6f}',
                flush=True,
            )
            print(
                '# angle_bin_coverage_ratio: %.6f'
                % self._bin_coverage_ratio(),
                flush=True,
            )
        print(f'# raw_x_min: {self.min_x:.6f}', flush=True)
        print(f'# raw_x_max: {self.max_x:.6f}', flush=True)
        print(f'# raw_z_min: {self.min_z:.6f}', flush=True)
        print(f'# raw_z_max: {self.max_z:.6f}', flush=True)
        print(f'# raw_x_radius: {calibration.x_radius:.6f}', flush=True)
        print(f'# raw_z_radius: {calibration.z_radius:.6f}', flush=True)
        print(
            '# radius_ratio_x_over_z: %.6f'
            % (
                calibration.x_radius / calibration.z_radius
                if calibration.z_radius != 0.0
                else math.inf
            ),
            flush=True,
        )
        print(
            '# covariance_ratio_major_over_minor: %.6f'
            % calibration.covariance_ratio_major_over_minor,
            flush=True,
        )
        if self.sample_csv_path:
            print(f'# sample_csv_path: {self.sample_csv_path}', flush=True)

    def _write_samples_csv(self, csv_path: Path):
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open('w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                'stamp_sec',
                'raw_x',
                'raw_z',
                'accumulated_vio_yaw_deg',
            ])
            for sample in self.samples:
                writer.writerow(
                    [
                        f'{sample.stamp_sec:.9f}',
                        f'{sample.raw_x:.9f}',
                        f'{sample.raw_z:.9f}',
                        ''
                        if sample.accumulated_vio_yaw_deg is None
                        else f'{sample.accumulated_vio_yaw_deg:.9f}',
                    ]
                )
        self.get_logger().info(
            'wrote %d calibration samples to %s'
            % (len(self.samples), str(csv_path))
        )


def main(args: Optional[list] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = ZedHeadingCalibrator()
        # Shutdown only after the active callback has returned.
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
