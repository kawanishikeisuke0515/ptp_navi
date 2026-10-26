import math
import os
from dataclasses import dataclass
from typing import Optional
from typing import Sequence
from typing import Tuple

import rclpy
from rclpy.node import Node
from uwb_interfaces.msg import UwbDistances

try:
    import serial
    from serial import SerialException
except ImportError:
    serial = None

    class SerialException(Exception):
        pass


@dataclass(frozen=True)
class ParsedUwbDistances:
    device_time_ms: int
    distance_cm: Tuple[int, int, int]
    raw_line: str


class UwbParseError(ValueError):
    pass


def normalize_line(line) -> str:
    if isinstance(line, bytes):
        text = line.decode('utf-8', errors='replace')
    else:
        text = str(line)

    text = text.strip()
    if text.startswith('(') and text.endswith(')'):
        text = text[1:-1].strip()
    return text


def parse_uwb_csv(line) -> ParsedUwbDistances:
    raw_line = normalize_line(line)
    if raw_line == '':
        raise UwbParseError('empty line')

    columns = [column.strip() for column in raw_line.split(',')]
    if len(columns) != 4:
        raise UwbParseError(f'expected 4 columns, got {len(columns)}')

    try:
        device_time_ms = int(columns[0])
        distances_cm = (int(columns[1]), int(columns[2]), int(columns[3]))
    except ValueError as exc:
        raise UwbParseError('failed to parse integer fields') from exc

    if device_time_ms < 0:
        raise UwbParseError('device_time_ms must be greater than or equal to 0')

    return ParsedUwbDistances(
        device_time_ms=device_time_ms,
        distance_cm=distances_cm,
        raw_line=raw_line,
    )


def distance_cm_to_m(
    distance_cm: int,
    invalid_distance_cm: int = 65535,
) -> Tuple[float, bool]:
    if distance_cm < 0 or distance_cm == invalid_distance_cm:
        return math.nan, False
    return float(distance_cm) / 100.0, True


def build_message(
    parsed: ParsedUwbDistances,
    stamp,
    frame_id: str,
    invalid_distance_cm: int = 65535,
) -> UwbDistances:
    distances = [
        distance_cm_to_m(distance_cm, invalid_distance_cm)
        for distance_cm in parsed.distance_cm
    ]

    msg = UwbDistances()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.device_time_ms = parsed.device_time_ms
    msg.anchor_1_distance_m = distances[0][0]
    msg.anchor_2_distance_m = distances[1][0]
    msg.anchor_3_distance_m = distances[2][0]
    msg.anchor_1_valid = distances[0][1]
    msg.anchor_2_valid = distances[1][1]
    msg.anchor_3_valid = distances[2][1]
    msg.raw_line = parsed.raw_line
    return msg


class UwbDistancePublisher(Node):
    def __init__(self):
        super().__init__('uwb_distance_publisher')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('read_timeout', 0.1)
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('invalid_distance_cm', 65535)
        self.declare_parameter('frame_id', 'uwb')
        self.declare_parameter('timer_period', 0.01)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.read_timeout = float(self.get_parameter('read_timeout').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value
        )
        self.invalid_distance_cm = int(
            self.get_parameter('invalid_distance_cm').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.timer_period = float(self.get_parameter('timer_period').value)
        self._validate_parameters()

        self.publisher_ = self.create_publisher(UwbDistances, '/uwb/distances', 10)
        self.serial_connection = None
        self.last_reconnect_attempt_time = None

        self.try_open_serial()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            'serial_port=%s baudrate=%d read_timeout=%.3f '
            'reconnect_interval=%.3f invalid_distance_cm=%d frame_id=%s'
            % (
                self.serial_port,
                self.baudrate,
                self.read_timeout,
                self.reconnect_interval,
                self.invalid_distance_cm,
                self.frame_id,
            )
        )
        self.get_logger().info('publishing UWB distances on /uwb/distances')

    def _validate_parameters(self):
        if self.baudrate <= 0:
            raise ValueError('baudrate must be greater than 0')
        if self.read_timeout < 0.0:
            raise ValueError('read_timeout must be greater than or equal to 0')
        if self.reconnect_interval <= 0.0:
            raise ValueError('reconnect_interval must be greater than 0')
        if self.timer_period <= 0.0:
            raise ValueError('timer_period must be greater than 0')
        if self.serial_port == '':
            raise ValueError('serial_port must not be empty')

    def try_open_serial(self) -> bool:
        self.last_reconnect_attempt_time = self.get_clock().now()
        if serial is None:
            self.get_logger().error(
                'python3-serial is not available; install python3-serial'
            )
            return False

        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.read_timeout,
            )
        except SerialException as exc:
            self.serial_connection = None
            self.get_logger().warn(
                f'failed to open serial port {self.serial_port}: {exc}'
            )
            return False

        self.get_logger().info(
            f'opened serial port {self.serial_port} at {self.baudrate} bps'
        )
        return True

    def close_serial(self):
        if self.serial_connection is None:
            return

        try:
            self.serial_connection.close()
        except SerialException as exc:
            self.get_logger().warn(f'failed to close serial port: {exc}')
        finally:
            self.serial_connection = None

    def timer_callback(self):
        if self.serial_connection is None:
            self._try_reconnect_if_due()
            return

        try:
            line = self.serial_connection.readline()
        except (OSError, SerialException) as exc:
            self.get_logger().warn(f'failed to read serial line: {exc}')
            self.close_serial()
            return

        normalized = normalize_line(line)
        if normalized == '':
            return

        try:
            parsed = parse_uwb_csv(normalized)
        except UwbParseError as exc:
            self.get_logger().warn(f'invalid UWB CSV: {exc}; raw_line="{normalized}"')
            return

        msg = build_message(
            parsed,
            self.get_clock().now().to_msg(),
            self.frame_id,
            self.invalid_distance_cm,
        )
        self.publisher_.publish(msg)

    def _try_reconnect_if_due(self):
        now = self.get_clock().now()
        if self.last_reconnect_attempt_time is not None:
            elapsed = now - self.last_reconnect_attempt_time
            if elapsed.nanoseconds * 1e-9 < self.reconnect_interval:
                return

        self.try_open_serial()

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args: Optional[Sequence[str]] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = UwbDistancePublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
