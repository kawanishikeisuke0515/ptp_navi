import math
import os
from collections import deque
from typing import Optional
from typing import Sequence
from typing import Tuple

import rclpy
from rclpy.node import Node
from uwb_interfaces.msg import UwbDistances
from uwb_interfaces.msg import UwbPosition

from uwb_position_publisher.trilateration import trilaterate_2d


def is_finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def build_position_message(
    distances_msg: UwbDistances,
    anchor_1: Tuple[float, float],
    anchor_2: Tuple[float, float],
    anchor_3: Tuple[float, float],
    height_differences: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    use_least_squares: bool = False,
) -> UwbPosition:
    msg = UwbPosition()
    msg.header = distances_msg.header
    msg.device_time_ms = distances_msg.device_time_ms

    anchors_valid = (
        distances_msg.anchor_1_valid
        and distances_msg.anchor_2_valid
        and distances_msg.anchor_3_valid
    )
    if not anchors_valid:
        msg.x_m = math.nan
        msg.y_m = math.nan
        msg.valid = False
        return msg

    position = trilaterate_2d(
        anchor_1,
        anchor_2,
        anchor_3,
        float(distances_msg.anchor_1_distance_m),
        float(distances_msg.anchor_2_distance_m),
        float(distances_msg.anchor_3_distance_m),
        height_differences,
        use_least_squares=use_least_squares,
    )
    if position is None:
        msg.x_m = math.nan
        msg.y_m = math.nan
        msg.valid = False
        return msg

    msg.x_m = position[0]
    msg.y_m = position[1]
    msg.valid = True
    return msg


class PositionFilter:
    def __init__(
        self,
        moving_average_window: int,
    ):
        self.moving_average_window = moving_average_window
        self._samples = deque(maxlen=moving_average_window)

    def filter(self, msg: UwbPosition) -> UwbPosition:
        filtered_msg = UwbPosition()
        filtered_msg.header = msg.header
        filtered_msg.device_time_ms = msg.device_time_ms

        if not msg.valid or not is_finite(float(msg.x_m), float(msg.y_m)):
            filtered_msg.x_m = math.nan
            filtered_msg.y_m = math.nan
            filtered_msg.valid = False
            return filtered_msg

        sample = (float(msg.x_m), float(msg.y_m))
        self._samples.append(sample)
        filtered_sample = self._moving_average()

        filtered_msg.x_m = filtered_sample[0]
        filtered_msg.y_m = filtered_sample[1]
        filtered_msg.valid = True
        return filtered_msg

    def _moving_average(self) -> Tuple[float, float]:
        count = len(self._samples)
        x = sum(sample[0] for sample in self._samples) / count
        y = sum(sample[1] for sample in self._samples) / count
        return x, y


class UwbPositionPublisher(Node):
    def __init__(self):
        super().__init__('uwb_position_publisher')

        self.declare_parameter('uwb_distances_topic', '/uwb/distances')
        self.declare_parameter('uwb_position_topic', '/uwb/position')
        self.declare_parameter('anchor_1_x', 0.0)
        self.declare_parameter('anchor_1_y', 0.0)
        self.declare_parameter('anchor_2_x', 1.0)
        self.declare_parameter('anchor_2_y', 0.0)
        self.declare_parameter('anchor_3_x', 0.0)
        self.declare_parameter('anchor_3_y', 1.0)
        for i in range(1, 4):
            self.declare_parameter(f'anchor_{i}_height_difference_m', 0.0)
        self.declare_parameter('use_least_squares', False)
        self.declare_parameter('moving_average_window', 5)

        self.uwb_distances_topic = str(
            self.get_parameter('uwb_distances_topic').value
        )
        self.uwb_position_topic = str(
            self.get_parameter('uwb_position_topic').value
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
        self.use_least_squares = bool(
            self.get_parameter('use_least_squares').value
        )
        self.moving_average_window = int(
            self.get_parameter('moving_average_window').value
        )
        self._validate_parameters()
        self.position_filter = PositionFilter(
            self.moving_average_window,
        )

        self.publisher_ = self.create_publisher(
            UwbPosition,
            self.uwb_position_topic,
            10,
        )
        self.create_subscription(
            UwbDistances,
            self.uwb_distances_topic,
            self.uwb_distances_callback,
            10,
        )

        self.get_logger().info(
            'subscribing %s, publishing %s, '
            'anchor_1=(%.3f, %.3f) anchor_2=(%.3f, %.3f) '
            'anchor_3=(%.3f, %.3f) '
            'moving_average_window=%d '
            'use_least_squares=%s'
            % (
                self.uwb_distances_topic,
                self.uwb_position_topic,
                self.anchor_1[0],
                self.anchor_1[1],
                self.anchor_2[0],
                self.anchor_2[1],
                self.anchor_3[0],
                self.anchor_3[1],
                self.moving_average_window,
                self.use_least_squares,
            )
        )

    def _validate_parameters(self):
        if not all(math.isfinite(h) for h in self.height_differences):
            raise ValueError('anchor height differences must be finite')
        if self.uwb_distances_topic == '':
            raise ValueError('uwb_distances_topic must not be empty')
        if self.uwb_position_topic == '':
            raise ValueError('uwb_position_topic must not be empty')
        if not is_finite(
            self.anchor_1[0],
            self.anchor_1[1],
            self.anchor_2[0],
            self.anchor_2[1],
            self.anchor_3[0],
            self.anchor_3[1],
        ):
            raise ValueError(
                'anchor coordinates must be finite'
            )
        if self.moving_average_window <= 0:
            raise ValueError('moving_average_window must be greater than 0')

    def uwb_distances_callback(self, msg: UwbDistances):
        position_msg = build_position_message(
            msg,
            self.anchor_1,
            self.anchor_2,
            self.anchor_3,
            self.height_differences,
            use_least_squares=self.use_least_squares,
        )
        position_msg = self.position_filter.filter(position_msg)
        self.publisher_.publish(position_msg)


def main(args: Optional[Sequence[str]] = None):
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    rclpy.init(args=args)
    node = None
    try:
        node = UwbPositionPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
