"""Exercise real ROS message serialization and topic delivery when ROS is available."""
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('uwb_interfaces.msg')
pytest.importorskip('zed_interfaces.msg')
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from uwb_interfaces.msg import UwbPosition
from zed_interfaces.msg import ZedHeading
from uwb_robot_pose_publisher.uwb_robot_pose_publisher import UwbRobotPosePublisher


def test_topic_pose_and_invalid_input():
    rclpy.init()
    node = UwbRobotPosePublisher()
    probe = Node('robot_pose_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    received = []
    probe.create_subscription(PoseStamped, '/uwb/robot_pose', received.append, 10)
    heading_pub = probe.create_publisher(ZedHeading, '/zed/heading', 10)
    position_pub = probe.create_publisher(UwbPosition, '/uwb/position', 10)

    def spin_until(condition, timeout=3):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        return condition()

    try:
        assert spin_until(lambda: heading_pub.get_subscription_count() > 0
                          and position_pub.get_subscription_count() > 0
                          and node.publisher.get_subscription_count() > 0)
        stamp = probe.get_clock().now().to_msg()
        heading = ZedHeading()
        heading.header.stamp = stamp
        heading.valid = True
        heading.robot_yaw_rad = 0.0
        position = UwbPosition()
        position.header.stamp = stamp
        position.valid = True
        position.x_m, position.y_m = 1.0, 2.0
        heading_pub.publish(heading)
        position_pub.publish(position)
        assert spin_until(lambda: len(received) == 1)
        msg = received[0]
        assert msg.header.stamp == stamp
        assert msg.header.frame_id == 'world'
        assert msg.pose.position.x == pytest.approx(.990)
        assert msg.pose.position.y == pytest.approx(1.665)
        assert msg.pose.position.z == 0
        assert msg.pose.orientation.z == pytest.approx(math.sqrt(.5))
        assert msg.pose.orientation.w == pytest.approx(math.sqrt(.5))
        position.valid = False
        position.header.stamp = probe.get_clock().now().to_msg()
        position_pub.publish(position)
        assert not spin_until(lambda: len(received) > 1, .15)
    finally:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
