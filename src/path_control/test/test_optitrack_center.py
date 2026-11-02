import math

import pytest
from path_control.optitrack_geometry import center_position


def test_offset_rotates_with_body_in_three_dimensions():
    assert center_position((1, 2, 3), (0, 0, 0, 1), (.3, -.1, .2)) == pytest.approx((.7, 2.1, 2.8))
    s = math.sqrt(.5)
    assert center_position((1, 2, 3), (0, 0, s, s), (.3, 0, 0)) == pytest.approx((1, 1.7, 3))
    assert center_position((1, 2, 3), (0, s, 0, s), (.3, 0, 0)) == pytest.approx((1, 2, 3.3))


@pytest.mark.parametrize('quaternion', [(0, 0, 0, 0), (0, 0, 0, 2), (0, 0, math.nan, 1)])
def test_invalid_orientation(quaternion):
    with pytest.raises(ValueError):
        center_position((1, 2, 3), quaternion, (0, 0, 0))


def test_ros_callback_preserves_stamp_frame_orientation_and_rejects_invalid():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseStamped
    from path_control.optitrack_center import OptitrackCenter
    from unittest.mock import Mock

    rclpy.init()
    node = OptitrackCenter()
    publisher = node.publisher
    try:
        node.publisher = Mock()
        node.offset = (.3, 0, 0)
        msg = PoseStamped()
        msg.header.frame_id = 'mocap'
        msg.header.stamp.sec = 123
        msg.pose.position.x = 1.
        msg.pose.orientation.w = 1.
        node.receive(msg)
        result = node.publisher.publish.call_args.args[0]
        assert result.header == msg.header
        assert result.pose.orientation == msg.pose.orientation
        assert result.pose.position.x == pytest.approx(.7)
        assert msg.pose.position.x == 1.
        msg.pose.orientation.w = 0.
        node.receive(msg)
        assert node.publisher.publish.call_count == 1
    finally:
        node.publisher = publisher
        node.destroy_node()
        rclpy.shutdown()
