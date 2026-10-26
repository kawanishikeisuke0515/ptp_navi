import json
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from path_control.node import PathControl


def test_topics_and_readonly_parameters():
    rclpy.init(args=['--ros-args', '-p', 'goal_x:=2.0', '-p', 'goal_y:=0.0',
                     '-p', 'init_sample_count:=2', '-p', 'pose_topic:=/test_path/pose',
                     '-p', 'command_topic:=/test_path/cmd', '-p', 'status_topic:=/test_path/status'])
    controller = probe = None
    try:
        controller = PathControl()
        probe = Node('path_control_test')
        pub = probe.create_publisher(PoseStamped, '/test_path/pose', 10)
        commands, statuses = [], []
        probe.create_subscription(Twist, '/test_path/cmd', commands.append, 10)
        probe.create_subscription(String, '/test_path/status',
                                  lambda m: statuses.append(json.loads(m.data)), 10)

        def spin_until(condition):
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline:
                rclpy.spin_once(controller, timeout_sec=.01)
                rclpy.spin_once(probe, timeout_sec=.01)
                if condition():
                    return
            pytest.fail('ROS topic condition timed out')

        spin_until(lambda: pub.get_subscription_count() > 0 and statuses)
        assert all(m.linear.x == 0 for m in commands)
        for count in (1, 2):
            msg = PoseStamped()
            msg.header.frame_id = 'world'
            msg.header.stamp = probe.get_clock().now().to_msg()
            msg.pose.orientation.w = 1.
            pub.publish(msg)
            spin_until(lambda: len(controller.control.samples) == count)
        spin_until(lambda: any(m.linear.x > 0 for m in commands))
        pub.publish(PoseStamped())
        spin_until(lambda: any(m['state'] == 'ERROR' for m in statuses))
        spin_until(lambda: commands[-1].linear.x == 0.)
        assert controller.control.reason == 'frame_mismatch'
        assert not controller.set_parameters([Parameter('goal_x', value=3.)])[0].successful
    finally:
        if controller is not None:
            controller.destroy_node()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
