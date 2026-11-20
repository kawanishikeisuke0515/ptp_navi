import json
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from path_control.node import PathControl


def test_topics_and_readonly_parameters():
    rclpy.init(args=['--ros-args', '-p', 'goal_x:=2.0', '-p', 'goal_y:=0.0',
                     '-p', 'init_sample_count:=2', '-p', 'pose_topic:=/test_path/pose',
                     '-p', 'command_topic:=/test_path/cmd', '-p', 'status_topic:=/test_path/status',
                     '-p', 'obstacle_topic:=/test_path/obstacle'])
    controller = probe = None
    try:
        controller = PathControl()
        probe = Node('path_control_test')
        pub = probe.create_publisher(PoseStamped, '/test_path/pose', 10)
        obstacle_pub = probe.create_publisher(Bool, '/test_path/obstacle', 1)
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
        spin_until(lambda: obstacle_pub.get_subscription_count() > 0)
        commands.clear()
        obstacle_pub.publish(Bool(data=True))
        spin_until(lambda: controller.control.state == 'PAUSE')
        spin_until(lambda: commands and commands[-1].linear.x == 0.)
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


@pytest.mark.parametrize('paused', [False, True])
def test_goal_pose_callback_publishes_zero_immediately(paused):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from path_control.core import Config, Controller

    control = Controller(Config(2., 0., init_sample_count=1), 0.)
    control.receive(0., 0., 0., 0., 0.)
    if paused:
        control.receive_obstacle(True, .1)
    publisher = Mock()
    adapter = SimpleNamespace(control=control, world_frame='world',
                              command_pub=publisher, now=lambda: .2)
    msg = PoseStamped()
    msg.header.frame_id = 'world'
    msg.header.stamp.nanosec = 200000000
    msg.pose.position.x = 2.
    msg.pose.orientation.w = 1.
    PathControl.receive(adapter, msg)
    assert control.state == 'COMPLETE'
    publisher.publish.assert_called_once()
    command = publisher.publish.call_args.args[0]
    assert (command.linear.x, command.linear.y, command.angular.z) == (0., 0., 0.)
