"""ROS node and real pyserial smoke test using pseudo terminals only."""
import os
import pty
import time
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from locomotion_core_rpm.rover_velocity import RoverVelocity
from locomotion_core_rpm.cmd_roboteq import MotorDriver


def drain(fd):
    result = b''
    while True:
        try:
            result += os.read(fd, 4096)
        except BlockingIOError:
            return result


def test_nodes_with_virtual_serial(tmp_path):
    pairs = [pty.openpty(), pty.openpty()]
    for master, _ in pairs:
        os.set_blocking(master, False)
    config = tmp_path / 'test.yaml'
    config.write_text('''/**:
  ros__parameters:
    max_motor_rpm: [100.0, 100.0, 100.0, 100.0]
    front_port: "''' + os.ttyname(pairs[0][1]) + '''"
    back_port: "''' + os.ttyname(pairs[1][1]) + '''"
''')
    nodes = []
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    try:
        rover = RoverVelocity()
        nodes.append(rover)
        motor = MotorDriver()
        nodes.append(motor)
        assert all(drain(master) == b'!MS 1_!MS 2_' for master, _ in pairs)
        msg = Twist()
        msg.linear.x = 0.1
        rover.receive(msg)
        rpm = rover.command.get()
        assert rpm == pytest.approx([15.038262]*4, rel=1e-6)
        motor.receive(Float32MultiArray(data=rpm))
        motor.tick()
        assert all(drain(master) == b'!S 1 -15_!S 2 -15_' for master, _ in pairs)
        motor.driver.stop()
        assert all(drain(master) == b'!MS 1_!MS 2_' for master, _ in pairs)
        # Expire Twist without wall-clock sleeping.
        rover.command.received = time.monotonic() - 1.0
        assert rover.command.get() is None
        motor.receive(Float32MultiArray(data=rpm))
        motor.driver.command.received = time.monotonic() - 1.0
        motor.tick()
        assert not motor.driver.armed
        assert all(drain(master) == b'!MS 1_!MS 2_' for master, _ in pairs)
    finally:
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for pair in pairs:
            for fd in pair:
                os.close(fd)
