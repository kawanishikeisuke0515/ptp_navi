"""Exercise real ROS messages and node using a fake tegrastats executable."""
import time
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('jetson_interfaces.msg')
from jetson_interfaces.msg import JetsonPower
from jetson_power_publisher.jetson_power_publisher import JetsonPowerPublisher


def test_node_delivery_and_shutdown(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    executable = tmp_path / 'tegrastats'
    executable.write_text('#!/usr/bin/python3\nimport time\nwhile True:\n'
                          ' print("VDD_IN 5000mW/4500mW", flush=True)\n time.sleep(.1)\n')
    executable.chmod(0o755)
    rclpy.init(args=['--ros-args', '-p', 'tegrastats_path:=' + str(executable)])
    node = probe = None
    try:
        probe = rclpy.create_node('power_test_probe')
        received = []
        probe.create_subscription(JetsonPower, '/jetson/power', received.append, 10)
        node = JetsonPowerPublisher()
        deadline = time.monotonic() + 5
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.1)
        assert received
        assert received[-1].valid and received[-1].power_w == 5.
        assert received[-1].average_power_w == 4.5
        assert received[-1].header.stamp.sec > 0
        process = node.monitor.process
        node.destroy_node()
        node = None
        assert process.poll() is not None
    finally:
        if node is not None:
            node.destroy_node()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
