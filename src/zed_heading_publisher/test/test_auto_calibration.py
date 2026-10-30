from unittest.mock import Mock

import pytest

from zed_heading_publisher.auto_calibrate_zed_heading import AutoZedHeadingCalibrator


def controller():
    node = object.__new__(AutoZedHeadingCalibrator)
    node.command_publisher = Mock()
    node.sensor_timeout = 1.0
    node.startup_timeout = 60.0
    node.max_duration_sec = 90.0
    node.wait_started = 0.0
    node.turn_started = None
    node.last_mag = None
    node.last_odom = None
    node.finished = False
    node.speed = 0.15
    node.abort = Mock()
    return node


def test_waiting_never_commands_rotation(monkeypatch):
    monkeypatch.setattr('time.monotonic', lambda: 10.0)
    node = controller()
    node.control_callback()
    assert node.command_publisher.publish.call_args.args[0].angular.z == 0.0
    node.abort.assert_not_called()


@pytest.mark.parametrize('mag,odom', [(None, None), (9.9, None), (None, 9.9)])
def test_startup_timeout(monkeypatch, mag, odom):
    monkeypatch.setattr('time.monotonic', lambda: 61.0)
    node = controller()
    node.last_mag, node.last_odom = mag, odom
    node.control_callback()
    node.abort.assert_called_once()


def test_rotates_only_with_fresh_sensors(monkeypatch):
    monkeypatch.setattr('time.monotonic', lambda: 10.0)
    node = controller()
    node.turn_started = 1.0
    node.last_mag = node.last_odom = 9.9
    node.control_callback()
    command = node.command_publisher.publish.call_args.args[0]
    assert command.angular.z == 0.15
    assert command.linear.x == command.linear.y == 0.0
    node.last_odom = 8.0
    node.control_callback()
    node.abort.assert_called_once()


def test_rotation_has_wall_clock_limit(monkeypatch):
    monkeypatch.setattr('time.monotonic', lambda: 100.0)
    node = controller()
    node.turn_started = 1.0
    node.last_mag = node.last_odom = 99.9
    node.control_callback()
    node.abort.assert_called_once()
    node.command_publisher.publish.assert_not_called()


def test_finished_controller_never_restarts_rotation():
    node = controller()
    node.finished = True
    node.control_callback()
    assert node.command_publisher.publish.call_args.args[0].angular.z == 0.0
    node.abort.assert_not_called()


def test_completion_exits_process_without_hardware(tmp_path):
    import os
    import subprocess
    import sys

    script = '''
import time
from zed_heading_publisher import auto_calibrate_zed_heading as module
original_init = module.AutoZedHeadingCalibrator.__init__
def initialize(self):
    original_init(self)
    self.turn_started = time.monotonic()
    self.last_mag = self.last_odom = time.monotonic()
    self._has_reached_target_rotation = lambda: True
    self._finish = self.stop
module.AutoZedHeadingCalibrator.__init__ = initialize
module.main()
print('PROCESS_COMPLETED', flush=True)
'''
    result = subprocess.run(
        [sys.executable, '-c', script],
        env={**os.environ, 'ROS_LOG_DIR': str(tmp_path / 'logs'),
             'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'},
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'PROCESS_COMPLETED' in result.stdout
