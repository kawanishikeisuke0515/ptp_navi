import math
import pytest
from locomotion_core_rpm.RoverDescription import RoverDescription
from locomotion_core_rpm.control import RoboteqDriver, FreshCommand, limit_rpm


class Port:
    def __init__(self, **kwargs):
        self.writes = []
        self.closed = False
        self.fail = False

    def write(self, value):
        if self.fail:
            raise OSError('disconnected')
        self.writes.append(value)
        return len(value)

    def close(self):
        self.closed = True


def driver():
    now = [0.0]
    d = RoboteqDriver('front', 'back', 115200, [1000]*4, [-1]*4,
                      serial_factory=Port, clock=lambda: now[0])
    return d, now


def test_kinematics():
    r = RoverDescription()
    assert r.motor_rpm([0.1, 0, 0], [1000]*4) == pytest.approx([15.038262]*4, rel=1e-6)
    v = 60 / (2 * math.pi) / 0.0635
    assert r.motor_rpm([0, 1, 0], [1000]*4) == pytest.approx([v, -v, -v, v])
    t = v * 0.375
    assert r.motor_rpm([0, 0, 1], [1000]*4) == pytest.approx([t, -t, t, -t])
    assert r.motor_rpm([0, 0, 0], [1000]*4) == [0]*4
    geared = RoverDescription(gear_ratios=[1, 2, 3, 4])
    assert geared.motor_rpm([0.1, 0, 0], [1000]*4) == pytest.approx([0.1*v*g for g in [1,2,3,4]])


def test_proportional_limits():
    assert limit_rpm([100, -200, 300, -400], [100, 100, 100, 100]) == [25, -50, 75, -100]


@pytest.mark.parametrize('values', [[1,2,3], [1,2,3,4,5], [math.nan]*4, [math.inf]*4])
def test_invalid_input_stops(values):
    d, _ = driver()
    d.receive([10]*4)
    d.tick()
    with pytest.raises(ValueError):
        d.receive(values)
    assert not d.armed
    assert all(p.writes[-2:] == [b'!MS 1_', b'!MS 2_'] for p in d.ports)


def test_fresh_input_starts_without_permission_and_stop_discards_command():
    d, _ = driver()
    d.tick()
    assert all(p.writes == [b'!MS 1_', b'!MS 2_'] for p in d.ports)
    d.receive([10.9, -20.9, 30.9, -40.9])
    d.tick()
    assert d.ports[0].writes[-2:] == [b'!S 1 -10_', b'!S 2 20_']
    assert d.ports[1].writes[-2:] == [b'!S 1 -30_', b'!S 2 40_']
    d.stop()
    d.tick()
    assert not d.armed
    assert all(p.writes[-2:] == [b'!MS 1_', b'!MS 2_'] for p in d.ports)


@pytest.mark.parametrize('send_first', [False, True])
def test_timeout_even_before_first_serial_tick(send_first):
    d, now = driver()
    d.receive([10]*4)
    if send_first:
        d.tick()
    now[0] = 0.5
    with pytest.raises(TimeoutError):
        d.tick()
    assert not d.armed
    d.receive([20]*4)
    d.tick()
    assert d.ports[0].writes[-1] == b'!S 2 -20_'


def test_serial_resend_does_not_refresh_timeout():
    d, now = driver()
    d.receive([10]*4)
    for t in [0.1, 0.2, 0.3, 0.4]:
        now[0] = t
        d.tick()
    now[0] = 0.51
    with pytest.raises(TimeoutError):
        d.tick()


def test_partial_connection_cleanup():
    ports = []
    def factory(**kwargs):
        if ports:
            raise OSError('second port failed')
        ports.append(Port())
        return ports[0]
    with pytest.raises(OSError):
        RoboteqDriver('a', 'b', 115200, [100]*4, [-1]*4, serial_factory=factory)
    assert ports[0].closed
    assert ports[0].writes == [b'!MS 1_', b'!MS 2_']


def test_disconnect_stops_other_controller_and_latches_fault():
    d, _ = driver()
    d.receive([10]*4)
    d.ports[0].fail = True
    with pytest.raises(OSError):
        d.tick()
    assert d.faulted and not d.armed
    assert d.ports[1].writes[-2:] == [b'!MS 1_', b'!MS 2_']
    d.receive([20]*4)
    d.tick()
    assert not d.armed


def test_close_stops_and_closes():
    d, _ = driver()
    ports = d.ports[:]
    assert d.close() == []
    assert all(p.closed and p.writes[-1] == b'!MS 2_' for p in ports)


@pytest.mark.parametrize('limits', [[0]*4, [-1]*4, [65536]*4, [math.inf]*4])
def test_bad_config_never_opens_ports(limits):
    def factory(**kwargs):
        pytest.fail('Must validate before accessing hardware')
    with pytest.raises(ValueError):
        RoboteqDriver('a', 'b', 115200, limits, [-1]*4, serial_factory=factory)


def test_fresh_command_discards_expired_values():
    now = [0.0]
    command = FreshCommand(0.5, lambda: now[0])
    command.set([1]*4)
    now[0] = 0.5
    assert command.get() is None
    assert command.values is None
