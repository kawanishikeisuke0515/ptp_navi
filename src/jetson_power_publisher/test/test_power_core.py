import math
import sys
import threading
import time

import pytest
from jetson_power_publisher.core import Monitor, parse


@pytest.mark.parametrize('value', ['5000/4500', '5000mW/4500mW', '5000.0/4500.0'])
def test_parse(value):
    line = 'VDD_SOC 200/100 VDD_IN ' + value + ' CPU@40C\n'
    result = parse(line)
    assert result.valid
    assert (result.power_w, result.average_power_w) == (5., 4.5)
    assert result.raw_line == line.rstrip('\n')
    assert parse('VDD_IN 0/0').valid


@pytest.mark.parametrize('line,status', [
    ('VDD_SOC 1/2', 'rail_missing'), ('X_VDD_IN 1/2', 'rail_missing'),
    ('VDD_IN', 'parse_error'), ('VDD_IN 1/2 VDD_IN 3/4', 'parse_error'),
    *[('VDD_IN ' + value, 'parse_error') for value in
      ['-1/2', 'nan/2', 'inf/2', '1W/2W', '1mW/2', '1/', '1/2/3']],
])
def test_invalid(line, status):
    result = parse(line)
    assert not result.valid and result.status == status
    assert math.isnan(result.power_w) and math.isnan(result.average_power_w)


def test_process_timeout_recovery_exit_and_stderr():
    results = []
    exited = threading.Event()
    def receive(reading):
        results.append(reading)
        if reading.status == 'process_exited':
            exited.set()
    code = ("import sys,time; sys.stderr.write('x'*200000); sys.stderr.flush(); "
            "time.sleep(.2); print('VDD_IN 5000/4000',flush=True); time.sleep(.1)")
    monitor = Monitor([sys.executable, '-c', code], 'VDD_IN', .08, .02, receive)
    try:
        assert exited.wait(4)
    finally:
        monitor.close()
    statuses = [r.status for r in results]
    assert 'timeout' in statuses and 'ok' in statuses
    assert statuses.index('timeout') < statuses.index('ok') < statuses.index('process_exited')
    assert not monitor.thread.is_alive()
    assert monitor.process.poll() is not None


def test_shutdown_owned_process():
    monitor = Monitor([sys.executable, '-c', 'import time; time.sleep(60)'],
                      'VDD_IN', 3., 1., lambda r: None)
    monitor.close()
    assert monitor.process.poll() is not None
    assert not monitor.thread.is_alive()


def test_missing_executable():
    with pytest.raises(FileNotFoundError):
        Monitor(['/nonexistent/tegrastats'], 'VDD_IN', 3., 1., lambda r: None)
