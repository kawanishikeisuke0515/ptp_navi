"""Power parsing and owned subprocess monitoring, independent of ROS."""
from dataclasses import dataclass
import math
import re
import selectors
import subprocess
import threading
import time
import os


@dataclass(frozen=True)
class Reading:
    status: str
    power_w: float = math.nan
    average_power_w: float = math.nan
    raw_line: str = ''

    @property
    def valid(self):
        return self.status == 'ok'


def parse(line, rail='VDD_IN'):
    line = line.rstrip('\r\n')
    tokens = line.split()
    positions = [i for i, token in enumerate(tokens) if token == rail]
    if not positions:
        return Reading('rail_missing', raw_line=line)
    if len(positions) != 1 or positions[0] + 1 == len(tokens):
        return Reading('parse_error', raw_line=line)
    number = r'(\d+(?:\.\d+)?)'
    match = re.fullmatch(number + r'(mW)?/' + number + r'(mW)?',
                         tokens[positions[0] + 1])
    if match and bool(match[2]) == bool(match[4]):
        values = [float(match[i]) / 1000 for i in (1, 3)]
        if all(math.isfinite(v) for v in values):
            return Reading('ok', *values, raw_line=line)
    return Reading('parse_error', raw_line=line)


class Monitor:
    """Drain both pipes without blocking ROS; callback runs on the reader thread."""

    def __init__(self, command, rail, timeout, interval, callback):
        self.rail, self.timeout, self.interval = rail, timeout, interval
        self.callback = callback
        self.stop = threading.Event()
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, bufsize=0)
        self.thread = threading.Thread(target=self.run, name='tegrastats-reader', daemon=True)
        self.thread.start()

    def run(self):
        last_line = time.monotonic()
        last_notice = float('-inf')
        previous = None
        pending = b''
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ, 'stdout')
            selector.register(self.process.stderr, selectors.EVENT_READ, 'stderr')
            while not self.stop.is_set():
                for key, _ in selector.select(timeout=min(.05, self.interval)):
                    data = os.read(key.fd, 65536)
                    if not data:
                        selector.unregister(key.fileobj)
                    if key.data != 'stdout':
                        continue  # stderr is drained, never parsed as telemetry.
                    pending += data
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        last_line = time.monotonic()
                        previous = None
                        self.callback(parse(line.decode('utf-8', errors='replace'), self.rail))
                    if len(pending) > 65536:
                        pending = b''
                        self.callback(Reading('parse_error'))
                now = time.monotonic()
                status = ('process_exited' if self.process.poll() is not None else
                          'timeout' if now - last_line > self.timeout else None)
                if status and (status != previous or now - last_notice >= self.interval):
                    self.callback(Reading(status))
                    previous, last_notice = status, now
                if not selector.get_map():
                    self.stop.wait(min(.05, self.interval))

    def close(self):
        self.stop.set()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.thread.join()
        self.process.stdout.close()
        self.process.stderr.close()
