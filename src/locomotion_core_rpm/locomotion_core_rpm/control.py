"""ROS-independent validation, input freshness and Roboteq transport."""
import math
import time


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def vector(values, size=4):
    values = [float(v) for v in values]
    if len(values) != size or not all(math.isfinite(v) for v in values):
        raise ValueError(f'Expected {size} finite values')
    return values


def rpm_limits(values):
    values = vector(values)
    for value in values:
        if not 0 < value <= 65535:
            raise ValueError('Set each max_motor_rpm to a value in (0, 65535]')
    return values


def limit_rpm(values, limits):
    values, limits = vector(values), rpm_limits(limits)
    scale = min([1.0] + [m / abs(v) for v, m in zip(values, limits) if v])
    return [v * scale for v in values]


class FreshCommand:
    def __init__(self, timeout, clock=time.monotonic):
        self.timeout = positive(timeout, 'timeout')
        self.clock = clock
        self.clear()

    def clear(self):
        self.values = None
        self.received = None

    def set(self, values):
        self.values = vector(values)
        self.received = self.clock()

    def get(self):
        if self.received is None or self.clock() - self.received >= self.timeout:
            self.clear()
            return None
        return self.values


class RoboteqDriver:
    def __init__(self, front_port, back_port, baudrate, limits, directions,
                 timeout=0.5, serial_factory=None, clock=time.monotonic):
        self.limits = rpm_limits(limits)
        self.directions = vector(directions)
        if any(d not in (-1, 1) for d in self.directions):
            raise ValueError('motor_directions must contain four +1/-1 values')
        if not front_port or not back_port or front_port == back_port:
            raise ValueError('Two distinct serial ports are required')
        if not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError('baudrate must be a positive integer')
        self.command = FreshCommand(timeout, clock)
        self.armed = False
        self.faulted = False
        self.ports = []
        if serial_factory is None:
            from serial import Serial
            serial_factory = Serial
        try:
            for path in (front_port, back_port):
                self.ports.append(serial_factory(
                    port=path, baudrate=baudrate, bytesize=8, parity='N',
                    stopbits=1, timeout=0, write_timeout=0.1, exclusive=True))
            errors = self.stop()
            if errors:
                raise OSError('; '.join(errors))
        except Exception:
            self.close()
            raise

    @staticmethod
    def write(port, payload):
        data = payload.encode('ascii')
        if port.write(data) != len(data):
            raise OSError('Incomplete serial write')

    def stop(self):
        self.armed = False
        self.command.clear()
        errors = []
        for port in self.ports:
            for channel in (1, 2):
                try:
                    self.write(port, f'!MS {channel}_')
                except Exception as exc:
                    errors.append(str(exc))
        if errors:
            self.faulted = True
        return errors

    def receive(self, values):
        try:
            values = limit_rpm(values, self.limits)
        except (ValueError, TypeError, OverflowError):
            self.stop()
            raise
        if not self.faulted:
            self.command.set(values)
            self.armed = True

    def tick(self):
        if not self.armed or self.faulted:
            return
        had_command = self.command.received is not None
        values = self.command.get()
        if values is None:
            # An expired command must be discarded before resuming.
            if had_command:
                errors = self.stop()
                raise TimeoutError('RPM input timeout' + (': ' + '; '.join(errors) if errors else ''))
            return
        try:
            for i, (rpm, direction) in enumerate(zip(values, self.directions)):
                self.write(self.ports[i // 2], f'!S {i % 2 + 1} {int(rpm * direction)}_')
        except Exception:
            self.faulted = True
            self.stop()
            raise

    def close(self):
        errors = self.stop()
        for port in self.ports:
            try:
                port.close()
            except Exception as exc:
                errors.append(str(exc))
        self.ports.clear()
        return errors
