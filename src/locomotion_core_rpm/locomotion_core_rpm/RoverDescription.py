"""Legacy wheel ordering with explicit SI units and motor RPM."""
import math
from .RobotDefinition import RobotDefinition
from .control import positive, vector, limit_rpm


class RoverDescription(RobotDefinition):
    def __init__(self, wheel_radius_m=0.0635, half_length_m=0.205,
                 half_width_m=0.170, gear_ratios=(1.0, 1.0, 1.0, 1.0)):
        self.radius = positive(wheel_radius_m, 'wheel_radius_m')
        self.lever = (positive(half_length_m, 'half_length_m') +
                      positive(half_width_m, 'half_width_m'))
        self.gears = vector(gear_ratios)
        for gear in self.gears:
            positive(gear, 'gear_ratios')

    def kinematic_equation(self, velocity_vector):
        vx, vy, wz = vector(velocity_vector, 3)
        turn = self.lever * wz
        return vector([(vx + vy + turn) / self.radius,
                       (vx - vy - turn) / self.radius,
                       (vx - vy + turn) / self.radius,
                       (vx + vy - turn) / self.radius])

    def motor_rpm(self, velocity_vector, limits):
        omega = self.kinematic_equation(velocity_vector)
        rpm = [w * 60.0 / (2.0 * math.pi) * g
               for w, g in zip(omega, self.gears)]
        return limit_rpm(rpm, limits)
