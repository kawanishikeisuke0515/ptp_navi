"""Common interface for body-to-wheel kinematics."""
from abc import ABC, abstractmethod


class RobotDefinition(ABC):
    @abstractmethod
    def kinematic_equation(self, velocity_vector):
        """Return wheel angular velocities in rad/s."""
