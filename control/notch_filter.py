import numpy as np
from scipy.signal import bilinear
from control import BaseFilter


class NotchFilter(BaseFilter):
    """
    NotchFilter - A discrete-time notch filter implementation.

    This filter is used to attenuate a specific frequency while allowing others to pass.
    """

    def __init__(self, Tc: float, wn: float, xi_z: float, xi_p: float):
        """
        Constructor to create a NotchFilter object.

        :param Tc: Sampling time (must be a positive scalar).
        :param wn: Natural frequency (rad/s) of the notch filter.
        :param xi_z: Damping ratio of the zeros.
        :param xi_p: Damping ratio of the poles.
        """
        super().__init__(Tc)

        if wn <= 0 or xi_z < 0 or xi_p < 0:
            raise ValueError("wn must be positive, xi_z and xi_p must be non-negative.")

        omega_n = wn  # Natural frequency in rad/s
        Ts = Tc  # Sampling time

        # Continuous-time transfer function coefficients
        num = [1, 2 * xi_z * omega_n, omega_n ** 2]
        den = [1, 2 * xi_p * omega_n, omega_n ** 2]

        # Discretization using bilinear transformation (Tustin)
        self._b, self._a = bilinear(num, den, fs=1 / Ts)

        self._x = np.zeros(len(self._b) - 1)  # Input delay buffer
        self._y = np.zeros(len(self._a) - 1)  # Output delay buffer

    def initialize(self):
        """Initialize the filter state."""
        self._x.fill(0)
        self._y.fill(0)

    def starting(self, input_value: float):
        """
        Set the starting conditions based on the input.

        :param input_value: Initial input value (must be a scalar).
        """
        if not isinstance(input_value, (int, float)):
            raise ValueError("Input must be a scalar")

        self._y.fill(input_value)
        self._x.fill(input_value)

    def step(self, input_value: float) -> float:
        """
        Apply the notch filter to the input value.

        :param input_value: Current input sample.
        :return: Filtered output sample.
        """
        if not isinstance(input_value, (int, float)):
            raise ValueError("Input must be a scalar")

        output = (self._b[0] * input_value + np.dot(self._b[1:], self._x)
                  - np.dot(self._a[1:], self._y))

        # Update state buffers
        self._x[1:] = self._x[:-1]
        self._x[0] = input_value
        self._y[1:] = self._y[:-1]
        self._y[0] = output

        return output
