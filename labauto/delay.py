import numpy as np
from labauto import BaseFilter


class Delay(BaseFilter):
    """
    Digital Delay Filter.

    This class implements a digital delay filter that delays the input signal by a specified amount of time.
    """

    def __init__(self, Tc: float, L: float):
        """
        Initialize the Delay filter.

        :param Tc: Sampling time (must be a positive scalar).
        :param L: Delay time (must be a positive scalar).
        """
        super().__init__(Tc)

        if not isinstance(L, (int, float)) or L <= 0:
            raise ValueError("Delay (L) must be a positive scalar.")

        self._n = np.ceil(L / Tc)  # Compute the number of samples corresponding to the delay time
        self._idx = 0  # Index for circular buffer
        self._buffer = np.zeros(int(self._n))  # Initialize buffer with zeros

    def initialize(self):
        """
        Reset the buffer and index to their initial states.
        """
        self._buffer.fill(0)
        self._idx = 0

    def starting(self, input_value: float):
        """
        Set the initial conditions of the buffer.

        :param input_value: Initial input value (must be a scalar).
        """
        if not isinstance(input_value, (int, float)):
            raise ValueError("Input must be a scalar.")

        self._buffer.fill(input_value)  # Fill buffer with initial value
        self._idx = 0

    def step(self, input_value: float) -> float:
        """
        Process one step of the delay filter.

        :param input_value: Current input value (must be a scalar).
        :return: Delayed output value.
        """
        if not isinstance(input_value, (int, float)):
            raise ValueError("Input must be a scalar.")

        output = self._buffer[self._idx]  # Retrieve delayed value
        self._buffer[self._idx] = input_value  # Store new input value in buffer
        self._idx = (self._idx + 1) % int(self._n)  # Implement circular buffer logic

        return output