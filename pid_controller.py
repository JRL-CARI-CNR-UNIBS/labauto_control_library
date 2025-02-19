from base_controller import BaseController
import numpy as np


class PIDController(BaseController):
    """
    PIDController - Proportional-Integral-Derivative (PID) Controller implementation.

    This class implements a PID controller, a widely used feedback controller in control systems.
    It consists of Proportional (P), Integral (I), and Derivative (D) components for computing
    the control action.
    """

    def __init__(self, Tc: float, Kp: float, Ki: float, Kd: float,
                 filters_on_derivative_error=None, filters_on_error_signal=None, filters_on_measure=None):
        """
        Constructor to create a PIDController object.

        :param Tc: Sampling time (must be a positive scalar).
        :param Kp: Proportional gain (must be non-negative).
        :param Ki: Integral gain (must be non-negative).
        :param Kd: Derivative gain (must be non-negative).
        :param filters_on_derivative_error: Filter applied to derivative of the error (optional).
        :param filters_on_error_signal: Filter applied to error signal (optional).
        :param filters_on_measure: Filter applied to measurement (optional).
        """
        super().__init__(Tc)

        if Kp < 0 or Ki < 0 or Kd < 0:
            raise ValueError("Gains Kp, Ki, and Kd must be non-negative")

        self._integral_value = 0
        self._proportional_gain = Kp
        self._integral_gain = Ki
        self._derivative_gain = Kd
        self._filtered_error_for_derivative = 0
        self._filters_on_error_signal = filters_on_error_signal
        self._filters_on_measure = filters_on_measure
        self._filters_on_derivative_error = filters_on_derivative_error

    def initialize(self):
        """Initialize the state variables."""
        self._integral_value = 0
        self._filtered_error_for_derivative = 0

    def starting(self, reference: float, y: float, u: float, uff: float):
        """
        Set the starting conditions based on the input.

        :param reference: Setpoint.
        :param y: Output measurement.
        :param u: Control action.
        :param uff: Feedforward action.
        """
        error_signal = reference - y
        self._filtered_error_for_derivative = error_signal
        self._integral_value = u - self._proportional_gain * error_signal - uff

    def compute_control_action(self, reference: float, y: float, uff: float) -> float:
        """
        Compute the control action.

        :param reference: Setpoint.
        :param y: Output measurement.
        :param uff: Feedforward action.
        :return: Control action value.
        """
        error_signal = reference - y

        if self._filters_on_error_signal:
            error_signal = self._filters_on_error_signal.step(error_signal)

        if self._derivative_gain != 0:
            last_filtered_error = self._filtered_error_for_derivative
            self._filtered_error_for_derivative = self._filters_on_derivative_error.step(
                error_signal) if self._filters_on_derivative_error else error_signal
            derivative_term = self._derivative_gain * (
                        self._filtered_error_for_derivative - last_filtered_error) / self._Tc
        else:
            derivative_term = 0

        print(f"i={self._integral_value} u={self._proportional_gain*error_signal} d={derivative_term} ff={uff}")
        control_action = self._integral_value + self._proportional_gain * error_signal + derivative_term + uff

        if self._umax is not None:
            control_action = np.clip(control_action, -self._umax, self._umax)

        self._integral_value += self._integral_gain * self._Tc * error_signal

        return control_action
