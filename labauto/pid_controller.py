from labauto import BaseController
import numpy as np
import math


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
        :param filters_on_derivative_error: Filters applied to derivative of the error (optional).
        :param filters_on_error_signal: Filters applied to error signal (optional).
        :param filters_on_measure: Filters applied to measurement (optional).
        """
        super().__init__(Tc)

        for name, value in [("Kp", Kp), ("Ki", Ki), ("Kd", Kd)]:
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError(f"{name} must be a non-negative finite scalar")

        self._integral_value = 0.0
        self._proportional_gain = float(Kp)
        self._integral_gain = float(Ki)
        self._derivative_gain = float(Kd)
        self._filtered_error_for_derivative = 0.0
        self._filters_on_error_signal = filters_on_error_signal
        self._filters_on_measure = filters_on_measure
        self._filters_on_derivative_error = filters_on_derivative_error

    def initialize(self):
        self._integral_value = 0.0
        self._filtered_error_for_derivative = 0.0

        if self._filters_on_error_signal:
            for f in self._filters_on_error_signal:
                f.initialize()

        if self._filters_on_measure:
            for f in self._filters_on_measure:
                f.initialize()

        if self._filters_on_derivative_error:
            for f in self._filters_on_derivative_error:
                f.initialize()

    def starting(self, reference: float, measure: float, u: float, uff: float):
        """
        Set the starting conditions based on the input.

        :param reference: Setpoint.
        :param y: Output measurement.
        :param u: Control action.
        :param uff: Feedforward action.
        """
        error_signal = reference - measure
        self._filtered_error_for_derivative = error_signal
        self._integral_value = u - self._proportional_gain * error_signal - uff

        if self._filters_on_error_signal:
            for f in self._filters_on_error_signal:
                f.starting(error_signal)

        if self._filters_on_measure:
            for f in self._filters_on_measure:
                f.starting(measure)

        if self._filters_on_derivative_error:
            for f in self._filters_on_derivative_error:
                f.starting(error_signal)

    def compute_control_action(self, reference: float, y: float, uff: float) -> float:
        measure = y
        if self._filters_on_measure:
            for f in self._filters_on_measure:
                measure = f.step(measure)

        error_signal = reference - measure

        if self._filters_on_error_signal:
            for f in self._filters_on_error_signal:
                error_signal = f.step(error_signal)

        if self._derivative_gain != 0:
            last_filtered_error = self._filtered_error_for_derivative
            error_signal_for_compute_derivative = error_signal

            if self._filters_on_derivative_error:
                for f in self._filters_on_derivative_error:
                    error_signal_for_compute_derivative = f.step(error_signal_for_compute_derivative)

            self._filtered_error_for_derivative = error_signal_for_compute_derivative
            derivative_term = self._derivative_gain * (
                self._filtered_error_for_derivative - last_filtered_error
            ) / self._Tc
        else:
            derivative_term = 0.0

        control_action = (
            self._integral_value
            + self._proportional_gain * error_signal
            + derivative_term
            + uff
        )

        control_action_sat = np.clip(control_action, -self._umax, self._umax)

        update_integral = (
            np.isclose(control_action_sat, control_action)
            or (control_action_sat < control_action and error_signal < 0)
            or (control_action_sat > control_action and error_signal > 0)
        )

        if update_integral:
            self._integral_value += self._integral_gain * self._Tc * error_signal

        return float(control_action_sat)
