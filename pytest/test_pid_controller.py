import pytest
import numpy as np
from pid_controller import PIDController

@pytest.mark.parametrize("Tc, Kp, Ki, Kd", [
    (0.01, 1.0, 0.1, 0.05),
    (0.01, 2.0, 0.2, 0.1),
    (0.01, 0.5, 0.05, 0.02),
    (0.01, 0.5, 0.00, 0.02),
    (0.01, 0.0, 0.05, 0.02),
    (0.01, 0.5, 0.05, 0.0),
    (0.01, 0.5, 0.0, 0.0)
])
def test_pid_initialization(Tc, Kp, Ki, Kd):
    """Test initialization of the PID controller."""
    pid = PIDController(Tc, Kp, Ki, Kd)
    assert pid._proportional_gain == Kp, "Incorrect proportional gain."
    assert pid._integral_gain == Ki, "Incorrect integral gain."
    assert pid._derivative_gain == Kd, "Incorrect derivative gain."
    assert pid._integral_value == 0, "Integral value should be initialized to zero."
    assert pid._filtered_error_for_derivative == 0, "Derivative error should initialize to zero."

@pytest.mark.parametrize("Tc, Kp, Ki, Kd", [
    (0.01, 1.0, 0.1, 0.05),
    (0.01, 2.0, 0.2, 0.1),
    (0.01, 0.5, 0.05, 0.02),
    (0.01, 0.5, 0.00, 0.02),
    (0.01, 0.0, 0.05, 0.02),
    (0.01, 0.5, 0.05, 0.0),
    (0.01, 0.5, 0.0, 0.0)
])
def test_pid_starting(Tc, Kp, Ki, Kd):
    """Test starting conditions of the PID controller."""
    pid = PIDController(Tc, Kp, Ki, Kd)
    pid.starting(reference=2.0, y=1.0, u=0.5, uff=0.0)
    assert pid._integral_value == 0.5 - pid._proportional_gain, "Incorrect integral value after starting."
    assert pid._filtered_error_for_derivative == 1.0, "Filtered error should match initial error."

@pytest.mark.parametrize("Tc, Kp, Ki, Kd", [
    (0.01, 1.0, 0.1, 0.05),
    (0.01, 2.0, 0.2, 0.1),
    (0.01, 0.5, 0.05, 0.02),
    (0.01, 0.5, 0.00, 0.02),
    (0.01, 0.0, 0.05, 0.02),
    (0.01, 0.5, 0.05, 0.0),
    (0.01, 0.5, 0.0, 0.0)
])
def test_pid_control_action(Tc, Kp, Ki, Kd):
    """Test computation of control action."""
    y=0
    r=1
    de=(r-y)/Tc
    pid = PIDController(Tc, Kp, Ki, Kd)
    pid.starting(reference=0.0, y=0.0, u=0.0, uff=0.0)
    control_action = pid.compute_control_action(reference=1.0, y=0.0, uff=0.0)
    expected_output = pid._proportional_gain*(r-y)+de*pid._derivative_gain
    assert np.isclose(control_action, expected_output, atol=1e-5), "Control action mismatch."


    control_action = pid.compute_control_action(reference=1.0, y=0.0, uff=0.0)
    expected_output = pid._proportional_gain*(r-y)+pid._integral_gain*(r-y)*Tc
    assert np.isclose(control_action, expected_output, atol=1e-5), "Control action mismatch."

    control_action = pid.compute_control_action(reference=1.0, y=0.0, uff=0.0)
    expected_output = pid._proportional_gain*(r-y)+pid._integral_gain*(r-y)*2.0*Tc
    assert np.isclose(control_action, expected_output, atol=1e-5), "Control action mismatch."

@pytest.mark.parametrize("Tc, Kp, Ki, Kd", [
    (0.01, 1.0, 0.1, 0.05),
    (0.01, 2.0, 0.2, 0.1),
    (0.01, 0.5, 0.05, 0.02),
    (0.01, 0.5, 0.00, 0.02),
    (0.01, 0.0, 0.05, 0.02),
    (0.01, 0.5, 0.05, 0.0),
    (0.01, 0.5, 0.0, 0.0)
])
def test_pid_saturation(Tc, Kp, Ki, Kd):
    """Test control action saturation."""
    pid =PIDController(Tc, Kp, Ki, Kd)
    pid._umax = 1.0  # Set maximum control action
    control_action = pid.compute_control_action(reference=10.0, y=0.0, uff=0.0)
    assert control_action <= 1.0, "Control action exceeded saturation limit."


def test_invalid_gains():
    """Test that invalid gains raise an error."""
    with pytest.raises(ValueError):
        PIDController(0.01, -1.0, 0.1, 0.05)  # Negative Kp
    with pytest.raises(ValueError):
        PIDController(0.01, 1.0, -0.1, 0.05)  # Negative Ki
    with pytest.raises(ValueError):
        PIDController(0.01, 1.0, 0.1, -0.05)  # Negative Kd

if __name__ == "__main__":
    pytest.main()
