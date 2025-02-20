from first_order_low_pass_filter import FirstOrderLowPassFilter
import numpy as np
import pytest
from scipy.signal import TransferFunction, cont2discrete, dlti, dstep


def create_transfer_function(Tf):
    """
    Creates a continuous-time transfer function for the notch filter.
    """
    num = [1]
    den = [Tf, 1]
    return TransferFunction(num, den)


@pytest.mark.parametrize("time_constant, Tc", [
    (0.1,  0.01),
    (1.0,  0.01),
    (0.01,  0.001),
    (1.0,  0.001),
])
def test_filter(time_constant, Tc):
    """
    Test the FirstOrderLowPassFilter class by comparing its step response to the theoretical discrete-time filter.
    """
    # Create the notch filter
    f = FirstOrderLowPassFilter(Tc, time_constant)
    f.initialize()

    # Define the continuous-time transfer function
    F = create_transfer_function(time_constant)

    # Convert to discrete-time using Tustin's method
    Fd_num, Fd_den, Td = cont2discrete((F.num, F.den), Tc, method='zoh')
    Fd = dlti(Fd_num.flatten(), Fd_den.flatten(), dt=Tc)

    # Compute step response of the discrete-time system
    t, y = dstep(Fd)
    t = t.flatten()
    y = y[0].flatten()

    # Compute step response of the Python notch filter
    f.starting(0)
    y_python = np.array([f.step(1) for _ in range(len(t))])
    # Compare results
    assert np.allclose(y, y_python, atol=1e-10), "Output mismatch"


if __name__ == "__main__":
    pytest.main()
