import numpy as np
import pytest
from scipy.signal import TransferFunction, cont2discrete, dlti, dstep
from notch_filter import NotchFilter


def create_transfer_function(wn, xi_z, xi_p):
    """
    Creates a continuous-time transfer function for the notch filter.
    """
    num = [1, 2 * xi_z * wn, wn ** 2]
    den = [1, 2 * xi_p * wn, wn ** 2]
    return TransferFunction(num, den)


@pytest.mark.parametrize("wn, xi_z, xi_p, Tc", [
    (10.0, 0.1, 0.9, 0.01),
    (20.0, 0.05, 0.95, 0.005),
    (100.0, 0.1, 0.9, 0.001),
    (200.0, 0.05, 0.95, 0.001),
])
def test_notch_filter(wn, xi_z, xi_p, Tc):
    """
    Test the NotchFilter class by comparing its step response to the theoretical discrete-time notch filter.
    """
    # Create the notch filter
    nf = NotchFilter(Tc, wn, xi_z, xi_p)
    nf.initialize()

    # Define the continuous-time transfer function
    F = create_transfer_function(wn, xi_z, xi_p)

    # Convert to discrete-time using Tustin's method
    Fd_num, Fd_den, Td = cont2discrete((F.num, F.den), Tc, method='tustin')
    Fd = dlti(Fd_num.flatten(), Fd_den.flatten(), dt=Tc)

    # Compute step response of the discrete-time system
    t, y = dstep(Fd)
    t = t.flatten()
    y = y[0].flatten()

    # Compute step response of the Python notch filter
    nf.starting(0)
    y_python = np.array([nf.step(1) for _ in range(len(t))])
    y_python[0]=1000
    # Compare results
    assert np.allclose(y, y_python, atol=1e-10), "Output mismatch"


if __name__ == "__main__":
    pytest.main()
