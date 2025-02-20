import numpy as np
import pytest
from labauto import FIRFilter
from scipy.signal import lfilter, dlti


def create_fir_system(fir_coef, Tc):
    """
    Creates a discrete-time system representation for an FIR filter.
    """
    num = fir_coef
    den = [1]
    return dlti(num, den, dt=Tc)


@pytest.mark.parametrize("Tc, fir_coef", [
    (0.01, [0.2, 0.5, 0.3]),
    (0.01, [1, -1]),
    (0.001, [0.1, 0.6, 0.3]),
    (0.001, [0.5, -0.5])
])
def test_fir_filter(Tc, fir_coef):
    """
    Test FIRFilter by comparing its response to scipy's lfilter implementation with a random input signal.
    """
    fir_filter = FIRFilter(Tc, fir_coef)
    fir_filter.initialize()

    # Generate random input signal
    np.random.seed(42)  # For reproducibility
    input_signal = np.random.randn(100)

    # Compute theoretical response using scipy lfilter
    y_theoretical = lfilter(fir_coef, [1], input_signal)

    # Compute response using the implemented FIR filter
    fir_filter.starting(0)
    y_filter = np.array([fir_filter.step(x) for x in input_signal])

    # Check that the outputs match
    assert np.allclose(y_filter, y_theoretical, atol=1e-10), "FIR filter output mismatch"


if __name__ == "__main__":
    pytest.main()