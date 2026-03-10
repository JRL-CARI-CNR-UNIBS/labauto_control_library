import numpy as np
import pytest
import sys
import os

# Get the project root directory and add it to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from labauto import Delay


def test_delay_initialization():
    """Test initialization of the Delay filter."""
    delay = Delay(Tc=0.01, L=0.05)
    assert delay._buffer.shape == (5,), "Incorrect buffer size after initialization."
    assert np.all(delay._buffer == 0), "Buffer should be initialized to zeros."
    assert delay._idx == 0, "Index should be initialized to zero."


def test_delay_starting():
    """Test starting condition of the Delay filter."""
    delay = Delay(Tc=0.01, L=0.05)
    delay.starting(3.5)
    assert np.all(delay._buffer == 3.5), "Buffer should be filled with initial value."
    assert delay._idx == 0, "Index should reset to zero."


def test_delay_step():
    """Test the step function of the Delay filter."""
    delay = Delay(Tc=0.01, L=0.03)  # 3 samples delay
    delay.starting(1.0)

    # Step through with new inputs and check outputs
    input_signal = [2, 3, 4, 5, 6]
    expected_output = [1, 1, 1, 2, 3]  # Delayed by 3 samples

    output_signal = [delay.step(x) for x in input_signal]
    assert output_signal == expected_output, "Step function is not producing the expected delay."


def test_delay_invalid_parameters():
    """Test invalid parameters for Delay filter initialization."""
    with pytest.raises(ValueError):
        Delay(Tc=0.01, L=-0.05)  # Negative delay should raise error

    with pytest.raises(ValueError):
        Delay(Tc=0.01, L=0)  # Zero delay should raise error

    with pytest.raises(ValueError):
        delay = Delay(Tc=0.01, L=0.05)
        delay.step("invalid")  # Non-numeric input should raise error


if __name__ == "__main__":
    pytest.main()