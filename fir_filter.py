import numpy as np
from base_filter import BaseFilter

class FIRFilter(BaseFilter):
    """
    FIRFilter - Discrete-time Finite Impulse Response (FIR) Filter implementation.
    
    This class represents a discrete-time FIR filter, which filters input signals
    by convolving them with a finite-length sequence of coefficients (filter taps).
    The filter coefficients determine the filter's frequency response.
    """
    
    def __init__(self, Tc: float, fir_coef: np.ndarray):
        """
        Constructor to create an FIRFilter object.
        
        :param Tc: Sampling time (must be a positive scalar).
        :param fir_coef: Coefficients of the FIR filter (must be a numeric vector).
        """
        super().__init__(Tc)
        
        if not isinstance(fir_coef, (list, np.ndarray)) or len(fir_coef) == 0:
            raise ValueError("fir_coef must be a non-empty numeric vector")
        
        self._fir_coef = np.array(fir_coef, dtype=float).reshape(-1, 1)  # Ensure column vector
        self._buffer = np.zeros((len(self._fir_coef), 1))  # Initialize buffer
    
    def initialize(self):
        """Initialize the state variable (reset the buffer)."""
        self._buffer.fill(0)
    
    def starting(self, input_value: float):
        """
        Set the starting conditions based on the input.
        
        :param input_value: Initial input value (must be a scalar).
        """
        if not isinstance(input_value, (int, float)):
            raise ValueError("Input must be a scalar")
        
        self._buffer.fill(input_value)
    
    def step(self, input_value: float) -> float:
        """
        Perform one step of the FIR filter and compute the output.
        
        :param input_value: Input value for the current step (must be a scalar).
        :return: Filtered output.
        """
        if not isinstance(input_value, (int, float)):
            raise ValueError("Input must be a scalar")
        
        # Update the buffer with the latest input
        self._buffer = np.roll(self._buffer, shift=1)
        self._buffer[0] = input_value
        
        # Compute the output using the FIR filter equation
        output = float(np.dot(self._buffer.T, self._fir_coef))
        return output
