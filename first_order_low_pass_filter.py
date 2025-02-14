import math
from base_filter import BaseFilter

class FirstOrderLowPassFilter(BaseFilter):
    """
    FirstOrderLowPassFilter - Discrete-time First Order Low Pass Filter implementation.
    
    This class represents a discrete-time first-order low-pass filter. It filters
    input signals to attenuate high-frequency components, allowing only
    low-frequency components to pass through.
    """
    
    def __init__(self, Tc: float, time_constant: float):
        """
        Constructor to create FirstOrderLowPassFilter objects.
        
        :param Tc: Sampling time (Tc), must be a positive scalar.
        :param time_constant: Time constant of the low-pass filter, must be a non-negative scalar.
        """
        super().__init__(Tc)
        
        if not isinstance(time_constant, (int, float)) or time_constant <= 0:
            raise ValueError("time_constant must be a positive scalar")
        
        self._x = 0  # State variable
        self._A = math.exp(-Tc / time_constant)  # Coefficient for the exponential term
        self._B = 1 - self._A  # Coefficient for the input term
    
    def initialize(self):
        """Initialize the state variable."""
        self._x = 0
    
    def starting(self, input_value: float):
        """
        Set the starting conditions based on the input.
        
        :param input_value: Initial input value, must be a scalar.
        """
        if not isinstance(input_value, (int, float)):
            raise ValueError("Input must be a scalar")
        
        self._x = input_value
    
    def step(self, input_value: float) -> float:
        """
        Perform one step of the low-pass filter and compute the output.
        
        :param input_value: Input value for the current step, must be a scalar.
        :return: Filtered output.
        """
        if not isinstance(input_value, (int, float)):
            raise ValueError("Input must be a scalar")
        
        output = self._x
        self._x = self._A * self._x + self._B * input_value
        return output
