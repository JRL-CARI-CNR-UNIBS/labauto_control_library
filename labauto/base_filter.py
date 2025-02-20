from abc import ABC, abstractmethod

class BaseFilter(ABC):
    """
    BaseFilter - Abstract class for implementing digital filters.
    
    This class serves as a base class for implementing digital filters.
    It includes properties and abstract methods that should be implemented
    by derived filter classes.
    """
    
    def __init__(self, Tc: float):
        """
        Constructor to create BaseFilter objects.
        
        :param Tc: Sampling time (cycle time), must be a positive scalar.
        """
        if not isinstance(Tc, (int, float)) or Tc <= 0:
            raise ValueError("Sampling time (Tc) must be a positive scalar")
        
        self._Tc = Tc  # Protected attribute for sampling time
    
    @property
    def Tc(self) -> float:
        """Returns the sampling time (Tc)."""
        return self._Tc
    
    @abstractmethod
    def initialize(self):
        """Abstract method to initialize variables."""
        pass
    
    @abstractmethod
    def starting(self, input_value):
        """Abstract method to set the filter to a steady-state condition."""
        pass
    
    @abstractmethod
    def step(self, input_value):
        """Abstract method to compute the filter output given an input."""
        pass
