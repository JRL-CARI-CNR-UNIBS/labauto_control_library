from abc import ABC, abstractmethod

class BaseController(ABC):
    """
    BaseController - Abstract class for implementing control systems.
    
    This abstract class serves as a base class for implementing control systems.
    It includes properties and abstract methods that should be implemented
    by derived controller classes.
    """
    
    def __init__(self, Tc: float):
        """
        Constructor to create BaseController objects.
        
        :param Tc: Sampling time (cycle time), must be a positive scalar.
        """
        if not isinstance(Tc, (int, float)) or Tc <= 0:
            raise ValueError("Sampling time (Tc) must be a positive scalar")
        
        self._Tc = Tc  # Protected attribute for sampling time
        self._umax = float('inf') # Maximum control action (initialized as infinit)
    
    def set_umax(self, umax: float):
        """Set the maximum control action."""
        if not isinstance(umax, (int, float)):
            raise ValueError("Maximum control action (umax) must be a scalar")
        self._umax = umax
    
    def get_umax(self) -> float:
        """Get the maximum control action."""
        return self._umax
    
    def get_sampling_period(self) -> float:
        """Get the sampling period (Tc)."""
        return self._Tc
    
    @abstractmethod
    def initialize(self):
        """Abstract method to (re)initialize the controller."""
        pass
    
    @abstractmethod
    def starting(self, reference: float, y: float, u: float, uff: float):
        """
        Abstract method to start the controller with a bumpless transition.
        
        :param reference: Setpoint.
        :param y: Output.
        :param u: Control action.
        :param uff: Feedforward action.
        """
        pass
    
    @abstractmethod
    def compute_control_action(self, reference: float, y: float, uff: float) -> float:
        """
        Abstract method to compute the control action.
        
        :param reference: Setpoint.
        :param y: Output.
        :param uff: Feedforward action.
        :return: Computed control action.
        """
        pass
