# How to implement a controller

To create a controller from the `BaseController` abstract base class (ABC), you need to create a concrete subclass that implements the abstract methods defined in the base class. Here's an example of how you can create a `DummyController` that inherits from `BaseController`:

```python
from labauto import BaseController

class DummyController(BaseController):
    """
    DummyController - Concrete implementation of BaseController.
    
    This class implements the abstract methods defined in the BaseController
    class to create a simple dummy controller.
    """
    
    def __init__(self, Tc: float, k: float):
        """
        Constructor to create DummyController objects.
        
        :param Tc: Sampling time (cycle time), must be a positive scalar.
        :param k: Gain for the dummy controller.
        """
        super().__init__(Tc)
        self._k = k
        self.initialize()
    
    def initialize(self):
        """Implement the initialize method from BaseController."""
        self._x = 0.0  # Internal state variable
    
    def starting(self, reference: float, y: float, u: float, uff: float):
        """Implement the starting method from BaseController."""
        self._x = y  # Set the initial state to the current output
    
    def compute_control_action(self, reference: float, y: float, uff: float) -> float:
        """Implement the compute_control_action method from BaseController."""
        self._x += self._Tc * (reference - y)  # Update the internal state
        return self._k * self._x  # Compute the control action
```

In this example, the `DummyController` class inherits from the `BaseController` class and implements the abstract methods defined in the base class. The `DummyController` has an additional parameter `k` in its constructor, which represents the gain of the dummy controller.

The `initialize()` method sets the initial value of the internal state variable `_x` to 0.0. The `starting()` method sets the initial state of the controller to the current output `y`. The `compute_control_action()` method updates the internal state variable `_x` based on the difference between the reference and the output, and then computes the control action by multiplying the state variable by the gain `_k`.

To use the `DummyController`, you can create an instance of the class and call its methods:

```python
# Create a DummyController instance
controller = DummyController(Tc=0.1, k=2.0)

# Set the maximum control action
controller.set_umax(10.0)

# Start the controller
controller.starting(reference=5.0, y=0.0, u=0.0, uff=0.0)

# Compute the control action
u = controller.compute_control_action(reference=5.0, y=2.0, uff=0.0)
print(f"Control action: {u}")
```

In this example, we create a `DummyController` instance with a sampling time of 0.1 seconds and a gain of 2.0. We then set the maximum control action to 10.0, start the controller, and compute the control action based on a reference of 5.0 and an output of 2.0.

The key steps in creating a controller from the `BaseController` ABC are:

1. Define a concrete subclass that inherits from `BaseController`.
2. Implement the abstract methods `initialize()`, `starting()`, and `compute_control_action()` in the subclass.
3. Optionally, add any additional parameters or attributes specific to the controller implementation.
4. Create an instance of the subclass and use its methods to control the system.

The `BaseController` ABC provides a common interface for implementing various control systems, allowing you to create custom controllers that share a consistent structure and behavior.
