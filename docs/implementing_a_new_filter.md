# How to implement a filter

To create a filter from the `BaseFilter` abstract base class (ABC), you need to create a concrete subclass that implements the abstract methods defined in the base class. Here's an example of how you can create a `DummyFilter` that inherits from `BaseFilter`:

```python
from abc import ABC, abstractmethod

class DummyFilter(BaseFilter):
    """
    DummyFilter - Concrete implementation of BaseFilter.
    
    This class implements the abstract methods defined in the BaseFilter
    class to create a simple dummy filter.
    """
    
    def __init__(self, Tc: float, a: float, b: float):
        """
        Constructor to create DummyFilter objects.
        
        :param Tc: Sampling time (cycle time), must be a positive scalar.
        :param a: Feedback coefficient for the dummy filter.
        :param b: Feedforward coefficient for the dummy filter.
        """
        super().__init__(Tc)
        self._a = a
        self._b = b
        self.initialize()
    
    def initialize(self):
        """Implement the initialize method from BaseFilter."""
        self._x_prev = 0.0  # Previous input value
        self._y_prev = 0.0  # Previous output value
    
    def starting(self, input_value):
        """Implement the starting method from BaseFilter."""
        self._x_prev = input_value
        self._y_prev = input_value
    
    def step(self, input_value):
        """Implement the step method from BaseFilter."""
        y = self._b * input_value + self._a * self._y_prev
        self._x_prev = input_value
        self._y_prev = y
        return y
```

In this example, the `DummyFilter` class inherits from the `BaseFilter` class and implements the abstract methods defined in the base class. The `DummyFilter` has two additional parameters `a` and `b` in its constructor, which represent the autoregressive (feedback) and exogenous (feedforward) coefficients of the dummy filter, respectively.

The `initialize()` method sets the initial values of the previous input `_x_prev` and previous output `_y_prev` to 0.0. The `starting()` method sets the initial values of `_x_prev` and `_y_prev` to the given input value. The `step()` method computes the current output `y` based on the current input, the previous output, and the filter coefficients `_a` and `_b`. It then updates the previous input and output values for the next step.

To use the `DummyFilter`, you can create an instance of the class and call its methods:

```python
# Create a DummyFilter instance
filter = DummyFilter(Tc=0.1, a=0.8, b=0.2)

# Start the filter
filter.starting(input_value=0.0)

# Apply the filter to an input sequence
input_values = [1.0, 2.0, 3.0, 4.0, 5.0]
output_values = [filter.step(x) for x in input_values]

print("Input values:", input_values)
print("Output values:", output_values)
```

In this example, we create a `DummyFilter` instance with a sampling time of 0.1 seconds, a feedback coefficient of 0.8, and a feedforward coefficient of 0.2. We then start the filter with an initial input value of 0.0 and apply the filter to a sequence of input values, storing the resulting output values.

The key steps in creating a filter from the `BaseFilter` ABC are:

1. Define a concrete subclass that inherits from `BaseFilter`.
2. Implement the abstract methods `initialize()`, `starting()`, and `step()` in the subclass.
3. Optionally, add any additional parameters or attributes specific to the filter implementation.
4. Create an instance of the subclass and use its methods to filter the input data.

The `BaseFilter` ABC provides a common interface for implementing various digital filters, allowing you to create custom filters that share a consistent structure and behavior.
