# BaseFilter - Abstract Class for Digital Filters

## Overview

The `BaseFilter` class is an abstract base class designed for implementing digital filters. It defines a common interface that must be followed by all derived filter classes, ensuring a consistent structure and behavior.


## Methods

### Constructor

```python
BaseFilter(Tc: float)
```

Initializes a filter with the given sampling time.

### Abstract Methods

Derived classes must implement the following methods:

- `initialize()`: Initializes filter variables.
- `starting(input_value)`: Sets the filter to a steady-state condition.
- `step(input_value)`: Computes the filter output given an input.

## Usage

To create a custom filter, inherit from `BaseFilter` and implement the abstract methods:

```python
from base_filter import BaseFilter

class CustomFilter(BaseFilter):
    def initialize(self):
        pass
    
    def starting(self, input_value):
        pass
    
    def step(self, input_value):
        return input_value  # Example implementation
```

## License

This project is open-source and can be used freely for research and development purposes.

