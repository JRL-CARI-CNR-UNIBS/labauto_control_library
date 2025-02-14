# BaseController - Abstract Class for Control Systems

## Overview
The `BaseController` class is an abstract base class for implementing control systems. It defines a common interface that must be followed by all derived controller classes, ensuring consistency and structure.


## Constructor
```python
BaseController(Tc: float)
```
### Parameters:
- `Tc` (float): Sampling time (must be a positive scalar).

## Methods
### `set_umax(umax: float)`
Sets the maximum control action.

### `get_umax() -> float`
Returns the maximum control action.

### `get_sampling_period() -> float`
Returns the sampling period (`Tc`).

### Abstract Methods (To be implemented by derived classes)
- `initialize()`: Initializes the controller.
- `starting(reference: float, y: float, u: float, uff: float)`: Starts the controller with bumpless transition.
- `compute_control_action(reference: float, y: float, uff: float) -> float`: Computes the control action.

## License
This project is open-source and can be used freely for research and development purposes.

