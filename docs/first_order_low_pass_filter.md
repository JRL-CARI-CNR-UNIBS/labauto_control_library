# FirstOrderLowPassFilter - Discrete-time First Order Low Pass Filter

## Overview
The `FirstOrderLowPassFilter` class implements a discrete-time first-order low-pass filter.
$\frac{1}{T_f s+1}$


## Constructor
```python
FirstOrderLowPassFilter(Tc: float, time_constant: float)
```
### Parameters:
- `Tc` (float): Sampling time (must be a positive scalar).
- `time_constant` (float): Time constant of the filter (must be a positive scalar).

## Methods
### `initialize()`
Initializes the state variable to zero.

### `starting(input_value: float)`
Sets the starting condition based on the given input value.

### `step(input_value: float) -> float`
Computes one step of the filter and returns the filtered output.

## Example Usage
```python
from first_order_lp_filter import FirstOrderLowPassFilter

# Create a low-pass filter with sampling time 0.01s and time constant 0.1s
filter = FirstOrderLowPassFilter(Tc=0.01, time_constant=0.1)

# Initialize the filter
filter.initialize()

# Set initial condition
filter.starting(1.0)

# Apply filtering
output = filter.step(0.8)
print("Filtered Output:", output)
```

## License
This project is open-source and can be used freely for research and development purposes.
