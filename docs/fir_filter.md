# FIRFilter - Finite Impulse Response (FIR) Filter

## Overview
The `FIRFilter` class implements a discrete-time Finite Impulse Response (FIR) filter. 


## Constructor
```python
FIRFilter(Tc: float, fir_coef: np.ndarray)
```
### Parameters:
- `Tc` (float): Sampling time, must be a positive scalar.
- `fir_coef` (np.ndarray): Coefficients of the FIR filter, must be a non-empty numeric vector.

## Methods
### `initialize()`
Resets the internal buffer.

### `starting(input_value: float)`
Sets the initial buffer values to the given input.

### `step(input_value: float) -> float`
Processes the input signal and returns the filtered output.

## Example Usage
```python
import numpy as np
from fir_filter import FIRFilter

# Define filter coefficients
fir_coefficients = np.array([0.2, 0.5, 0.3])

# Create an FIR filter with a sampling time of 0.01s
filter = FIRFilter(Tc=0.01, fir_coef=fir_coefficients)

# Initialize the filter
filter.initialize()

# Set initial conditions
filter.starting(1.0)

# Process a new input sample
output = filter.step(0.8)
print("Filtered Output:", output)
```

## License
This project is open-source and can be used freely for research and development purposes.

