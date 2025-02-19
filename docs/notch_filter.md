# NotchFilter - Discrete-Time Notch Filter

## Overview
The `NotchFilter` class implements a discrete-time notch filter designed to attenuate a specific frequency while allowing others to pass. This is useful for filtering out undesired oscillations in control and signal processing applications.

The continuous transfer function is

$F(s)=\frac{s^2+2\xi_z s \omega_n +\omega_n^2}{s^2+2\xi_z s \omega_n +\omega_n^2}$

discretized with Tustin method using _Tc_ as sample period.



## Constructor
```python
NotchFilter(Tc: float, wn: float, xi_z: float, xi_p: float)
```
### Parameters:
- `Tc` (float): Sampling time (must be a positive scalar).
- `wn` (float): Natural frequency in radians per second.
- `xi_z` (float): Damping ratio of the zero (xi_z<xi_p).
- `xi_p` (float): Damping ratio of the pole.

## Methods
### `initialize()`
Resets the filter state.

### `step(input_value: float) -> float`
Processes an input sample and returns the filtered output.

## Example Usage
```python
from notch_filter import NotchFilter

# Define filter parameters
Tc = 0.001  # Sampling time (1ms)
wn = 50.0   # Notch frequency in rad/s
xi_z = 0.1  # Zero damping ratio
xi_p = 0.05 # Pole damping ratio

# Create notch filter
notch = NotchFilter(Tc, wn, xi_z, xi_p)

# Initialize filter
notch.initialize()

# Process input samples
input_signal = [1.0, 0.5, -0.2, 0.1, -0.1]
filtered_output = [notch.step(x) for x in input_signal]
print("Filtered Output:", filtered_output)
```

## License
This project is open-source and can be used freely for research and development purposes.
