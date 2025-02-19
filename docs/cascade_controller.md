# CascadeController - Nested Control Loops

## Overview
The `CascadeController` class implements a cascade control structure with inner and outer loops. The outer loop controls position, while the inner loop controls velocity.

## Constructor
```python
CascadeController(Tc: float, inner_ctrl: BaseController, outer_ctrl: BaseController)
```
### Parameters:
- `Tc` (float): Sampling time, must be a positive scalar.
- `inner_ctrl` (`BaseController`): Controller for the inner velocity loop.
- `outer_ctrl` (`BaseController`): Controller for the outer position loop.

## Methods
### `initialize()`
Resets both inner and outer controllers.

### `set_umax(umax: float)`
Sets the maximum control action.

### `starting(reference: list, y: list, u: float, uff: float)`
Initializes the controllers with reference values, system state, control action, and feedforward input.

### `compute_control_action(reference: list, y: list, uff: float) -> float`
Computes the control action based on the reference, system state, and feedforward input.

## Example Usage
```python
from cascade_controller import CascadeController
from pid_controller import PIDController

# Create inner and outer PID controllers
inner_ctrl = PIDController(Tc=0.01, Kp=1.5, Ki=0.2, Kd=0.1)
outer_ctrl = PIDController(Tc=0.01, Kp=2.0, Ki=0.3, Kd=0.15)

# Create a cascade controller
controller = CascadeController(Tc=0.01, inner_ctrl=inner_ctrl, outer_ctrl=outer_ctrl)

# Initialize the controller
controller.initialize()

# Set initial conditions
reference = [1.0, 0.5]
y = [0.8, 0.3]
u = 0.0
uff = 0.0
controller.starting(reference, y, u, uff)

# Compute a control action
control_output = controller.compute_control_action(reference, y, uff)
print("Control Output:", control_output)
```

## License
This project is open-source and can be used freely for research and development purposes.