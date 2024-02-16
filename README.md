## Overview

This repository contains a collection of MATLAB classes designed to facilitate the development and implementation of control systems for mechatronic applications.

## Key Features

### Controller Classes

- **BaseControl:** Abstract class for designing control laws in mechatronic systems.
- **PIDController:** Proportional-Integral-Derivative controller with customizable gains and filtering options.
- **CascadeController:** Cascade control structure combining inner and outer controllers for improved performance [scheme](docs/cascade_controller.png).
- **DecentralizedController:** Decentralized control structure with precomputed torque [scheme](docs/decentralized_controller.png)

### Filter Classes

- **NotchFilter:** Discrete-time notch filter for mitigating specific frequency components.
- **FirstOrderLowPassFilter:** Discrete-time first-order low-pass filter for signal conditioning.
- **FIRFilter:** Discrete-time Finite Impulse Response (FIR) filter for smoothing signals.
- **BaseFilter:** Base class for filter implementations, providing common functionalities.

### Inverse kinematics

- **IkMotion:** class for computing forward and inverse kinematics.
- **MotionLaw:** Abstract class for computing motion law.
- **TrapezoidalMotionLaw:** class for computing trapezoidal motion law.


## Usage



