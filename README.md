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


## Scripts for testing an application
- **test_scara_application.m** runs several moving programs and evaluate the execution. For simulation, you need to provide a +package to simulate the system and a +package with the functions to compute kinematics and dynamics regressors functions.


## Scripts for testing and debugging
- **test_cascade_controller.m**
- **test_control_loop.m**
- **test_filters.m**
- **test_motion_law.m**
- **test_open_loop.m**
- **test_scara.m**
- **test_scara_application.m**
