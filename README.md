## Overview

This repository contains a collection of Python classes designed to facilitate the development and implementation of control systems for mechatronic applications.

## Build Status

![CI Test](https://github.com/JRL-CARI-CNR-UNIBS/labauto_control_library/actions/workflows/ci.yml/badge.svg?branch=master)


## Table of Contents
- [Overview](#overview)
- [Key Features](#key-features)
  - [Controller Classes](#controller-classes)
  - [Filter Classes](#filter-classes)
  - [Motion Law](#motion-law)
- [Robot Model Structure](#robot-model-structure)
- [Simulation](#simulation)
- [Python Scripts](#python-scripts)
- [MATLAB Scripts](#matlab-scripts)
- [Testing & Debugging](#python-scripts-for-testing-and-debugging)

## Key Features

### Controller Classes

- **BaseController:** Abstract class for designing control laws in mechatronic systems.
- **PIDController:** Proportional-Integral-Derivative controller with customizable gains and filtering options.
- **CascadeController:** Cascade control structure combining inner and outer controllers for improved performance.
  
  ![Cascade Controller Scheme](docs/cascade_controller.png)

- **DecentralizedController:** Decentralized control structure with precomputed torque.
  
  ![Decentralized Controller Scheme](docs/decentralized_controller.png)

### Filter Classes

- **NotchFilter:** Discrete-time notch filter for mitigating specific frequency components.
- **FirstOrderLowPassFilter:** Discrete-time first-order low-pass filter for signal conditioning.
- **FIRFilter:** Discrete-time Finite Impulse Response (FIR) filter for smoothing signals.
- **BaseFilter:** Base class for filter implementations, providing common functionalities.

### Motion Law

- **MotionLaw:** Abstract class for computing motion law and performing a list of instructions.
- **TrapezoidalMotionLaw:** Class for computing trapezoidal motion law.

## Robot Model Structure
- **model.urdf** - Description of the robot.
- **config.yaml** - Description of flexible joints (motor inertia, friction, stiffness, and damping).
- **control_config.yaml** - Control parameters.

Each robot folder (e.g., `Scara0/`) should contain these three files.

## Simulation
- **mechanical_system.py** - Abstract class for simulating mechanical systems.
- **pinocchio_robotic_system.py** - Simulates a robot with flexible joints using the Spong model. Reads URDF and config from a folder.

## Python Scripts
- **robot_simulation.py** - Run a simulation of the robot performing a list of instructions.
- **identification_experiment.py** - Run a chirp identification experiment on the robot in a working point.
- **validation_experiment.py** - Run randomized chirp validation experiments on the robot in a working point.
- **validation_experiments_working_points.py** - Run chirp validation experiments on the robot in multiple working points.

## MATLAB Scripts
- **IdentificazioneGiunto1.mlx** - Identification of joint 1.
- **IdentificazioneGiunto2.mlx** - Identification of joint 2.
- **ValidazioneGiunto.m** - Validation in the same working point.
- **ValidazioneSpazioLavoro.m** - Validation in multiple working configurations.

## Python Scripts for Testing and Debugging
- **test_notch_filter.py** - Run tests on the notch filter.

