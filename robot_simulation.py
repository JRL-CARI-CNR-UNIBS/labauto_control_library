import numpy as np
import matplotlib.pyplot as plt
from labauto import PinocchioRoboticSystem
from labauto import TrapezoidalMotionLaw
from labauto import loadController
from labauto import loadInstructions

from scipy.io import savemat
from datetime import datetime
import yaml

model_name = "scara0"
#program_name="random_trj"
program_name="test_trj1"

# Load controller parameters and dynamic parameters
with open(f'{model_name}/control_config.yaml', 'r') as file:
    params_yaml = yaml.safe_load(file)
    controller_params = params_yaml['controller']
    dynamic_params = np.array(params_yaml['model_parameters'])

# Create simulator for SCARA robot
robot = PinocchioRoboticSystem(st=0.001, model_name=model_name)
robot.initialize()

# Set the cycle time (sampling time) for motion law updates
Tc = robot.get_sampling_period()

# Load the tuned controller using parameters from YAML
decentralized_ctrl=loadController(Tc,controller_params,dynamic_params,model_name)
decentralized_ctrl.initialize()
decentralized_ctrl.set_umax(robot.get_umax())


# Initial reference is equal to the initial state of the robot
measured_output = robot.read_sensor_value()
q0 = measured_output[:2]
Dq0 = measured_output[2:]
DDq0 = np.array([0, 0])
initial_reference = np.concatenate((q0, Dq0, DDq0))

# Define the Motion Law
max_Dq = np.array([2, 2])
max_DDq = np.array([5, 5])
ml = TrapezoidalMotionLaw(max_Dq, max_DDq, Tc)
ml.set_initial_condition(q0)

# Define a sequence of motion instructions
instructions = loadInstructions(f'{model_name}/{program_name}.txt')
ml.add_instructions(instructions)

# Read the initial torque
joint_torque = robot.read_actuator_value()
feedforward_action = np.array([0.0, 0.0])

decentralized_ctrl.starting(initial_reference, measured_output, joint_torque, feedforward_action)

# Simulation loop
t, measured_signal, control_action, reference_signal, link_position = [], [], [], [], []
actual_time=0.0
while ml.depending_instructions():
    target_q, target_Dq, target_DDq = ml.compute_motion_law()
    reference = np.concatenate((target_q, target_Dq, target_DDq))
    measured_output = robot.read_sensor_value()
    joint_torque = decentralized_ctrl.compute_control_action(reference, measured_output, feedforward_action)
    robot.write_actuator_value(joint_torque)

    # Store data
    t.append(actual_time)
    measured_signal.append(measured_output)
    control_action.append(joint_torque)
    reference_signal.append(reference)
    link_position.append(robot.link_position())
    actual_time+=Tc

    robot.simulate()

# Convert lists to numpy arrays
t = np.array(t)
measured_signal = np.array(measured_signal)
control_action = np.array(control_action)
reference_signal = np.array(reference_signal)
link_position = np.array(link_position)

# Post-processing
joint_position = measured_signal[:, :2]
joint_velocity = measured_signal[:, 2:]

reference_position = reference_signal[:, :2]
reference_velocity = reference_signal[:, 2:4]
reference_acceleration = reference_signal[:, 4:]

# Save test data
timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
test_data = {
    "reference_position": reference_position,
    "reference_velocity": reference_velocity,
    "reference_acceleration": reference_acceleration,
    "joint_position": joint_position,
    "joint_velocity": joint_velocity,
    "joint_torque": control_action,
    "link_position": link_position,
    "time": t,
    "name": "test"
}
savemat(f"{model_name}/tests/{program_name}_{timestamp}.mat", {key: test_data[key] for key in test_data})

# Plot results in 3x2 grid
fig, axes = plt.subplots(3, 2, figsize=(10, 10))
axes[0, 0].plot(t, measured_signal[:, 0], label='Position 1')
axes[0, 0].plot(t, reference_signal[:, 0], linestyle='dashed', label='Reference Position 1')
axes[0, 1].plot(t, measured_signal[:, 1], label='Position 2')
axes[0, 1].plot(t, reference_signal[:, 1], linestyle='dashed', label='Reference Position 2')
axes[1, 0].plot(t, measured_signal[:, 2], label='Velocity 1')
axes[1, 0].plot(t, reference_signal[:, 2], linestyle='dashed', label='Reference Velocity 1')
axes[1, 1].plot(t, measured_signal[:, 3], label='Velocity 2')
axes[1, 1].plot(t, reference_signal[:, 3], linestyle='dashed', label='Reference Velocity 2')
axes[2, 0].plot(t, control_action[:, 0], label='Torque 1')
axes[2, 1].plot(t, control_action[:, 1], label='Torque 2')
for ax in axes.flat:
    ax.grid()
    ax.legend()
plt.xlabel("Time (s)")
plt.show()
