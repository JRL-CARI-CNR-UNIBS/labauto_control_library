import numpy as np
import matplotlib.pyplot as plt
from labauto import PinocchioRoboticSystem
from labauto import TrapezoidalMotionLaw
from labauto import loadController
from scipy.io import savemat
from datetime import datetime
import yaml
from scipy.signal import chirp

model_name = "scara0"


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

# define chirp
Duration = 20.0 # seconds
t = np.arange(0, Duration + Tc, Tc)  # Ensure inclusion of Duration if possible

f0=1.0
f1=500.0
A=10.0
joint_number=1
chirp_signal = A*chirp(t, f0=f0, f1=f1, t1=Duration, method='logarithmic')



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
max_Dq = np.array([5, 5])
max_DDq = np.array([5, 5])
ml = TrapezoidalMotionLaw(max_Dq, max_DDq, Tc)
ml.set_initial_condition(q0)

# Define a sequence of motion instructions
instructions = ["pause: 1", f"move: [0.0, {np.pi/2.0}]", "pause: 5"]
ml.add_instructions(instructions)

# Read the initial torque
joint_torque = robot.read_actuator_value()
feedforward_action = np.array([0.0, 0.0])
decentralized_ctrl.starting(initial_reference, measured_output, joint_torque, feedforward_action)

# Simulation loop
actual_time=0.0
while ml.depending_instructions():
    target_q, target_Dq, target_DDq = ml.compute_motion_law()
    reference = np.concatenate((target_q, target_Dq, target_DDq))
    measured_output = robot.read_sensor_value()
    joint_torque = decentralized_ctrl.compute_control_action(reference, measured_output, feedforward_action)
    robot.write_actuator_value(joint_torque)

    robot.simulate()




measured_signal, control_action=  [], []
feedforward_action = np.array([0.0, 0.0])

for actual_time,disturbance in zip(t,chirp_signal):
    target_q, target_Dq, target_DDq = ml.compute_motion_law()
    reference = np.concatenate((target_q, target_Dq, target_DDq))
    measured_output = robot.read_sensor_value()
    feedforward_action[joint_number]= disturbance
    joint_torque = decentralized_ctrl.compute_control_action(reference, measured_output, feedforward_action)
    robot.write_actuator_value(joint_torque)

    # Store data
    measured_signal.append(measured_output)
    control_action.append(joint_torque)

    robot.simulate()

# Convert lists to numpy arrays
measured_signal = np.array(measured_signal)
control_action = np.array(control_action)

# Post-processing
joint_position = measured_signal[:, :2]
joint_velocity = measured_signal[:, 2:]

# Save test data
timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
test_data = {
    "joint_position": joint_position,
    "joint_velocity": joint_velocity,
    "joint_torque": control_action,
    "chirp_signal": chirp_signal,
    "joint_number": joint_number,
    "f0": f0,
    "f1": f1,
    "A": A,
    "time": t
}
savemat(f"{model_name}/tests/chirp_experiment_joint{joint_number+1}_{timestamp}.mat", {key: test_data[key] for key in test_data})

# Plot results in 3x2 grid
fig, axes = plt.subplots(3, 2, figsize=(10, 10))
axes[0, 0].plot(t, measured_signal[:, 0], label='Position 1')
axes[0, 1].plot(t, measured_signal[:, 1], label='Position 2')
axes[1, 0].plot(t, measured_signal[:, 2], label='Velocity 1')
axes[1, 1].plot(t, measured_signal[:, 3], label='Velocity 2')
axes[2, 0].plot(t, control_action[:, 0], label='Torque 1')
axes[2, 1].plot(t, control_action[:, 1], label='Torque 2')
for ax in axes.flat:
    ax.grid()
    ax.legend()
plt.xlabel("Time (s)")
plt.show()
