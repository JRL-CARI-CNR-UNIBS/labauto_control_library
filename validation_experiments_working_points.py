import numpy as np
import matplotlib.pyplot as plt
from labauto import PinocchioRoboticSystem
from labauto import TrapezoidalMotionLaw
from labauto import load_controller
from scipy.io import savemat
from datetime import datetime
import yaml

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

working_points=[[0.0,0.0],[np.pi/2,0.0],[np.pi/2.0,np.pi/2.0],[0.0,np.pi/2.0],[0.0,np.pi]]

# run
from scipy.signal import chirp

Duration = 20.0  # seconds
t = np.arange(0, Duration + Tc, Tc)  # Ensure inclusion of Duration if possible

f0 = 1.0
f1 = 500.0
A = 10.0
joint_number = 1
chirp_signal = A * chirp(t, f0=f0, f1=f1, t1=Duration, method='logarithmic')

# Define a sequence of motion instructions
joint_torque = robot.read_actuator_value()
feedforward_action = np.array([0.0, 0.0])
decentralized_ctrl.starting(initial_reference, measured_output, joint_torque, feedforward_action)

for wp in working_points:
    print(f"Running experiment in [{wp[0]}, {wp[1]}]")

    instructions = ["pause: 1", f"move: [{wp[0]}, {wp[1]}]", "pause: 5"]
    measured_output = robot.read_sensor_value()
    q0 = measured_output[:2]
    ml = TrapezoidalMotionLaw(max_Dq, max_DDq, Tc)
    ml.set_initial_condition(q0)
    ml.add_instructions(instructions)

    # Read the initial torque
    feedforward_action = np.array([0.0, 0.0])

    # Simulation loop


    print(f"Moving to [{wp[0]}, {wp[1]}]")
    while ml.depending_instructions():
        target_q, target_Dq, target_DDq = ml.compute_motion_law()
        reference = np.concatenate((target_q, target_Dq, target_DDq))
        measured_output = robot.read_sensor_value()
        joint_torque = decentralized_ctrl.compute_control_action(reference, measured_output, feedforward_action)
        robot.write_actuator_value(joint_torque)

        robot.simulate()



    measured_signal, control_action=  [], []
    feedforward_action = np.array([0.0, 0.0])

    print(f"start chirp")
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
    savemat(f"{model_name}/tests/wp_validation_chirp_experiment_joint{joint_number+1}_{timestamp}.mat", {key: test_data[key] for key in test_data})
