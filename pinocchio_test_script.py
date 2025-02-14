import numpy as np
import matplotlib.pyplot as plt
import time
from pinocchio_robotic_system import PinocchioRoboticSystem

# Define simulation parameters
st = 1e-3  # Sampling time
r = PinocchioRoboticSystem(st, 'scara0')  # Initialize the robotic system

# Initialize the system
r.initialize()

# Simulation time vector
t = np.arange(0, 2 + st, st)  # From 0 to 5 seconds with step size st

# Storage for sensor measurements
measures = np.zeros((len(t), r.num_output))

# Measure execution time
start_time = time.time()

# Run simulation
for idx in range(len(t)):
    r.write_actuator_value(np.array([50,10]))
    measures[idx, :] = r.read_sensor_value()  # Read sensor values
    r.simulate()  # Simulate system dynamics

end_time = time.time()
execution_time = end_time - start_time
print(f"Execution Time: {execution_time:.4f} seconds")

# Plot results
plt.figure()
plt.plot(t, measures[:, :2])
plt.xlabel('Time [s]')
plt.ylabel('Measurements')
plt.title('Joint Position Measurements')
plt.legend(['Position 1', 'Position 2'])
plt.grid()
plt.show()
