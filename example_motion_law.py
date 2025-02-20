import numpy as np
import matplotlib.pyplot as plt
from labauto import TrapezoidalMotionLaw

# Set the cycle time (sampling time) for motion law updates
Tc = 1e-3

# Set the maximum velocities (max_Dy) and maximum accelerations (max_DDy)
max_Dy = np.array([2, 4])
max_DDy = 3 * np.ones(2)

# Initial position
q0 = np.array([0, 0])
# Assuming a placeholder function for fkFcn (forward kinematics)
y0 = np.array([0.00, 0.05])  # Replace with actual computation if needed

# Create an instance of the TrapezoidalMotionLaw class
ml = TrapezoidalMotionLaw(max_Dy, max_DDy, Tc)
ml.set_initial_condition(y0)

# Define a sequence of motion instructions
ml.add_instructions(["move: [0.0, 0.0]"])
ml.add_instructions(["pause: 3"])
ml.add_instructions(["move: [0.0, 0.3]"])
ml.add_instructions(["pause: 3"])
ml.add_instructions(["move: [0.3, 0.3]"])
ml.add_instructions(["pause: 3"])
ml.add_instructions(["move: [0.3, 0.0]"])
ml.add_instructions(["pause: 3"])
ml.add_instructions(["move: [0.0, 0.0]"])
ml.add_instructions(["pause: 3"])

# Simulate the motion over time
num_iterations = int(5e4)
y, Dy, DDy = [], [], []

for idx in range(num_iterations):
    y_i, Dy_i, DDy_i = ml.compute_motion_law()
    
    y.append(y_i)
    Dy.append(Dy_i)
    DDy.append(DDy_i)
    
    # Check if there are more instructions to follow
    if not ml.depending_instructions:
        break

print(f"ended in {idx} iterations")
# Convert lists to numpy arrays
y = np.array(y)
Dy = np.array(Dy)
DDy = np.array(DDy)

# Plot results
plt.figure()
plt.plot(y[:, 0], y[:, 1], label='Position')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Motion Path')
plt.legend()
plt.grid()
plt.show()


plt.figure()
plt.plot(y, label=['Position X', 'Position Y'])
plt.xlabel('Time Steps')
plt.ylabel('Position')
plt.title('Position Profile')
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(Dy, label=['Velocity X', 'Velocity Y'])
plt.xlabel('Time Steps')
plt.ylabel('Velocity')
plt.title('Velocity Profile')
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(DDy, label=['Acceleration X', 'Acceleration Y'])
plt.xlabel('Time Steps')
plt.ylabel('Acceleration')
plt.title('Acceleration Profile')
plt.legend()
plt.grid()
plt.show()
