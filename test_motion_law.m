% test_motion_law - Example script demonstrating the use of TrapezoidalMotionLaw
%
% This script creates an instance of the TrapezoidalMotionLaw class and
% defines a sequence of motion instructions, including movements and pauses.
% The script then simulates the motion over time and plots the resulting
% positions, velocities, and accelerations.
%
% Instructions:
%   - The robot starts at the initial position.
%   - It moves to the position [2, -1] and pauses for 3 seconds.
%   - It then moves to the position [1, 0] and pauses for 3 seconds.
%   - Finally, it moves to the position [0, 3] and pauses for 3 seconds.
%
% Note: The TrapezoidalMotionLaw class is used to generate smooth and
% controlled motion profiles with trapezoidal velocity profiles.

% Clear workspace, close figures, and clear command window
clear all; close all; clc;

% Set the cycle time (sampling time) for motion law updates
Tc = 1e-3;

% Set the maximum velocities (max_Dy) and maximum accelerations (max_DDy)
max_Dy = [10; 4];
max_DDy = 30 * ones(2, 1);

q0=[0;0];
T=spong2_ctrl.fkFcn(q0);
y0=T([1 3],4);


% Create an instance of the TrapezoidalMotionLaw class
ml = TrapezoidalMotionLaw(max_Dy, max_DDy, Tc);
ml.setInitialCondition(y0);

% Define a sequence of motion instructions
ml.addInstructions("move: [0.04,0.05]");
ml.addInstructions("pause: 3");
ml.addInstructions("move: [0.03,0.03]");
ml.addInstructions("pause: 3");
ml.addInstructions("move: [0,.03]");
ml.addInstructions("pause: 3");

% Simulate the motion over time (5e4 iterations)
for idx = 1:5e4
    [y(idx, :), Dy(idx, :), DDy(idx, :)] = ml.computeMotionLaw();

    % Check if there are more instructions to follow
    if ~ml.dependingInstructions
        break;
    end
end

% Plot the resulting positions, velocities, and accelerations
subplot(3, 1, 1)
plot(y)
title('Positions (y)')
subplot(3, 1, 2)
plot(Dy)
title('Velocities (Dy)')
subplot(3, 1, 3)
plot(DDy)
title('Accelerations (DDy)')

return
ikm=IkMotion(Tc,q0,@spong2_ctrl.jacobFcn,@spong2_ctrl.jacobDotFcn,@spong2_ctrl.ikFcn,@spong2_ctrl.fkFcn);
try
    for idx=1:size(y,1)
        [q(idx,:),Dq(idx,:),DDq(idx,:)]=ikm.ik(y(idx,:)',Dy(idx,:)',DDy(idx,:)');
    end
end
figure
subplot(3, 1, 1)
plot(q)
title('Joint configuration (q)')
subplot(3, 1, 2)
plot(Dq)
title('Joint Velocities (Dq)')
subplot(3, 1, 3)
plot(DDq)
title('Joint Accelerations (DDq)')