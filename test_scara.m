
clear all; close all; clc;




% create simulator for scara robot
robot=ElasticRoboticSystem('scara');
robot.initialize;

% Set the cycle time (sampling time) for motion law updates
Tc = robot.getSamplingPeriod;

% load the tuned controller
load +scara_data\scara_controller_object.mat
decentralized_ctrl.initialize;


% initial reference is equal to the inital state of the robot
measured_output = robot.readSensorValue();
q0=measured_output(1:2);
Dq0=measured_output(3:4);
DDq0=[0;0];

initial_reference=[q0;Dq0;DDq0];

% define the Motion law
max_Dq=[5;5];
max_DDq=[5;5];
ml = TrapezoidalMotionLaw(max_Dq, max_DDq, Tc);
ml.setInitialCondition(q0);

% Define a sequence of motion instructions
instructions=["pause: 1";
              "move: [0, 1.57]";
              "move: [0.5, 1.57]";
              "move: [0.5, 1.0]";
              "move: [-0.5, 1.2]";
              "move: [-1.5, -2]";
              "move: [0, 0]";
              "move: [0, 1.57]";
              "pause: 1";
              ];
ml.addInstructions(instructions);



% read the initial torque
joint_torque=robot.readActuatorValue;

% feedforward action, not used in this script
feedforward_action=[0;0];

% starting controller
decentralized_ctrl.starting(initial_reference,measured_output,joint_torque,feedforward_action);



% execute the code
for idx = 1:600e3

    % compute joint target motion
    [target_q(idx,:),target_Dq(idx,:),target_DDq(idx,:)] = ml.computeMotionLaw();
    % define reference
    reference=[target_q(idx,:),target_Dq(idx,:),target_DDq(idx,:)]';


    % y joint position from encoder
    measured_output = robot.readSensorValue();

    % compute control law
    joint_torque = decentralized_ctrl.computeControlAction(reference,measured_output,feedforward_action);

    % apply torque
    robot.writeActuatorValue(joint_torque);

    % simulate
    robot.simulate();

    % store variable for evaluation and plotting
    t(idx,:)=(idx-1)*Tc;
    measured_signal(idx,:)=measured_output;
    control_action(idx,:)=joint_torque;
    reference_signal(idx,:)=reference;
    full_position(idx,:)=robot.fullJointPosition;
    if ~ml.dependingInstructions
        break;
    end
end


%%
joint_position=measured_signal(:,[1 2]);
joint_velocity=measured_signal(:,[3 4]);

% acceleration is computed Savitzky-Golay filter
joint_acceleration = zeros(length(joint_velocity),size(joint_position,2));


% The Savitzky-Golay filter is a smoothing filter that fits a polynomial of
% a specified order to the data within a sliding window and then evaluates
% the derivative of that polynomial at the center point of the window.
window=2;
[b,g] = sgolay(1,1+2*window);

g_filter=g(:,1)'; % moving average
g_filter_der=g(:,2)'/Tc; % first derivative

for iax = 1:size(joint_position,2)
    for idx = (1+window):(size(joint_position,1)-window)
        joint_acceleration(idx,iax) = g_filter_der*joint_velocity((idx-window):(idx+window),iax);
    end
end
 
reference_position=reference_signal(:,[1 2]);
reference_velocity=reference_signal(:,[3 4]);
reference_acceleration=reference_signal(:,[5 6]);

figure
subplot(4,2,1)
plot(t,joint_position(:,1),t,reference_position(:,1))
hold on
grid on
xlabel('Time')
ylabel('Position 1')
legend({'Position','Position reference'})

subplot(4,2,3)
plot(t,joint_velocity(:,1),t,reference_velocity(:,1))
hold on
grid on
xlabel('Time')
ylabel('Velocity 1')
legend({'Velocity','Velocity reference'})

subplot(4,2,5)
plot(t,joint_acceleration(:,1),t,reference_acceleration(:,1))
hold on
grid on
xlabel('Time')
ylabel('Acceleration 1')
legend({'Acceleration','Acceleration reference'})

subplot(4,2,7)
plot(t,control_action(:,1))
hold on
grid on
xlabel('Time')
ylabel('Torque 1')
legend({'Torque'})


subplot(4,2,2)
plot(t,joint_position(:,2),t,reference_position(:,2))
hold on
grid on
xlabel('Time')
ylabel('Position 2')
legend({'Position','Position reference'})

subplot(4,2,4)
plot(t,joint_velocity(:,2),t,reference_velocity(:,2))
hold on
grid on
xlabel('Time')
ylabel('Velocity 2')
legend({'Velocity','Velocity reference'})

subplot(4,2,6)
plot(t,joint_acceleration(:,2),t,reference_acceleration(:,2))
hold on
grid on
xlabel('Time')
ylabel('Acceleration 2')
legend({'Acceleration','Acceleration reference'})

subplot(4,2,8)
plot(t,control_action(:,2))
hold on
grid on
xlabel('Time')
ylabel('Torque 2')
legend({'Torque'})


test_data.full_position=full_position;
test_data.joint_position=joint_position;
test_data.joint_velocity=joint_velocity;
test_data.joint_acceleration=joint_acceleration;
test_data.joint_torque=control_action;
test_data.time=t;
test_data.name="test";
save +scara_data/test_movement  test_data
%%
