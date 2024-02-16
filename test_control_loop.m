
clear all; close all; clc;


% load the tuned controller

robot=ElasticRoboticSystem('spong1');
robot.initialize;

robot.show;
% Set the cycle time (sampling time) for motion law updates
Tc = robot.getSamplingPeriod;

% Define inner controller parameters
Kpv = 500;
Tiv = 1.0;
Kiv = Kpv / Tiv;
Tdv = 0.0;
Kdv = Kpv * Tdv;
Tfv = Tdv / 5;

notch_filter = [];
der_filter = [];

Kpp = 5;
Kip = 0;
Kdp = 0;

% Define control objects
inner_ctrl = PIDController(Tc, Kpv, Kiv, Kdv, der_filter, notch_filter, []);
outer_ctrl = PIDController(Tc, Kpp, Kip, Kdp, [], [], []);
cascade_ctrl = CascadeController(Tc, inner_ctrl, outer_ctrl);

cascade_ctrl.initialize;
cascade_ctrl.setUMax(robot.getUMax);
measured_output = robot.readSensorValue();
q0=measured_output(1);
Dq0=measured_output(2);
DDq0=0;

% initial reference is equal to the inital state of the robot
initial_reference=[q0;Dq0];


% define the Motion law
max_Dq=5;
max_DDq=10;
ml = TrapezoidalMotionLaw(max_Dq, max_DDq, Tc);
ml.setInitialCondition(q0);

% Define a sequence of motion instructions
instructions=["move: [1]";
              "pause: 1"];
ml.addInstructions(instructions);

% read the initial torque
joint_torque=robot.readActuatorValue;

% feedforward action, not used in this script
feedforward_action=0;

% starting controller
cascade_ctrl.starting(initial_reference,measured_output,joint_torque,feedforward_action);


% execute the code 
for idx = 1:10e3

    % ===========================
    % Cyclic control code - Begin
    % ===========================

    % compute joint target motion
    [target_q(idx,:),target_Dq(idx,:),target_DDq(idx,:)] = ml.computeMotionLaw();
    % define reference
    reference=[target_q(idx,:),target_Dq(idx,:)]';

    % joint position and velocity from encoder
    measured_output = robot.readSensorValue();

    % compute control law
    joint_torque = cascade_ctrl.computeControlAction(reference,measured_output,feedforward_action);

    % apply torque
    robot.writeActuatorValue(joint_torque);

    % ===========================
    % Cyclic control code -  End 
    % ===========================

    % simulate
    robot.simulate();

    % store variable for evaluation and plotting
    t(idx,:)=(idx-1)*Tc;
    measured_signal(idx,:)=measured_output;
    control_action(idx,:)=joint_torque;
    reference_signal(idx,:)=reference;
    full_position(idx,:)=robot.fullJointPosition;
    % Check if there are more instructions to follow
    if ~ml.dependingInstructions
        %break;
    end
end


%%
joint_position=measured_signal(:,1);
joint_velocity=measured_signal(:,2);

reference_position=reference_signal(:,1);
reference_velocity=reference_signal(:,2);

figure
subplot(3,1,1)
plot(t,joint_position(:,1),t,reference_position(:,1))
hold on
grid on
xlabel('Time')
ylabel('Position 1')
legend({'Position','Position reference'})

subplot(3,1,2)
plot(t,joint_velocity(:,1),t,reference_velocity(:,1))
hold on
grid on
xlabel('Time')
ylabel('Velocity 1')
legend({'Velocity','Velocity reference'})


subplot(3,1,3)
plot(t,control_action(:,1))
hold on
grid on
xlabel('Time')
ylabel('Torque 1')
legend({'Torque'})

