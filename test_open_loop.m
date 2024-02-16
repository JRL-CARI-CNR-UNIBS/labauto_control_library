
clear all; close all; clc;


% load the tuned controller
model='spong3';
robot=ElasticRoboticSystem(model);
robot.show;
% Set the cycle time (sampling time) for motion law updates
Tc = robot.getSamplingPeriod;

joint_torque=robot.readActuatorValue;
dof=length(joint_torque);
joint_torque=zeros(dof,1);

robot.initialize;

% execute the code 
for idx = 1:60e3

    % ===========================
    % Cyclic control code - Begin
    % ===========================

    % joint position and velocity from encoder
    measured_output = robot.readSensorValue();

    %joint_torque = cascade_ctrl.computeControlAction(reference,measured_output,feedforward_action);


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
end


%%
joint_position=measured_signal(:,1:dof);
joint_velocity=measured_signal(:,(1:dof)+dof);

for iax=1:dof
figure
subplot(3,1,1)
plot(t,joint_position(:,iax))
hold on
grid on
xlabel('Time')
ylabel(sprintf('Position %d',iax))
legend({'Position'})

subplot(3,1,2)
plot(t,joint_velocity(:,iax))
hold on
grid on
xlabel('Time')
ylabel(sprintf('Velocity %d',iax))
legend({'Velocity'})


subplot(3,1,3)
plot(t,control_action(:,iax))
hold on
grid on
xlabel('Time')
ylabel(sprintf('Torque %d',iax))
legend({'Torque'})

end
drawnow

load(['+',model,filesep,'corke'])
if isa(show_robot,'SerialLink')
    figure
    show_robot.plot(joint_position(1:20:end,:),'fps',50)
else
    %% If you want to visualize the robot, you need to install the Corke ToolBox
    % https://petercorke.com/toolboxes/robotics-toolbox/
    % and add it to the path
end
