clear all;close all;clc

% compute dynamic parameters

load +scara_data\test_movement.mat

ndata=length(test_data.time);

joint_position=test_data.joint_position;
joint_velocity=test_data.joint_velocity;
joint_torque=test_data.joint_torque;
joint_position_filt=joint_position;
joint_velocity_filt=joint_velocity;
joint_acceleration_filt=zeros(size(joint_velocity,1),size(joint_velocity,2));
joint_torque_filt=joint_torque;

% The Savitzky-Golay filter is a smoothing filter that fits a polynomial of
% a specified order to the data within a sliding window and then evaluates
% the derivative of that polynomial at the center point of the window.
window=30; % don't filter too much!
[b,g] = sgolay(1,1+2*window);

Tc=1e-3;
g_filter=g(:,1)'; % moving average
g_filter_der=g(:,2)'/Tc; % first derivative

% filtering data (filtering introduces lag, every signals must have the
% same lag)
for iax = 1:size(joint_position,2)
    for idx = (1+window):(size(joint_position,1)-window)
        joint_position_filt(idx,iax) = g_filter*joint_position((idx-window):(idx+window),iax);
        joint_velocity_filt(idx,iax) = g_filter*joint_velocity((idx-window):(idx+window),iax);
        joint_torque_filt(idx,iax) = g_filter*joint_torque((idx-window):(idx+window),iax);

        joint_acceleration_filt(idx,iax) = g_filter_der*joint_velocity((idx-window):(idx+window),iax);
    end
end



FullRegressor=[];
FullTorque=[];

for idx=1:ndata
    q=joint_position_filt(idx,:)';
    Dq=joint_velocity_filt(idx,:)';
    DDq=joint_acceleration_filt(idx,:)';
    tau=joint_torque(idx,:)';
    dynamic_regressor=scara_ctrl.regressorFcn(q,Dq,DDq);
    friction_regressor=[diag(sign(Dq)) Dq];
    FullRegressor=[FullRegressor;
                   dynamic_regressor friction_regressor];
    FullTorque=[FullTorque;
                tau];
end



%%
dynamic_parameters=FullRegressor\FullTorque;
tau_model=FullRegressor*dynamic_parameters;
tau_model=reshape(tau_model,2,ndata)';
figure(1)
subplot(1,2,1)
plot(test_data.time,test_data.joint_torque(:,1))
hold on
plot(test_data.time,tau_model(:,1),'--')

subplot(1,2,2)
plot(test_data.time,test_data.joint_torque(:,2))
hold on
plot(test_data.time,tau_model(:,2),'--')

save +scara_data\dynamic_paramers dynamic_parameters