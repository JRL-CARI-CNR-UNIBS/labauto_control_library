% DecentralizedController - DecentralizedController for a 
% rigid body robot
%
% This class represents a controller designed for a multi-joint system.
% The controller consists of a inner controller for each joint of the arm,
% and it utilizes an inverse dynamics function for torque computation.
%
% Properties (Access = protected):
%   - joint1_ctrl: Inner controller for the first joint
%   - joint2_ctrl: Inner controller for the second joint
%   - inverse_dynamics_fcn: Inverse dynamics function for torque computation
%
% Methods (Access = public):
%   - DecentralizedController: Constructor to create DecentralizedController objects
%   - initialize: Method to initialize the inner and outer loop controllers
%   - starting: Method to set starting conditions for the inner and outer loops
%   - computeControlAction: Method to compute the control action based on reference, state, and feedforward
%
% Constructor Input Parameters:
%   - st: Sampling time (scalar numeric, st > 0)
%   - joint1_ctrl: Inner controller for the first joint (instance of BaseController)
%   - joint2_ctrl: Inner controller for the second joint (instance of BaseController)
%   - inverse_dynamics_fcn: Inverse dynamics function for torque computation (function handle)
%
classdef DecentralizedController < BaseController

    properties (Access = protected)
        joint_ctrls % Inner controller for the first joint
        inverse_dynamics_fcn % Inverse dynamics function
        njoints
    end

    methods
        % Constructor for CascadeController class
        function obj = DecentralizedController(st, joint_ctrls, inverse_dynamics_fcn)
            % Validate input parameters
            assert(isa(joint_ctrls, 'BaseController'), 'joint_ctrls must be an instance of BaseController');
            assert(isvector(joint_ctrls), 'joint_ctrls must be a vector of BaseController')

            % Call the constructor of the superclass (BaseController)
            obj@BaseController(st);

            % Initialize properties with input parameters
            obj.joint_ctrls = joint_ctrls;
            obj.njoints=length(joint_ctrls);
            
            if nargin==3
                obj.inverse_dynamics_fcn = inverse_dynamics_fcn;
            else
                obj.inverse_dynamics_fcn = @(q,Dq,DDq)zeros(obj.njoints,1);
            end

            for idx=1:obj.njoints
                obj.umax(idx,1)=joint_ctrls(idx).getUMax();
            end
        end

        % Method to initialize the inner and outer loop controllers
        function obj = initialize(obj)
            for idx=1:obj.njoints
                obj.joint_ctrls(idx).initialize();
            end
        end

        % Method to set starting conditions for the inner and outer loops
        function obj = starting(obj, reference, y, u, uff)
            % Validate input parameters
            assert(isvector(reference) && length(reference) == 3*obj.njoints, 'Reference must be a (3n)-element vector');
            assert(isvector(y) && length(y) == 2*obj.njoints, 'State (y) must be a (2n)-element vector');
            assert(isvector(u) && length(u) == obj.njoints, 'Control action (u) must be a n-element vector');
            assert(isvector(uff) && length(uff) == obj.njoints, 'Feedforward input (uff) must be a n-element vector');

            q=y(1:obj.njoints);
            Dq=y(obj.njoints+(1:obj.njoints));
            qref=reference(1:obj.njoints);
            Dqref=reference(obj.njoints+(1:obj.njoints));
            DDqref=reference(2*obj.njoints+(1:obj.njoints));

            % compute torque using dynamic model
            precomputed_torque=obj.inverse_dynamics_fcn(...
                qref,...
                Dqref,...
                DDqref)';

            uff=uff+precomputed_torque;

            for idx=1:obj.njoints
                reference_single_joint=[qref(idx);Dqref(idx)];
                y_single_joint=[q(idx);Dq(idx)];
                u_single_joint=u(idx);
                uff_single_joint=uff(idx);
                obj.joint_ctrls(idx).starting(reference_single_joint, ...
                    y_single_joint, ...
                    u_single_joint, ...
                    uff_single_joint);
            end
        end

        % Method to compute the control action based on reference, state, and feedforward
        function u = computeControlAction(obj, reference, y, uff)
            % Validate input parameters
            assert(isvector(reference) && length(reference) == 6, 'Reference must be a 6-element vector');
            assert(isvector(y) && length(y) == 4, 'State (y) must be a 4-element vector');
            assert(isvector(uff) && length(uff) == 2, 'Feedforward input (uff) must be a 2-element vector');

            q=y(1:obj.njoints);
            Dq=y(obj.njoints+(1:obj.njoints));
            qref=reference(1:obj.njoints);
            Dqref=reference(obj.njoints+(1:obj.njoints));
            DDqref=reference(2*obj.njoints+(1:obj.njoints));

            % compute torque using dynamic model
            precomputed_torque=obj.inverse_dynamics_fcn(...
                qref,...
                Dqref,...
                DDqref)';
            uff=uff+precomputed_torque;

            for idx=1:obj.njoints
                reference_single_joint=[qref(idx);Dqref(idx)];
                y_single_joint=[q(idx);Dq(idx)];
                uff_single_joint=uff(idx);
                u(idx,1)=obj.joint_ctrls(idx).computeControlAction(reference_single_joint, ...
                    y_single_joint, ...
                    uff_single_joint);
            end
        end
    end
end
