% ScaraController - Controller for a SCARA (Selective Compliance Assembly Robot Arm) System
%
% This class represents a controller designed for a SCARA system. The controller
% consists of two inner controllers for the two joints of the SCARA arm, and it
% utilizes an inverse dynamics function for torque computation.
%
% Properties (Access = protected):
%   - joint1_ctrl: Inner controller for the first joint
%   - joint2_ctrl: Inner controller for the second joint
%   - inverse_dynamics_fcn: Inverse dynamics function for torque computation
%
% Methods (Access = public):
%   - ScaraController: Constructor to create ScaraController objects
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
classdef ScaraController < BaseController
   
    properties (Access = protected)
        joint1_ctrl % Inner controller for the first joint
        joint2_ctrl % Inner controller for the second joint 
        inverse_dynamics_fcn % Inverse dynamics function
    end
    
    methods
        % Constructor for CascadeController class
        function obj = ScaraController(st, joint1_ctrl, joint2_ctrl,inverse_dynamics_fcn)
            % Validate input parameters
            assert(isa(joint1_ctrl, 'BaseController'), 'joint1_ctrl must be an instance of BaseController');
            assert(isa(joint2_ctrl, 'BaseController'), 'joint1_ctrl must be an instance of BaseController');

            % Call the constructor of the superclass (BaseController)
            obj@BaseController(st);

            % Initialize properties with input parameters
            obj.joint1_ctrl = joint1_ctrl;
            obj.joint2_ctrl = joint2_ctrl;
            obj.inverse_dynamics_fcn = inverse_dynamics_fcn;
        end

        % Method to initialize the inner and outer loop controllers
        function obj = initialize(obj)
            obj.joint2_ctrl.initialize();
            obj.joint1_ctrl.initialize();
        end

        % Method to set starting conditions for the inner and outer loops
        function obj = starting(obj, reference, y, u, uff)
            % Validate input parameters
            assert(isvector(reference) && length(reference) == 6, 'Reference must be a 6-element vector');
            assert(isvector(y) && length(y) == 4, 'State (y) must be a 4-element vector');
            assert(isvector(u) && length(u) == 2, 'Control action (u) must be a 2-element vector');
            assert(isvector(uff) && length(uff) == 2, 'Feedforward input (uff) must be a 2-element vector');
            
            reference_joint1=reference([1 3]);
            reference_joint2=reference([2 4]);
            y_joint1=y([1 3]);
            y_joint2=y([2 4]);

            qref=reference([1 2]);
            Dqref=reference([3 4]);
            DDqref=reference([5 6]);

            % compute torque using dynamic model
            precomputed_torque=@obj.inverse_dynamics_fcn(...
                q(1),qref(2),...
                Dqref(1),Dqref(2),...
                DDqref(1),DDqref(2));

            uff=uff+precomputed_torque;

            obj.joint1_ctrl.starting(reference_joint1, y_joint1, u(1),uff(1));
            obj.joint2_ctrl.starting(reference_joint2, y_joint2, u(2),uff(2));

        end

        % Method to compute the control action based on reference, state, and feedforward
        function u = computeControlAction(obj, reference, y, uff)
            % Validate input parameters
            assert(isvector(reference) && length(reference) == 6, 'Reference must be a 6-element vector');
            assert(isvector(y) && length(y) == 4, 'State (y) must be a 4-element vector');
            assert(isvector(uff) && length(uff) == 2, 'Feedforward input (uff) must be a 2-element vector');
            
            reference_joint1=reference([1 3]);
            reference_joint2=reference([2 4]);
            y_joint1=y([1 3]);
            y_joint2=y([2 4]);

            qref=reference([1 2]);
            Dqref=reference([3 4]);
            DDqref=reference([5 6]);

            % compute torque using dynamic model
            precomputed_torque=@obj.inverse_dynamics_fcn(...
                q(1),qref(2),...
                Dqref(1),Dqref(2),...
                DDqref(1),DDqref(2));

            uff=uff+precomputed_torque;

            u(1)=obj.joint1_ctrl.computeControlAction(reference_joint1, y_joint1, uff(1));
            u(2)=obj.joint2_ctrl.computeControlAction(reference_joint2, y_joint2, uff(2));
        end
    end
end
