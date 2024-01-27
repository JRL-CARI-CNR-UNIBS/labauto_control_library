% CascadeController - Class for implementing a cascade controller with inner and outer loops
%
% This class represents a cascade controller with two nested control loops.
% The outer loop controls the position, while the inner loop controls the velocity.
% The controller receives a reference signal, current state, and feedforward input,
% and computes the control action by combining the outputs of the inner and outer loops.
%
% Properties (Access = protected):
%   - InnerCtrl: Inner loop controller (instance of BaseController)
%   - OuterCtrl: Outer loop controller (instance of BaseController)
%
% Methods:
%   - CascadeController: Constructor to create CascadeController objects
%   - initialize: Method to initialize the inner and outer loop controllers
%   - starting: Method to set the starting conditions for the inner and outer loops
%   - computeControlAction: Method to compute the control action based on reference, state, and feedforward
%
% Constructor Input Parameters:
%   - st: Sampling time (scalar numeric)
%   - InnerCtrl: Inner loop controller object (instance of BaseController)
%   - OuterCtrl: Outer loop controller object (instance of BaseController)
%
classdef CascadeController < BaseController
   
    properties (Access = protected)
        InnerCtrl % Inner loop controller (instance of BaseController)
        OuterCtrl % Outer loop controller (instance of BaseController)
    end
    
    methods
        % Constructor for CascadeController class
        function obj = CascadeController(st, InnerCtrl, OuterCtrl)
            % Validate input parameters
            assert(isa(InnerCtrl, 'BaseController'), 'InnerCtrl must be an instance of BaseController');
            assert(isa(OuterCtrl, 'BaseController'), 'OuterCtrl must be an instance of BaseController');

            % Call the constructor of the superclass (BaseController)
            obj@BaseController(st);

            % Initialize properties with input parameters
            obj.InnerCtrl = InnerCtrl;
            obj.OuterCtrl = OuterCtrl;
        end

        % Method to initialize the inner and outer loop controllers
        function obj = initialize(obj)
            obj.OuterCtrl.initialize();
            obj.InnerCtrl.initialize();
        end

        % Method to set starting conditions for the inner and outer loops
        function obj = starting(obj, reference, y, u, uff)
            % Validate input parameters
            assert(isvector(reference) && length(reference) == 2, 'Reference must be a 2-element vector');
            assert(isvector(y) && length(y) == 2, 'State (y) must be a 2-element vector');
            assert(isscalar(u), 'Control action (u) must be a scalar');
            assert(isscalar(uff), 'Feedforward input (uff) must be a scalar');

            % Set starting conditions for the outer loop
            obj.OuterCtrl.starting(reference(1), y(1), y(2), reference(2));

            % Set starting conditions for the inner loop
            obj.InnerCtrl.starting(y(2), y(2), u, uff);
        end

        % Method to compute the control action based on reference, state, and feedforward
        function u = computeControlAction(obj, reference, y, uff)
            % Validate input parameters
            assert(isvector(reference) && length(reference) == 2, 'Reference must be a 2-element vector');
            assert(isvector(y) && length(y) == 2, 'State (y) must be a 2-element vector');
            assert(isscalar(uff), 'Feedforward input (uff) must be a scalar');

            % Use velocity=reference(2) as feedforward for the outer loop
            vel_ref = obj.OuterCtrl.computeControlAction(reference(1), y(1), reference(2));

            % Compute the control action using the inner loop
            u = obj.InnerCtrl.computeControlAction(vel_ref, y(2), uff);
        end
    end
end
