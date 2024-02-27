% BaseController - Abstract class for implementing control systems
%
% This abstract class serves as a base class for implementing control systems.
% It includes properties and abstract methods that should be implemented
% by derived controller classes.
%
% Properties (Access = protected):
%   - Tc: Sampling time (cycle time)
%   - umax: Maximum control action
%
% Methods (Access = public):
%   - BaseController: Constructor to create BaseController objects
%   - setUMax: Method to set the maximum control action
%   - getSamplingPeriod: Method to get the sampling period (Tc)
%
% Abstract Methods (Access = public):
%   - initialize: Abstract method to (re)initialize the controller
%   - starting: Abstract method to start the controller with bumpless transition
%   - computeControlAction: Abstract method to compute the control action
%
classdef (Abstract) BaseController < handle
    properties (Access = protected)
        Tc % Sampling time (cycle time)
        umax % Maximum control action
    end

    methods
        % Constructor for BaseController class
        function obj = BaseController(Tc)
            % Assign the sampling time to the property
            obj.Tc = Tc;
        end

        % Method to set the maximum control action
        function setUMax(obj, umax)
            obj.umax = umax;
        end
        % Method to set the maximum control action
        function umax=getUMax(obj)
            umax=obj.umax;
        end
        % Method to get the sampling period
        function st = getSamplingPeriod(obj)
            st = obj.Tc;
        end
    end

    methods (Abstract, Access = public)
        % Abstract methods to be implemented in derived classes

        % Method to (re)initialize the controller
        obj = initialize(obj)

        % Method to start the controller with bumpless transition
        %   reference = setpoint
        %   y = output
        %   u = control action
        %   uff = feedforward action
        obj = starting(obj, reference, y, u, uff)

        % Method to compute the control action
        %   reference = setpoint
        %   y = output
        %   u = control action
        %   uff = feedforward action
        u = computeControlAction(obj, reference, y, uff)
    end
end
