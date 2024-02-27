% FirstOrderLowPassFilter - Discrete-time First Order Low Pass Filter implementation
%
% This class represents a discrete-time first-order low-pass filter. It filters
% input signals to attenuate high-frequency components, allowing only
% low-frequency components to pass through. The filter is implemented using
% a simple first-order recursive equation.
%
% Properties (Access = protected):
%   - x: State variable of the low-pass filter
%   - A: Coefficient for the filter state in the filter equation
%   - B: Coefficient for the input term in the filter equation
%
% Methods (Access = public):
%   - FirstOrderLowPassFilter: Constructor to create FirstOrderLowPassFilter objects
%   - initialize: Method to initialize the state variable of the filter
%   - starting: Method to set the starting conditions based on the input
%   - step: Method to perform one step of the low-pass filter and compute the output
%
% Constructor Input Parameters:
%   - Tc: Sampling time (scalar numeric, Tc > 0)
%   - time_constant: Time constant of the low-pass filter (scalar numeric, time_constant >= 0)
%
classdef FirstOrderLowPassFilter < BaseFilter
    properties (Access = protected)
        x % State variable of the low-pass filter
        A % Coefficient for the exponential term in the filter equation
        B % Coefficient for the input term in the filter equation
    end
    
    methods (Access = public)
        % Constructor for FirstOrderLowPassFilter class
        function obj = FirstOrderLowPassFilter(Tc, time_constant)
            % Validate input parameters
            assert(isscalar(time_constant) && time_constant > 0, 'time_constant must be a non-negative scalar');
            
            % Call the constructor of the superclass (BaseFilter)
            obj@BaseFilter(Tc);
            
            % Initialize the state variable
            obj.x = 0;
            
            % Coefficients for the filter equation
            obj.A = exp(-Tc / time_constant);
            obj.B = 1 - obj.A;
        end

        % Method to initialize the state variable
        function obj = initialize(obj)
            obj.x = 0;
        end

        % Method to set starting conditions based on the input
        function obj = starting(obj, input)
            % Validate input
            assert(isscalar(input), 'Input must be a scalar');
            obj.x = input;
        end

        % Method to perform one step of the low-pass filter and compute the output
        function output = step(obj, input)
            % Validate input
            assert(isscalar(input), 'Input must be a scalar');

            % Compute output and update state variable
            output = obj.x;
            obj.x = obj.A * obj.x + obj.B * input;
        end
    end
end
