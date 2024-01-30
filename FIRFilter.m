% FIRFilter - Discrete-time Finite Impulse Response (FIR) Filter implementation
%
% This class represents a discrete-time FIR filter. FIR filters are used for
% filtering input signals by convolving them with a finite-length sequence of
% coefficients (filter taps). The filter coefficients determine the filter's
% frequency response.
%
% Properties (Access = protected):
%   - buffer: buffer to store input samples
%   - fir_coef: Coefficients of the FIR filter
%
% Methods (Access = public):
%   - FIRFilter: Constructor to create FIRFilter objects
%   - initialize: Method to initialize the state variables of the filter
%   - starting: Method to set the starting conditions based on the input
%   - step: Method to perform one step of the FIR filter and compute the output
%
% Constructor Input Parameters:
%   - Tc: Sampling time (scalar numeric, Tc > 0)
%   - fir_coef: Coefficients of the FIR filter (numeric vector)
%
classdef FIRFilter < BaseFilter
    properties (Access = protected)
        buffer % buffer to store input samples
        fir_coef % Coefficients of the FIR filter
    end
    
    methods (Access = public)
        % Constructor for FirstOrderLowPassFilter class
        function obj = FIRFilter(Tc, fir_coef)
            % Validate input parameters
            assert(isvector(fir_coef), 'fir_coef must be a vector');
            
            % Call the constructor of the superclass (BaseFilter)
            obj@BaseFilter(Tc);
            
            % Initialize the state variable and coefficients
            obj.buffer = zeros(length(fir_coef),1);
            if isrow(fir_coef)
                obj.fir_coef=fir_coef';
            else
                obj.fir_coef=fir_coef;
            end
        end

        % Method to initialize the state variable
        function obj = initialize(obj)
            % Reset the buffer
            obj.buffer = zeros(length(obj.fir_coef),1);
        end

        % Method to set starting conditions based on the input
        function obj = starting(obj, input)
            % Validate input
            assert(isscalar(input), 'Input must be a scalar');
            
            % Initialize the buffer with the input value
            obj.buffer = ones(length(obj.fir_coef),1) * input;
        end

        % Method to perform one step of the FIR filter and compute the output
        function output = step(obj, input)
            % Validate input
            assert(isscalar(input), 'Input must be a scalar');

            % Update the buffer with the latest input
            obj.buffer = [input;obj.buffer(1:end-1)];

            % Compute the output using the FIR filter equation
            output = obj.buffer.'*obj.fir_coef;
            
        end
    end
end
