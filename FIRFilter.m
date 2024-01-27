% FIRFilter - Discrete-time Finite Impulse Response (FIR) Filter implementation
%
% This class represents a discrete-time FIR filter. FIR filters are used for
% filtering input signals by convolving them with a finite-length sequence of
% coefficients (filter taps). The filter coefficients determine the filter's
% frequency response.
%
% Properties (Access = protected):
%   - circular_buffer: Circular buffer to store input samples
%   - idx_buffer: Index to track the current position in the circular buffer
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
        circular_buffer % Circular buffer to store input samples
        idx_buffer % Index to track the current position in the circular buffer
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
            obj.circular_buffer = zeros(length(fir_coef),1);
            obj.fir_coef=fir_coef;
            if isrow(fir_coef)
                obj.fir_coef=fir_coef';
            else
                obj.fir_coef=fir_coef;
            end
            obj.idx_buffer=1;
            
        end

        % Method to initialize the state variable
        function obj = initialize(obj)
            % Reset the circular buffer and index
            obj.circular_buffer = zeros(length(obj.fir_coef),1);
            obj.idx_buffer=1;
        end

        % Method to set starting conditions based on the input
        function obj = starting(obj, input)
            % Validate input
            assert(isscalar(input), 'Input must be a scalar');
            
            % Initialize the circular buffer with the input value
            obj.circular_buffer = ones(length(obj.fir_coef),1) * input;
            obj.idx_buffer=1;
        end

        % Method to perform one step of the FIR filter and compute the output
        function output = step(obj, input)
            % Validate input
            assert(isscalar(input), 'Input must be a scalar');

            % Update the circular buffer with the latest input
            obj.circular_buffer(obj.idx_buffer) = input;

            % Compute the output using the FIR filter equation
            output = 0;
            for idx=0:length(obj.circular_buffer)-1
                ib = obj.idx_buffer + idx;
                if ib > length(obj.circular_buffer)
                    ib = ib - length(obj.circular_buffer);
                end
                output = output + obj.fir_coef(idx+1) * obj.circular_buffer(ib);
            end

            % Update the circular buffer index
            if obj.idx_buffer < length(obj.circular_buffer)
                obj.idx_buffer = obj.idx_buffer + 1;
            else
                obj.idx_buffer = 1;
            end
            
        end
    end
end
