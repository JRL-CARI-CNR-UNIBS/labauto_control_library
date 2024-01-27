% NotchFilter - Discrete-time notch filter implementation
%
% This class represents a discrete-time notch filter. It filters input signals
% to attenuate frequencies around a specified notch frequency while allowing
% other frequencies to pass through. The filter is implemented using state-space
% representation with a notch filter transfer function.
%
% Properties (Access = protected):
%   - A: State matrix of the notch filter
%   - B: Input matrix of the notch filter
%   - C: Output matrix of the notch filter
%   - D: Feedforward matrix of the notch filter
%   - x: State vector of the notch filter
%
% Methods (Access = public):
%   - NotchFilter: Constructor to create NotchFilter objects
%   - initialize: Method to initialize the state vector of the filter
%   - starting: Method to set the starting conditions based on the input
%   - step: Method to perform one step of the notch filter and compute the output
%
% Constructor Input Parameters:
%   - Tc: Sampling time (scalar numeric, Tc > 0)
%   - wn: Notch frequency (scalar numeric, wn >= 0)
%   - xi_zero: Damping ratio for zero (scalar numeric, xi_zero >= 0)
%   - xi_pole: Damping ratio for pole (scalar numeric, xi_pole >= 0)
%
classdef NotchFilter < BaseFilter
    properties (Access = protected)
        A % State matrix of the notch filter
        B % Input matrix of the notch filter
        C % Output matrix of the notch filter
        D % Feedforward matrix of the notch filter
        x % State vector of the notch filter
    end
    
    methods (Access = public)
        % Constructor for NotchFilter class
        function obj = NotchFilter(Tc, wn, xi_zero, xi_pole)
            % Validate input parameters
            assert(isscalar(xi_zero) && xi_zero >= 0, 'xi_zero must be a non-negative scalar');
            assert(isscalar(xi_pole) && xi_pole >= 0, 'xi_pole must be a non-negative scalar');
            assert(isscalar(wn) && wn >= 0, 'wn must be a non-negative scalar');
            assert(isscalar(Tc) && Tc > 0, 'Tc must be a positive scalar');
            
            % Call the constructor of the superclass (BaseFilter)
            obj@BaseFilter(Tc);
            
            % Initialize the state vector
            obj.x = [0; 0];

            % Define continuous-time state-space matrices
            A_continuous = [0, 1; -wn^2, -2 * xi_pole * wn];
            B_continuous = [0; wn^2];

            % Convert to discrete-time using the matrix exponential
            % A discrete = expm(A_continuous*Tc)
            obj.A = expm(A_continuous * Tc);
            % B discrete = integral(expm(A_continuous*Tc),0,Tc)*B
            % B discrete = A^(-1) * (expm(A_continuous*Tc)- eye(2))*B
            obj.B = A_continuous \ (expm(A_continuous * Tc) - eye(2)) * B_continuous;

            % transfer function
            %        [ x_1/u ]   [  wn^2/(s^2+2*xi_pole*wn*s+wn^2) ]
            % x/u =  [       ] = [                              ]
            %        [ x_2/u ]   [  s*wn^2/(s^2+2*xi_pole*wn*s+wn^2) ]
            %
            % y/u = (s^2*2*xi_zero*wn*s+wn^2)/(s^2+2*xci*wn*s+wn^2)
            %
            % y/u = 1 + 2*(xi_pole-xi_zero)*wn * s/(s^2+2*xci*wn*s+wn^2)
            %
            % y/u = 1 + 2*(xi_pole-xi_zero)/wn  x_2/u
            %
            % y = u + 2*(xi_pole-xi_zero)/wn  x_2

            % Set output and feedforward matrices
            obj.C = [0, 2 * (xi_pole - xi_zero) / wn];
            obj.D = 1;
        end

        % Method to initialize the state vector
        function obj = initialize(obj)
            obj.x = [0; 0];
        end

        % Method to set starting conditions based on the input
        function obj = starting(obj, input)
            % Validate input
            assert(isscalar(input), 'Input must be a scalar');

            % Set initial conditions based on input
            % y(0)=C*x(0) + D*input = input
            % y(1)=C*x(1) + D*input = input => C*A*x(0)+C*B*input=input
            %
            % C*x(0) = (1-D)*input 
            % C*A*x(0) = (1 -D -C*B)*input
            obj.x = [obj.C; obj.C * obj.A] \ [1 - obj.D; 1 - obj.C * obj.B - obj.D] * input;
        end

        % Method to perform one step of the notch filter and compute the output
        function output = step(obj, input)
            % Validate input
            assert(isscalar(input), 'Input must be a scalar');


            % Compute output and update state vector
            output = input - obj.C * obj.x;
            obj.x = obj.A * obj.x + obj.B * input;
        end
    end
end


