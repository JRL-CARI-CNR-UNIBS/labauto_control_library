% PIDController - Proportional-Integral-Derivative (PID) Controller implementation
%
% This class implements a PID controller, a widely used type of feedback
% controller in control systems. The PID controller consists of three terms:
% Proportional (P), Integral (I), and Derivative (D). The control action
% is computed as u = Kp*e + xi + Kd*De, where e is the error, xi is the
% integral term, and De is the derivative term. The controller also allows
% the use of filters on the error signal, measurement, and derivative of the
% error signal.
%
% Properties (Access = protected):
%   - integral_value: Integral term value
%   - proportional_gain: Proportional gain (Kp)
%   - integral_gain: Integral gain (Ki)
%   - derivative_gain: Derivative gain (Kd)
%   - filters_on_error_signal: Filters applied to the error signal
%   - filters_on_measure: Filters applied to the measurement signal
%   - filters_on_derivative_error: Filters applied to the derivative of the error signal
%   - filtered_error_for_derivative: Filtered error for derivative computation
%
% Methods (Access = public):
%   - PIDController: Constructor to create PIDController objects
%   - initialize: Method to initialize the state variables of the controller
%   - starting: Method to set the starting conditions based on the input
%   - computeControlAction: Method to compute the control action based on the error and reference
%
% Constructor Input Parameters:
%   - st: Sampling time (scalar numeric, st > 0)
%   - Kp: Proportional gain (scalar numeric, Kp >= 0)
%   - Ki: Integral gain (scalar numeric, Ki >= 0)
%   - Kd: Derivative gain (scalar numeric, Kd >= 0)
%   - filters_on_derivative_error: Filters applied to the derivative of the error signal (optional)
%   - filters_on_error_signal: Filters applied to the error signal (optional)
%   - filters_on_measure: Filters applied to the measurement signal (optional)
%
%                  +-------+    +-------+
%        /---\ e   |Filter |    |       | u
% ref -->| + |---->|       +--->|  PID  +--->
%        \---/     |on e   |    |       |
%          ^       +-------+    +-------+
%          |
%      +---+----+
%      |Filter  |
%      |        |
%      |on y    |
%      +--------+
%          ^
%          |
%        measure

classdef PIDController < BaseController 
    properties  (Access = protected)
        integral_value % Integral term value
        proportional_gain % Proportional gain (Kp)
        integral_gain % Integral gain (Ki)
        derivative_gain % Derivative gain (Kd)
        filters_on_error_signal % Filters applied to the error signal
        filters_on_measure % Filters applied to the measurement signal
        filters_on_derivative_error % Filters applied to the derivative of the error signal
        filtered_error_for_derivative % Filtered error for derivative computation
    end
    methods
        % Constructor for PIDController class
        function obj=PIDController(st,Kp,Ki,Kd,filters_on_derivative_error,filters_on_error_signal,filters_on_measure)
            % Validate input parameters
            assert(isscalar(Kp) && Kp >= 0, 'Proportional gain (Kp) must be a non-negative scalar');
            assert(isscalar(Ki) && Ki >= 0, 'Integral gain (Ki) must be a non-negative scalar');
            assert(isscalar(Kd) && Kd >= 0, 'Derivative gain (Kd) must be a non-negative scalar');
            assert(isscalar(st) && st > 0, 'Sampling time (st) must be a positive scalar');
            
            if not(isempty(filters_on_derivative_error))
                assert(isa(filters_on_error_signal,'BaseFilter'), 'filters_on_error_signal must be an instance of BaseFilter');
            end
            if not(isempty(filters_on_derivative_error))
                assert(isa(filters_on_derivative_error,'BaseFilter'), 'filters_on_derivative_error must be an instance of BaseFilter');
            end
            if not(isempty(filters_on_measure))
                assert(isa(filters_on_measure,'BaseFilter'), 'filters_on_measure must be an instance of BaseFilter');
            end

            % Call the constructor of the superclass (BaseController)
            obj@BaseController(st);

            % Initialize properties
            obj.integral_value = 0;
            obj.proportional_gain = Kp;
            obj.integral_gain = Ki;
            obj.derivative_gain = Kd;
            
            obj.filtered_error_for_derivative = 0;
            obj.filters_on_error_signal = filters_on_error_signal;
            obj.filters_on_measure = filters_on_measure;
            obj.filters_on_derivative_error = filters_on_derivative_error;
        end
        
        % Method to initialize the state variables
        function obj=initialize(obj)
            obj.integral_value = 0;
            obj.filtered_error_for_derivative = 0;
        end

        % Method to set starting conditions based on the input
        function obj=starting(obj,reference,y,u,uff)
            % Validate input
            assert(isscalar(reference) && isscalar(y) && isscalar(u), 'Reference, y, and u must be scalars');
            
            % Compute error signal
            e = reference - y;
            
            % Apply filters to the measurement signal if available
            if ~isempty(obj.filters_on_measure)
                obj.filters_on_measure.starting(y);
            end

            obj.filtered_error_for_derivative = e;
            
            % Apply filters to the error signal
            for idx = 1:length(obj.filters_on_error_signal)
                obj.filters_on_error_signal(idx).starting(e);
            end
            
            % Set initial integral value
            % u = integral_value + Kp*e + uff 
            % integral_value = u - Kp*e - uff;
            obj.integral_value = u - obj.proportional_gain * e - uff;
        end

        % Method to compute the control action
        function u=computeControlAction(obj,reference,y,uff)
            % Validate input
            assert(isscalar(reference) && isscalar(y), 'Reference and y must be scalars');
            
            % Apply filters to the measurement signal if available
            if ~isempty(obj.filters_on_measure)
                for idx = 1:length(obj.filters_on_error_signal)
                    filter_measure = obj.filters_on_measure(idx).step(y);
                end
            else
                filter_measure = y;
            end
            
            % Compute error signal
            error_signal = reference - filter_measure;
            
            % Apply filters to the error signal
            for idx = 1:length(obj.filters_on_error_signal)
                error_signal = obj.filters_on_error_signal(idx).step(error_signal);
            end

            % Compute derivative
            if obj.derivative_gain ~= 0
                last_filtered_error = obj.filtered_error_for_derivative;
                obj.filtered_error_for_derivative = obj.filters_on_derivative_error.step(error_signal);
                De = (obj.filtered_error_for_derivative - last_filtered_error) / obj.Tc;
                ud = obj.derivative_gain * De;
            else
                ud = 0;
            end
            
            % Compute control action
            u = obj.integral_value + obj.proportional_gain * error_signal + ud + uff;

            % Constrain the control action within limits
            if u > obj.umax
                u = obj.umax;
                % Conditionally integrate based on the error signal
                if error_signal < 0
                    obj.integral_value = obj.integral_value + obj.integral_gain * obj.Tc * error_signal;
                end
            elseif u < -obj.umax
                u = -obj.umax;
                % Conditionally integrate based on the error signal
                if error_signal > 0
                    obj.integral_value = obj.integral_value + obj.integral_gain * obj.Tc * error_signal;
                end
            else
                % Integrate the error signal
                obj.integral_value = obj.integral_value + obj.integral_gain * obj.Tc * 0.5*error_signal;
                %obj.integral_value = obj.integral_value + obj.integral_gain * obj.Tc * 0.5*(error_signal+obj.previous_error);
            end
        end
    end
end
