% IkMotion - Class for implementing inverse kinematics (IK) motion control
%
% This class provides a framework for inverse kinematics motion control.
% It is designed to determine joint positions, velocities, and
% accelerations based on Cartesian positions, velocities, and accelerations.
%
% Properties:
%   - Tc: Cycle time (sampling time)
%   - jacobFcn: Function handle for the Jacobian matrix computation
%   - jacobDotFcn: Function handle for the time derivative of the Jacobian matrix computation
%   - ikFcn: Function handle for the inverse kinematics computation
%   - q0: Initial joint positions
%   - last_q: Last joint positions (used for choosing closest IK solution)
%   - ndof: Number of degrees of freedom
%
% Methods:
%   - IkMotion: Constructor to create IkMotion objects
%   - initialize: Initialize method to set initial values
%   - ik: Method to compute joint positions, velocities, and accelerations from Cartesian motion
%
% Constructor Input Parameters:
%   - Tc: Cycle time (scalar numeric)
%   - q0: Initial joint positions (column vector)
%   - jacobFcn: Function handle for Jacobian matrix computation
%   - jacobDotFcn: Function handle for time derivative of Jacobian matrix computation
%   - ikFcn: Function handle for inverse kinematics computation
%
classdef IkMotion < handle
    properties
        Tc; % Cycle time (sampling time)
        jacobFcn; % Function handle for the Jacobian matrix computation
        jacobDotFcn; % Function handle for the time derivative of the Jacobian matrix computation
        ikFcn; % Function handle for the inverse kinematics computation
        q0; % Initial joint positions
        last_q; % Last joint positions (used for choosing closest IK solution)

        ndof; % Number of degrees of freedom
    end
    methods  (Access = public)

        % Constructor for IkMotion class
        function obj=IkMotion(Tc, q0, jacobFcn, jacobDotFcn, ikFcn)
            
            % Input validation for initial joint positions (q0)
            assert(isvector(q0));
            assert(iscolumn(q0));
            assert(isnumeric(q0));
            
            % Input validation for cycle time (Tc)
            assert(isnumeric(Tc));
            assert(isscalar(Tc));
            obj.Tc = Tc;

            % Set properties from input parameters
            obj.jacobFcn = jacobFcn;
            obj.jacobDotFcn = jacobDotFcn;
            obj.ikFcn = ikFcn;

            obj.ndof = length(q0);
            obj.q0 = q0;
            
            % Initialize object
            initialize(obj);
        end

        % Initialize method to set initial values
        function obj=initialize(obj)
            obj.last_q = obj.q0;
        end

        % Method to compute joint positions, velocities, and accelerations from Cartesian motion
        function [q, Dq, DDq]=ik(obj, cart, Dcart, DDcart)
            % Input validation for Cartesian motion
            assert(isvector(cart));
            assert(iscolumn(cart));
            assert(isnumeric(cart));
            assert(length(cart) == obj.ndof);

            % Input validation for Cartesian velocity
            assert(isvector(Dcart));
            assert(iscolumn(Dcart));
            assert(isnumeric(Dcart));
            assert(length(Dcart) == obj.ndof);

            % Input validation for Cartesian acceleration
            assert(isvector(DDcart));
            assert(iscolumn(DDcart));
            assert(isnumeric(DDcart));
            assert(length(DDcart) == obj.ndof);

            % Compute IK solutions for the given Cartesian position
            iksols = obj.ikFcn(cart(1), cart(2));

            % Choose the closest solution based on Euclidean distance
            dist = inf;
            if any(isnan(iksols(:)))
                error("There are not ik solutions")
            end

            

            for solution = 1:size(iksols, 2)
                if not(all(isreal(iksols(:, solution))))
                    continue;
                end
                dist_solution = norm(iksols(:, solution) - obj.last_q);
                if (dist_solution < dist)
                    dist = dist_solution;
                    iksol = iksols(:, solution);
                end
            end
            if (dist==inf)
                error("There are not ik solutions")
            end

            % Update last joint positions
            obj.last_q = iksol;

            % Output joint positions
            q = iksol;

            % Compute Jacobian matrix for the current joint positions
            jacobian = obj.jacobFcn(obj.last_q(1), obj.last_q(2));
            jacobian=jacobian([1 3],:); % use only two dimensions

            % Compute joint velocities using the Jacobian inverse
            Dq = jacobian \ Dcart;

            % Compute time derivative of Jacobian matrix
            jacobianDot = obj.jacobDotFcn(obj.last_q(1), obj.last_q(2), Dq(1), Dq(2));


            % Compute joint accelerations using the Jacobian inverse
            DDq = jacobian \ (DDcart-jacobianDot([1 3]));
        end

    end  
end
