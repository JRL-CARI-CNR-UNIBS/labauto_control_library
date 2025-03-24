function [cin, ceq] = example_constraints(x)
%EXAMPLE_CONSTRAINTS Defines a nonlinear constraint for an optimization problem.
%
% Inputs:
%   x   - Input variable or variables for the constraint function.
%
% Outputs:
%   cin - Represents inequality constraints. The optimization algorithm 
%         attempts to ensure that cin <= 0.
%   ceq - Represents equality constraints (not used in this example).

% Define the inequality constraint based on the problem statement:
% We want sin(3*x) to be greater than 0.1, thus we define the constraint
% function as 0.1 - sin(3*x); this needs to be less than 0 for the 
% constraint to be satisfied.
cin = 0.1 - sin(3*x);

% Since there are no equality constraints, set ceq to an empty array.
ceq = [];
end
