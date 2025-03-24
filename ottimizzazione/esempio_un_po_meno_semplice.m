% Clear all variables, close all figures, and clear the command window
clear all; close all; clc;

% Define the fitness function for optimization
optim_fitness = @(x) 0.02*x.^2 - 3*cos(5*x);

% Define the nonlinear constraint function
optim_constraints = @example_constraints; % Adds nonlinear constraint sin(3*x) > 0.1 as an example

% Initial guess for the optimization
x0 = -4;

% Define the lower and upper bounds for the optimization
lower_bound = -5;
upper_bound = 5;

% Generate a range of values within the bounds for plotting
xrange = linspace(lower_bound, upper_bound, 1e3)';
f = optim_fitness(xrange);

% Evaluate constraints over the range
[cin, ceq] = optim_constraints(xrange);

% Plot the fitness function as a dashed black line
plot(xrange, f, '--k');

% Invalidates fitness values where constraints are not met
fvalid = f;
fvalid(cin >= 0) = nan;

% Hold the plot for overlaying more data
hold on

% Plot valid fitness function regions as a solid blue line
plot(xrange, fvalid, '-b');

% Plot the initial guess on the graph
plot(x0, optim_fitness(x0), 'or');

% Control flags for the optimization process
LOCAL_OPTIMIZATION = false;
VERY_SIMPLE_GRADIENT_DESCENT = false;

% Conditional block for local optimization
if LOCAL_OPTIMIZATION
    if VERY_SIMPLE_GRADIENT_DESCENT
        % Use a custom DummyGradientDescent function for optimization
        [x, fval, exitflag] = DummyGradientDescent(optim_fitness, x0, [], [], [], [], lower_bound, upper_bound, optim_constraints);
    else
        % Use fmincon for constrained optimization
        [x, fval, exitflag] = fmincon(optim_fitness, x0, [], [], [], [], lower_bound, upper_bound, optim_constraints);
    end
    % Plot the optimization path
    quiver(x0, optim_fitness(x0), x-x0, optim_fitness(x)-optim_fitness(x0), 0);
else
    % Create a GlobalSearch object with iterative display
    gs = GlobalSearch('Display', 'iter', 'NumTrialPoints', 5000);

    % Define the optimization problem
    problem = createOptimProblem('fmincon', 'x0', x0, 'objective', optim_fitness, 'lb', lower_bound, 'ub', upper_bound, 'nonlcon', optim_constraints);

    % Run the global optimization
    [x, fval, exitflag, output, solutions] = run(gs, problem);

    % Plot all successful attempts
    for isol = 1:length(solutions)
        for ix0 = 1:length(solutions(isol).X0)
            x0 = solutions(isol).X0{ix0};
            if and(solutions(isol).Exitflag > 0, and(lower_bound < x0, x0 < upper_bound))
                quiver(x0, optim_fitness(x0), solutions(isol).X-x0, optim_fitness(solutions(isol).X)-optim_fitness(x0), 0);
            end
        end
    end
end

% Check if the optimization was successful
if exitflag <= 0
    error("Optimization failed: Check constraints or initial guess");
else
    fprintf('Optimization successful\n');
end

% Plot the optimized point
plot(x, optim_fitness(x), 'ok');
