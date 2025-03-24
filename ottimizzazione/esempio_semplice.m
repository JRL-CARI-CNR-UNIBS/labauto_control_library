% Clear all variables, close all figures, and clear the command window
clear all; close all; clc;

% Define the fitness function for optimization
optim_fitness=@(x)0.02*x.^2-3*cos(5*x);

% Initial guess for the optimization
x0=-3;

% Define the lower and upper bounds for the optimization
lower_bound=-5;
upper_bound=5;

% Plot the fitness function within the specified bounds
fplot(optim_fitness,[lower_bound,upper_bound])
hold on

% Plot the initial guess on the graph
plot(x0,optim_fitness(x0),'or')

% Flags to control the flow of optimization
LOCAL_OPTIMIZATION=false;
VERY_SIMPLE_GRADIENT_DESCENT=false;

% Define any optimization constraints (empty for this example)
optim_constraints=[];

% Conditional block for local optimization
if LOCAL_OPTIMIZATION
    % Choose the optimization algorithm based on the flag
    if VERY_SIMPLE_GRADIENT_DESCENT
        % Perform optimization using a custom DummyGradientDescent function
        [x, fval, exitflag]=DummyGradientDescent(optim_fitness, x0, [], [], [], [], lower_bound, upper_bound, optim_constraints);
    else
        % Perform optimization using MATLAB's fmincon function
        [x, fval, exitflag] = fmincon(optim_fitness, x0, [], [], [], [], lower_bound, upper_bound, optim_constraints);
    end
    % Plot the optimization path
    quiver(x0,optim_fitness(x0),x-x0,optim_fitness(x)-optim_fitness(x0),0)
else
    % Perform global optimization using GlobalSearch
    gs = GlobalSearch('Display', 'iter','NumTrialPoints',5000);
    
    % Define the optimization problem
    problem = createOptimProblem('fmincon', 'x0', x0, 'objective', optim_fitness, 'lb', lower_bound, 'ub', upper_bound, 'nonlcon', optim_constraints);
    
    % Run the global optimization
    [x, fval, exitflag,output,solutions] = run(gs, problem);
    
    % Plot all successful trials
    for isol=1:length(solutions)
        for ix0=1:length(solutions(isol).X0)
            x0=solutions(isol).X0{ix0};
            if and(solutions(isol).Exitflag>0,and(lower_bound<x0,x0<upper_bound))
                quiver(x0,optim_fitness(x0),solutions(isol).X-x0,optim_fitness(solutions(isol).X)-optim_fitness(x0),0)
            end
        end
    end
end

% Check if optimization was successful
if exitflag <= 0
    error("Optimization failed: Check constraints or initial guess");
else
    fprintf('Optimization successful\n');
end

% Plot the optimized point
plot(x,optim_fitness(x),'ok')
