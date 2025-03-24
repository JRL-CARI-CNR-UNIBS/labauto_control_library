clear all; close all;clc

LOCAL_OPTIMIZATION=true;
controller_type='PI';

switch controller_type
    case 'PI'
        % Function handle for the frequency response of the controller
        controller_frequency_response = @pi_frequency_response;

        % Initial guess for the controller parameters
        controller_parameters_seed = [60 3];

        % Upper and lower bounds for the controller parameters during optimization
        controller_parameters_upper_bound = [100 5];
        controller_parameters_lower_bound = [0.01 0.2];
end

% Process parameters and its derivative with respect to the optimization parameter
s=tf('s');
Plant=1/(s+1)*exp(-2*s);

% Function handle for the process frequency response
process_frequency_response = @(w)reshape( freqresp(Plant,w), length(w),1);

% Loop shaping condition parameters
desired_wc = 1;
desired_phase_margin = 60;
desired_maximum_sensitivity = 2;
low_freq_disturbance_attenuation = 0.1;
low_freq_threshold = 0.0001;
high_freq_noise_attenuation = 0.1;
high_freq_threshold = 10000;

% Frequency vector for evaluation (logarithmic scale)
frequency_vector = logspace(-5, 5, 100000)';
frequency_vector = sort([frequency_vector; low_freq_threshold; high_freq_threshold]);

% Define optimization constraints using the constraints function
optim_constraints = @(controller_parameters) constraints(controller_parameters, ...
    controller_frequency_response, ...
    process_frequency_response, ...
    desired_maximum_sensitivity, ...
    desired_phase_margin, ...
    low_freq_disturbance_attenuation, ...
    low_freq_threshold, ...
    high_freq_noise_attenuation, ...
    high_freq_threshold, ...
    frequency_vector);

% Define optimization fitness function using the fitness_wc function
optim_fitness = @(controller_parameters) fitness(controller_parameters, ...
    controller_frequency_response, ...
    process_frequency_response, ...
    desired_wc, ...
    frequency_vector);

% Perform optimization using fmincon
tic
% Set options for the optimization algorithm

if LOCAL_OPTIMIZATION
    options = optimset('Display', 'final', 'TolX', 1e-6);
    [optimal_parameters, optimal_wc] = fmincon(optim_fitness, controller_parameters_seed, ...
        [], [], [], [], controller_parameters_lower_bound, controller_parameters_upper_bound, ...
        optim_constraints, ...
        options);
else
    optimal_parameters = ga(optim_fitness,length(controller_parameters_seed),[],[],[],[], controller_parameters_lower_bound, controller_parameters_upper_bound, ...
        optim_constraints)
end
computation_time = toc;

c=controller_frequency_response(optimal_parameters);

%%
L=@(w)c(w).*(process_frequency_response(w));
plot_bode(frequency_vector,L)

