function [cin, ceq] = constraints(controller_parameters, ...
    controller_frequency_response, ...
    process_frequency_response, ...
    desired_maximum_sensitivy, ...
    desired_phase_margin, ...
    low_freq_disturbance_attenuation, ...
    low_freq_threshold, ...
    high_freq_noise_attenuation, ...
    high_freq_threshold, ...
    frequency_vector)

% This function computes constraint values for optimization

% Input Arguments:
%   - controller_parameters: Parameters of the controller.
%   - controller_frequency_response: Function describing the frequency response of the controller.
%   - process_frequency_response: Function describing the frequency response of the process.
%   - desired_maximum_sensitivity: Desired maximum sensitivity (robustness).
%   - desired_phase_margin: Desired phase margin.
%   - low_freq_disturbance_attenuation: Desired low-frequency disturbance attenuation.
%   - low_freq_threshold: Low-frequency threshold for disturbance attenuation.
%   - high_freq_noise_attenuation: Desired high-frequency noise attenuation.
%   - high_freq_threshold: High-frequency threshold for noise attenuation.
%   - frequency_vector: Vector of frequencies for evaluation.

% Compute the controller based on its frequency response and parameters
controller = controller_frequency_response(controller_parameters);

% Define the open-loop transfer function L(w) = G(w) * C(w)
L = @(w) (process_frequency_response(w) .* controller(w));

% Determine the crossover frequency wc (frequency where |L(wc)| = 1)
wc=find_cutting_frequency(L,frequency_vector);
if not(isempty(wc))
    % add new frequency to frequency_vector
    frequency_vector=sort([frequency_vector;wc]);
    i_wc=find(frequency_vector==wc,1);

    % Compute phase of L(w) on the frequency vector
    L_phase = unwrap(angle(L(frequency_vector)));

    % Correct phase at wc to ensure it is continuous
    phase_wc = L_phase(i_wc);

    % Calculate actual phase margin in degrees
    actual_phase_margin = rad2deg(pi + phase_wc);
else
    actual_phase_margin=inf;
end

% Define functions for disturbance, sensitivity, and noise attenuation
disturbance_attenuation = @(w) abs(process_frequency_response(w) ./ (1 + L(w)));
sensitivity = @(w) abs(1 ./ (1 + L(w)));
noise_attenuation = @(w) abs(L(w) ./ (1 + L(w)));

% Find the index of the frequency closest to the low-frequency threshold
i_wl = find(frequency_vector <= low_freq_threshold, 1, 'last');

% Compute worst-case disturbance attenuation up to the low-frequency threshold
disturbance_attenuation_worst_case = max(disturbance_attenuation(frequency_vector(1:i_wl)));

% Find the index of the frequency closest to the high-frequency threshold
i_wh = find(frequency_vector >= high_freq_threshold, 1, 'first');

% Compute worst-case noise attenuation beyond the high-frequency threshold
noise_attenuation_worst_case = max(noise_attenuation(frequency_vector(i_wh:end)));

% Compute actual maximum sensitivity
actual_max_sensitivity = max(sensitivity(frequency_vector));

% Constraint equations
cin(1) = disturbance_attenuation_worst_case - low_freq_disturbance_attenuation; % Low-frequency disturbance attenuation constraint
cin(2) = noise_attenuation_worst_case - high_freq_noise_attenuation; % High-frequency noise attenuation constraint
cin(3) = actual_max_sensitivity - desired_maximum_sensitivy; % Maximum sensitivity (robustness) constraint
cin(4) = desired_phase_margin - actual_phase_margin; % Phase margin constraint

% Check for NaN values in constraints
errflag = 0;
if any(isnan(cin))
    errflag = 1;
    indnan = find(isnan(cin));
    for index = 1:length(indnan)
        fprintf('[Constraints] Element %d of cin is NaN!\n', indnan(index));
    end
end
ceq=[];

% If NaN values are detected, issue a warning and replace with large values
if errflag == 1
    warning('Some elements of cin and/or ceq are equal to NaN');
    cin = 1e5 * ones(size(cin));
end
