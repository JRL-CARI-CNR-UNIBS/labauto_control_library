function J=fitness(controller_parameters, ...
    controller_frequency_response, ...
    process_frequency_response,desired_wc,frequency_vector)

% controller_parameters
% process process
% wc desired cutting frequency
% w frequency vector


% Compute the controller based on its frequency response and parameters
controller = controller_frequency_response(controller_parameters);

% Define the open-loop transfer function L(w) = G(w) * C(w)
L = @(w) (process_frequency_response(w) .* controller(w));

actual_wc=find_cutting_frequency(L,frequency_vector);
if not(isempty(actual_wc))
    J=(actual_wc-desired_wc)^2;
    %J=-actual_wc/desired_wc;
else
    J=1000000;
end

