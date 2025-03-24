function wc = find_cutting_frequency(frequency_response, frequency_vector)
% This function determines the cutting frequency (crossover frequency) of a system's frequency response.

% Input Arguments:
%   - frequency_response: Function describing the frequency response of the system.
%   - frequency_vector: Vector of frequencies for evaluation.

% Compute the magnitude of the frequency response at each frequency
magnitude = abs(frequency_response(frequency_vector));

% Determine the crossover frequency wc (frequency where |L(wc)| = 1)
if ~isempty(find(magnitude > 1, 1)) && ~isempty(find(magnitude < 1, 1))
    % 0dB crossing exists

    % Find the index of the first frequency where magnitude is less than 1
    i_wc = find(magnitude < 1, 1, 'first');

    % Check if the index is empty (should not happen)
    if isempty(i_wc)
        error("This should not happen");
    end

    % Obtain the crossover frequency
    wc = frequency_vector(i_wc);
else
    % No 0dB crossing

    if abs(magnitude(1)) < 1
        % Frequency response magnitude at the first frequency is less than 1

        % Initialize a small value for wc and check if L(s) crosses 0dB
        wc = 1e-80;
        
        % Check if the magnitude at wc is less than 1, issue a warning
        if abs(frequency_response(wc)) < 1
            warning("L(s) does not cut the 0dB");
            wc = [];
            return
        end

        % Increase wc until it crosses 0dB
        while abs(frequency_response(wc)) > 1
            wc = wc * 10;
        end
    else
        % Frequency response magnitude at the last frequency is greater than 1

        % Initialize a large value for wc and check if L(s) is strictly proper
        wc = 1e80;
        
        % Check if the magnitude at wc is greater than 1, issue a warning
        if abs(frequency_response(wc)) > 1
            warning("L(s) is not strictly proper");
            wc = [];
            return
        end

        % Decrease wc until it crosses 0dB
        while abs(frequency_response(wc)) < 1
            wc = wc / 10;
        end
    end
end

% Refine the crossover frequency using fzero to find the exact 0dB crossing
wc = fzero(@(w) abs(frequency_response(w)) - 1, wc);

% If wc is negative, refine again using the positive counterpart
if wc < 0
    wc = fzero(@(w) abs(frequency_response(w)) - 1, -wc);
end
