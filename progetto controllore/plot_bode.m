function plot_bode(frequency_vector, frequency_response, varargin)
% Plots the Bode diagram for a given frequency response vector.

% Input Arguments:
%   - frequency_vector: Vector of frequencies.
%   - frequency_response: Vector of complex frequency response values.
%   - varargin: Additional optional arguments for customizing the plot (e.g., plot labels).

% Parse optional arguments
p = inputParser;
addOptional(p, 'LineWidth', 1, @isnumeric);  % Line width for the plot
addOptional(p, 'Color', 'b');  % Line color for the plot
addOptional(p, 'FrequencyUnit', 'rad/s', @ischar);  % Frequency unit for x-axis (rad/s or Hz)
parse(p, varargin{:});

% Extract values from the parsed results
lineWidth = p.Results.LineWidth;
lineColor = p.Results.Color;
frequencyUnit = p.Results.FrequencyUnit;

numerical_frequency_response = frequency_response(frequency_vector);
% Create Bode plot
subplot(2,1,1)
semilogx(frequency_vector, 20 * log10(abs(numerical_frequency_response)), 'LineWidth', lineWidth, 'Color', lineColor);
grid on
box on
hold on
xlabel(['Frequency (' frequencyUnit ')']);
ylabel('Magnitude (dB)');
xlim([min(frequency_vector) max(frequency_vector)]);
title('Bode Diagram');

subplot(2,1,2)
semilogx(frequency_vector, unwrap(angle(numerical_frequency_response)) * (180/pi), 'LineWidth', lineWidth, 'Color', lineColor);
grid on
box on
hold on
xlabel(['Frequency (' frequencyUnit ')']);
ylabel('Phase (degrees)');
xlim([min(frequency_vector) max(frequency_vector)]);
limy=ylim;
limy(1)=max(-540,limy(1));
ylim(limy);

end
