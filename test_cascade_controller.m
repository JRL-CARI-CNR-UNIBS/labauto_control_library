% Clear workspace, close figures, and clear command window
clear all;
close all;
clc;

% Sampling time
Tc = 0.01;

% Time vector from 0 to 3 seconds with a sampling interval of Tc
t = (0:Tc:3)';

% Define a test system
s = tf('s');
Pv = 1 / (0.2 * s + 1);
system = [1/s; 1] * ss(Pv);
system.OutputName = {'Position'; 'Velocity'};
system.InputName = {'torque'};
system_d = c2d(system, Tc);
[A, B, C, D] = ssdata(system_d);
x0 = zeros(size(A, 1), 1);

% Define inner controller parameters
Kpv = 10;
Tiv = 1.0;
Kiv = Kpv / Tiv;
Tdv = 0.01;
Kdv = Kpv * Tdv;
Tfv = Tdv / 5;

wn = 10;
xi_zero = 0.1;
xi_pole = 0.7;
notch = (s^2 + 2 * xi_zero * wn * s + wn^2) / (s^2 + 2 * xi_pole * wn * s + wn^2);
notch_filter = NotchFilter(Tc, wn, xi_zero, xi_pole);

der_filt = 1 / (Tfv * s + 1);
der_filter = FirstOrderLowPassFilter(Tc, Tfv);

inner_ctrl_transfer_function = (Kpv + Kiv / s + Kdv * s * der_filt) * notch;

% Compute inner closed-loop transfer function
Fv = feedback(inner_ctrl_transfer_function * Pv, 1);

% Compute transfer function from target velocity to position
Pp = Fv / s;

% Define outer controller parameters
Kpp = 5;
Kip = 0;
Kdp = 0;
outer_ctrl_trasnfer_function = Kpp;

% Compute outer closed-loop transfer function
Fp = feedback(outer_ctrl_trasnfer_function * Pp, 1);

% Compute expected output
y_continuous = step(Fp, t);

% Define control objects
inner_ctrl = PIDController(Tc, Kpv, Kiv, Kdv, der_filter, notch_filter, []);
outer_ctrl = PIDController(Tc, Kpp, Kip, Kdp, [], [], []);
cascade_ctrl = CascadeController(Tc, inner_ctrl, outer_ctrl);

% Initialize controllers starting from zero
cascade_ctrl.starting([0; 0], [0; 0], 0, 0);
x = x0;

reference = [1; 0];
u = 0;
for idx = 1:length(t)
    y = C * x + D * u;
    u = cascade_ctrl.computeControlAction(reference, y, 0);
    x = A * x + B * u;

    control_action(idx, :) = u;
    measures(idx, :) = y;
end

% Plot results
figure
subplot(3, 1, 1)
stairs(t, measures(:, 1))
hold
plot(t, y_continuous)
grid on
xlabel('time')
ylabel('position')
legend('implementation','continuous')

subplot(3, 1, 2)
stairs(t, measures(:, 2))
grid on
xlabel('time')
ylabel('velocity')

subplot(3, 1, 3)
stairs(t, control_action)
grid on
xlabel('time')
ylabel('torque')
