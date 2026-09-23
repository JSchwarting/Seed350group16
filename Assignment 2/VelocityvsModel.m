%% CompareResults.m
% Runs the closed-loop Simulink model and overlays it against
% experimental data captured from the Arduino via ReadFromArduino.mlx

%% Define motor/control parameters
K = 1.375;       % DC gain [rad/Vs]
sigma = 14;      % time constant reciprocal [1/s]
Kp = 3;          % proportional gain

%% Run the simulation
open_system('motorsimcomplicated')
out = sim('motorsimcomplicated');

%% Load experimental data
load('test.mat')   % columns: time, desired velocity, voltage, velocity
t_exp   = data(:,1);
Vd_exp  = data(:,2);
V_exp   = data(:,3);
Vel_exp = data(:,4);

%% Plot: simulated vs experimental
figure
subplot(2,1,1)
plot(out.Voltage, '--', 'linewidth', 2); hold on
plot(t_exp, V_exp, 'linewidth', 2); hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)'); ylabel('Voltage (V)')
title('Voltage: Simulated vs Experimental')

subplot(2,1,2)
plot(out.Velocity, '--', 'linewidth', 2); hold on
plot(t_exp, Vel_exp, 'linewidth', 2); hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)')
title('Velocity: Simulated vs Experimental')