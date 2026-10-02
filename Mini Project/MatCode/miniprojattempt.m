K = 1.375;       % DC gain [rad/Vs]
sigma = 14;      % time constant reciprocal [1/s]
Kp = 3;          % proportional gain


open_system('PositionModel')
out = sim('PositionModel');

StepVal = pi;   % Step amplitude for the reference signal'


%% Plot Results
figure('Name', 'Motor Simulation Results', 'NumberTitle', 'off');

% Panel 1: Desired Position
subplot(4,1,1)
plot(out.DesiredPosition, 'LineWidth', 2, 'Color', [0 0.4470 0.7410])
grid on
title('Desired Position')
xlabel('Time (s)')
ylabel('Position (rad)')

% Panel 2: Desired Velocity
subplot(4,1,2)
plot(out.DesiredVelocity, 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980])
grid on
title('Desired Velocity')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')

% Panel 3: Controller Voltage
subplot(4,1,3)
plot(out.Voltage, 'LineWidth', 2, 'Color', [0.9290 0.6940 0.1250])
grid on
title('Control Input (Voltage)')
xlabel('Time (s)')
ylabel('Voltage (V)')

% Panel 4: Output Position
subplot(4,1,4)
plot(out.Position, 'LineWidth', 2, 'Color', [0.4660 0.6740 0.1880])
grid on
title('Actual Output Position')
xlabel('Time (s)')
ylabel('Position (rad)')