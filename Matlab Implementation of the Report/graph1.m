clc;
clear all;
close all;

% Initial conditions for adaptation laws
b00 = 5;
b10 = 14;
b20 = 2;

% Initial conditions for the system state
q10 = 0.8;
q20 = 0.9;

% Solve the ODE using the ATSMC function % Adaptive Terminal Slidin Mode
% Control
[ta, xa] = ode45(@ATSMC, [0 10], [q10 q20 0 0 b00 b10 b20]);

% Solve the ODE using the CTSMC function %Terminal Sliding Mode control 
[tc, xc] = ode45(@CTSMC, [0 10], [q10 q20 0 0]);


% Desired angular positions for comparison wrt to Adaptive control
qd1a = 1.25 - 7/5 * exp(-ta) + 7/20 * exp(-4 * ta);
qd2a = 1.4 - 7/5 * exp(-ta) + 7/20 * exp(-4 * ta);

% Desired angular positions for comparison wrt to Terminal Control
qd1c = 1.25 - 7/5 * exp(-tc) + 7/20 * exp(-4 * tc);
qd2c = 1.4 - 7/5 * exp(-tc) + 7/20 * exp(-4 * tc);

% Plot the error in the first joint position
figure(1);
hold on;
plot(ta, xa(:, 1) - qd1a, 'LineWidth', 1.5);
plot(tc, xc(:, 1) - qd1c, '--', 'LineWidth', 1.5);  % Dotted line
hold off;
xlabel('Time (s)');
ylabel('e1 (rad)');
title('Error in e1 (rad) vs Time(s)');
legend('e1 error (Adaptive Terminal)', 'e1 error (Terminal)');

% Plot the error in the second joint position
figure(2);
hold on;
plot(ta, xa(:, 2) - qd2a, 'LineWidth', 1.5);
plot(tc, xc(:, 2) - qd2c, '--', 'LineWidth', 1.5);  % Dotted line
hold off;
xlabel('Time (s)');
ylabel('e2 (rad)');
title('Error in e2 (rad) vs Time(s)');
legend('e2 error (Adaptive Terminal)', 'e2 error (Terminal)');
