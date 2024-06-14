clc;
clear all;
close all;

% Initial conditions for adaptation laws and system state
b00 = 5;
b10 = 14;
b20 = 2;
q10 = 0.8;
q20 = 0.9;
%% Adaptive Terminal Sliding Mode 
% Solve the ODE using the ATSMC function % Adaptive Terminal Slidin Mode
% Control
[ta, xa] = ode45(@ATSMC, [0 10], [q10 q20 0 0 b00 b10 b20]);

% Desired angular positions
qd1a = 1.25 - 7/5 * exp(-ta) + 7/20 * exp(-4 * ta);
qd2a = 1.4 - 7/5 * exp(-ta) + 7/20 * exp(-4 * ta);

% Calculate errors in position
e1a = xa(:, 1) - qd1a;
e2a = xa(:, 2) - qd2a;

% Calculate derivatives of 'e'
de1_dta = diff(e1a) ./ diff(ta);
de2_dta = diff(e2a) ./ diff(ta);
%% Terminal Sliding Mode Control
% Solve the ODE using the CTSMC function %Terminal Sliding Mode control 
[tc, xc] = ode45(@CTSMC, [0 10], [q10 q20 0 0]);

% Desired angular positions
qd1c = 1.25 - 7/5 * exp(-tc) + 7/20 * exp(-4 * tc);
qd2c = 1.4 - 7/5 * exp(-tc) + 7/20 * exp(-4 * tc);

% Calculate errors in position
e1c = xc(:, 1) - qd1c;
e2c = xc(:, 2) - qd2c;

% Calculate derivatives of 'e'
de1_dtc = diff(e1c) ./ diff(tc);
de2_dtc = diff(e2c) ./ diff(tc);
%%
% Create a plot of de1/dt vs e1 for joint 1
figure(4);
hold on;
plot(e1a(1:end-1), de1_dta, 'LineWidth', 1.5);
plot(e1c(1:end-1), de1_dtc, '--', 'LineWidth', 1.5);  % Dotted line
hold off;
xlabel('e1 (rad)');
ylabel('de1/dt (rad/s)');   
title('de1/dt (rad/s) vs e1 (rad)');
legend('Joint 1(Adaptive Terminal)','Joint 1(Terminal)');

% Create a plot of de2/dt vs e2 for joint 2
figure(8);
hold on;
plot(e2a(1:end-1), de2_dta, 'LineWidth', 1.5);
plot(e2c(1:end-1), de2_dtc, '--', 'LineWidth', 1.5);  % Dotted line
hold off;
xlabel('e2 (rad)');
ylabel('de2/dt (rad/s)');
title('de2/dt (rad/s) vs e2 (rad)');
legend('Joint 2(Adaptive Terminal)','Joint 2(Terminal)');
