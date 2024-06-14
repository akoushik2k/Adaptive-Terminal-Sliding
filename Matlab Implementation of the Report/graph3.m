clc;
clear all;
close all;

%% Adaptive Sliding Mode Control
% Initial conditions for adaptation laws and system state
b00 = 5;
b10 = 14;
b20 = 2;
q10 = 0.8;
q20 = 0.9;

% Solve the ODE using the ATSMC function % Adaptive Terminal Slidin Mode
% Control
[ta, xa] = ode45(@ATSMC, [0 10], [q10 q20 0 0 b00 b10 b20]);

% Desired angular positions and qddot
qd1a = 1.25 - 7/5 * exp(-ta) + 7/20 * exp(-4 * ta);
qd2a = 1.4 - 7/5 * exp(-ta) + 7/20 * exp(-4 * ta);
qddot1a = 7/5 * exp(-ta) - 7/5 * exp(-4 * ta);

% Calculate errors in position and velocity
e11a = xa(:, 1) - qd1a;
e12a = xa(:, 2) - qd2a;
e21a = xa(:, 3) - qddot1a;
e22a = xa(:, 4) - qddot1a;

% Surface Parameters
a = 5;
b = 7;
c = diag([2, 2]);

% Calculate the surface equations
S1a = e21a + c(1, 1) * e11a.^(a / b) + c(1, 2) * e12a.^(a / b);
S2a = e22a + c(2, 1) * e11a.^(a / b) + c(2, 2) * e12a.^(a / b);
%% Terminal Sliding Mode Control
% Solve the ODE using the CTSMC function %Terminal Sliding Mode control 
[tc, xc] = ode45(@CTSMC, [0 10], [q10 q20 0 0]);

% Desired angular positions and qddot
qd1c = 1.25 - 7/5 * exp(-tc) + 7/20 * exp(-4 * tc);
qd2c = 1.4 - 7/5 * exp(-tc) + 7/20 * exp(-4 * tc);
qddot1c = 7/5 * exp(-tc) - 7/5 * exp(-4 * tc);

% Calculate errors in position and velocity
e11c = xc(:, 1) - qd1c;
e12c = xc(:, 2) - qd2c;
e21c = xc(:, 3) - qddot1c;
e22c = xc(:, 4) - qddot1c;

% Calculate the surface equations
S1c = e21c + c(1, 1) * e11c.^(a / b) + c(1, 2) * e12c.^(a / b);
S2c = e22c + c(2, 1) * e11c.^(a / b) + c(2, 2) * e12c.^(a / b);
%%
% Plot the surface S1
figure(3);
hold on;
plot(ta, S1a, 'LineWidth', 1.5);
plot(tc, S1c, '--', 'LineWidth', 1.5);  % Dotted line
hold off;
xlabel('Time (s)');
ylabel('S1');
title('Surface S1 vs Time(s)');
legend('S1(Adaptive Terminal)','S1(Terminal)');

% Plot the surface S2
figure(7);
hold on;
plot(ta, S2a, 'LineWidth', 1.5);
plot(tc, S2c, '--', 'LineWidth', 1.5);  % Dotted line
hold off;
xlabel('Time (s)');
ylabel('S2');
title('Surface S2 vs Time(s)');
legend('S2(Adaptive Terminal)','S2(Terminal)');
