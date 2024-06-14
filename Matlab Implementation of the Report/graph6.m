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

% Solve the ODE using the ATSMC function % Adaptive Terminal Sliding Mode
% Control
[ta, xa] = ode45(@ATSMC, [0 10], [q10 q20 0 0 b00 b10 b20]);

figure(9);
plot(ta,xa(:,5), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('b0^');
title('b0^ vs Time(s)');
legend('b0^');

figure(10);
plot(ta,xa(:,6), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('b1^');
title('b1^ vs Time(s)');
legend('b1^');

figure(11);
plot(ta,xa(:,7), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('b2^');
title('b2^ vs Time(s)');
legend('b2^');