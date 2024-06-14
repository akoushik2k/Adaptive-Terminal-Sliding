clc;
clear all;
close all;

% Initial conditions for adaptation laws and system state
b00 = 5;
b10 = 14;
b20 = 2;
q10 = 0.8;
q20 = 0.9;

% Solve the ODE using the ATSMC function % Adaptive Terminal Slidin Mode
% Control
[ta, xa] = ode45(@ATSMC, [0 10], [q10 q20 0 0 b00 b10 b20]);

% Solve the ODE using the CTSMC function %Terminal Sliding Mode control 
[tc, xc] = ode45(@CTSMC, [0 10], [q10 q20 0 0]);

% Initialize the control input matrix U
Ua = zeros(length(ta), 2);
Uc = zeros(length(tc), 2);

% for loop to find the torque using Adaptive Terminal Sliding Mode Control
for i = 1:length(ta)
    % Given Nominal values of m1 and m2 (assumed)
    m10 = 0.4;
    m20 = 1.2;

    % Other System parameters
    L1 = 1; % Units meters
    L2 = 0.8; % Units meters
    J1 = 5; % Units kg.m
    J2 = 5; % Units kg

    % Inertia Components with Nominal Values
    M110 = (m10 + m20) * L1^2 + m20 * L2^2 + 2 * m20 * L1 * L2 * cos(xa(i, 2)) + J1;
    M120 = m20 * L2^2 + m20 * L1 * L2 * cos(xa(i, 2));
    M220 = m20 * L2^2 + J2;

    % Centrifugal Components with Nominal Values
    C10 = -m20 * L1 * L2 * sin(xa(i, 2)) * xa(i, 3)^2 - 2 * m20 * L1 * L2 * sin(xa(i, 2)) * xa(i, 3) * xa(i, 4);
    C20 = m20 * L1 * L2 * sin(xa(i, 2)) * xa(i, 4);

    % Gravitational Components with Nominal Values
    G10 = (m10 + m20) * L1 * cos(xa(i, 2)) + m20 * L2 * cos(xa(i, 1) + xa(i, 2));
    G20 = m20 * L2 * cos(xa(i, 1) + xa(i, 2));

    % Inertia Matrix with Nominal Values
    M0 = [M110 M120; M120 M220];

    % Centrifugal Matrix with Nominal Values
    C0 = [C10; C20];

    % Gravitational Matrix with Nominal Values
    G0 = [G10; G20];

    % Desired angular positions
    qd1 = 1.25 - 7/5 * exp(-ta(i)) + 7/20 * exp(-4 * ta(i));
    qd2 = 1.4 - 7/5 * exp(-ta(i)) + 7/20 * exp(-4 * ta(i));
    qd = [qd1; qd2];

    % Desired qddot
    qddot1 = 7/5 * exp(-ta(i)) - 7/5 * exp(-4 * ta(i));
    qddot = [qddot1; qddot1];

    % Desired qddotdot
    qddotdot1 = -7/5 * exp(-ta(i)) + 28/5 * exp(-4 * ta(i));
    qddotdot = [qddotdot1; qddotdot1];

    % Error in angular Position
    e1 = [xa(i, 1); xa(i, 2)] - qd;

    % Error in qdot
    e2 = [xa(i, 3); xa(i, 4)] - qddot;

    % Surface Parameters
    a = 5;
    b = 7;
    c = diag([2, 2]);
    del = 0.0005;

    % Surface Equation
    S = e2 + c * e1.^(a/b);

    % Calculate equivalent control ueq
    ueq = M0 * (qddotdot - (a/b) * c * (e1.^(a/b - 1))) + C0 + G0;

    % Discontinuous control du
    if norm(S' * inv(M0)) >= del
        du = -(S' * inv(M0))' / (norm(S' * inv(M0))^2) * norm(S) * norm(inv(M0)) * (xa(5) + xa(6) * norm([xa(1); xa(2)]) + xa(7) * norm([xa(3); xa(4)]));
    else
        du = -(S' * inv(M0))' / (del^2) * norm(S) * norm(inv(M0)) * (xa(5) + xa(6) * norm([xa(1); xa(2)]) + xa(7) * norm([xa(3); xa(4)]));
    end

    % Calculate control input U
    Ua(i, 1) = (ueq(1, 1) + du(1, 1));
    Ua(i, 2) = (ueq(2, 1) + du(2, 1));
end

% for loop to find the torque using Terminal Sliding Mode Control
for i = 1:length(tc)
    % Given Nominal values of m1 and m2 (assumed)
    m10 = 0.4;
    m20 = 1.2;

    % Other System parameters
    L1 = 1; % Units meters
    L2 = 0.8; % Units meters
    J1 = 5; % Units kg.m
    J2 = 5; % Units kg

    % Inertia Components with Nominal Values
    M110 = (m10 + m20) * L1^2 + m20 * L2^2 + 2 * m20 * L1 * L2 * cos(xc(i, 2)) + J1;
    M120 = m20 * L2^2 + m20 * L1 * L2 * cos(xc(i, 2));
    M220 = m20 * L2^2 + J2;

    % Centrifugal Components with Nominal Values
    C10 = -m20 * L1 * L2 * sin(xc(i, 2)) * xc(i, 3)^2 - 2 * m20 * L1 * L2 * sin(xc(i, 2)) * xc(i, 3) * xc(i, 4);
    C20 = m20 * L1 * L2 * sin(xc(i, 2)) * xc(i, 4);

    % Gravitational Components with Nominal Values
    G10 = (m10 + m20) * L1 * cos(xc(i, 2)) + m20 * L2 * cos(xc(i, 1) + xc(i, 2));
    G20 = m20 * L2 * cos(xc(i, 1) + xc(i, 2));

    % Inertia Matrix with Nominal Values
    M0 = [M110 M120; M120 M220];

    % Centrifugal Matrix with Nominal Values
    C0 = [C10; C20];

    % Gravitational Matrix with Nominal Values
    G0 = [G10; G20];

    % Desired angular positions
    qd1 = 1.25 - 7/5 * exp(-tc(i)) + 7/20 * exp(-4 * tc(i));
    qd2 = 1.4 - 7/5 * exp(-tc(i)) + 7/20 * exp(-4 * tc(i));
    qd = [qd1; qd2];

    % Desired qddot
    qddot1 = 7/5 * exp(-tc(i)) - 7/5 * exp(-4 * tc(i));
    qddot = [qddot1; qddot1];

    % Desired qddotdot
    qddotdot1 = -7/5 * exp(-tc(i)) + 28/5 * exp(-4 * tc(i));
    qddotdot = [qddotdot1; qddotdot1];

    % Error in angular Position
    e1 = [xc(i, 1); xc(i, 2)] - qd;

    % Error in qdot
    e2 = [xc(i, 3); xc(i, 4)] - qddot;

    % Surface Parameters
    a = 5;
    b = 7;
    c = diag([2, 2]);
    del = 0.0005;

    % Surface Equation
    S = e2 + c * e1.^(a/b);

    % Calculate equivalent control ueq
    ueq = M0 * (qddotdot - (a/b) * c * (e1.^(a/b - 1))) + C0 + G0;

    b0 = 5;
    b1 = 14;
    b2 = 2;

    % Discontinuous control du
    if norm(S' * inv(M0)) >= del
        du = -(S' * inv(M0))' / (norm(S' * inv(M0))^2) * norm(S) * norm(inv(M0)) * (b0 + b1 * norm([xc(1); xc(2)]) + b2 * norm([xc(3); xc(4)]));
    else
        du = -(S' * inv(M0))' / (del^2) * norm(S) * norm(inv(M0)) * (b0 + b1 * norm([xc(1); xc(2)]) + b2 * norm([xc(3); xc(4)]));
    end

    % Calculate control input U
    Uc(i, 1) = (ueq(1, 1) + du(1, 1));
    Uc(i, 2) = (ueq(2, 1) + du(2, 1));
end

% Plot the control input for joint 1
figure(2);
hold on;
plot(ta, Ua(:, 1), 'LineWidth', 1.5);
plot(tc, Uc(:, 1), '--', 'LineWidth', 1.5);  % Dotted line
hold off;
xlabel('Time (s)');
ylabel('Torque 1 (N-m)');
title('Torque 1 (N-m) vs Time(s)');
legend('Torque 1(Adaptive Terminal)','Torque 1(Terminal)');

% Plot the control input for joint 2
figure(6);
hold on;
plot(ta, Ua(:, 2), 'LineWidth', 1.5);
plot(tc, Uc(:, 2), '--', 'LineWidth', 1.5);  % Dotted line
hold off;
xlabel('Time (s)');
ylabel('Torque 2 (N-m)');
title('Torque 2 (N-m) vs Time(s)');
legend('Torque 2(Adaptive Terminal)','Torque 2(Terminal)');
