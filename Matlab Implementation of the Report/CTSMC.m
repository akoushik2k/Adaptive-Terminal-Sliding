function dx = CTSMC(t, x)
% This is a MATLAB function that represents Continuou Terminal Sliding Mode Control for a two-degree freedom robot.

% Given Nominal values of m1 and m2 (assumed)
m10 = 0.4;
m20 = 1.2;

% Other System parameters
L1 = 1;   % Units meters
L2 = 0.8; % Units meters
J1 = 5;   % Units kg.m
J2 = 5;   % Units kg

% Disturbance Vector d(t) = [d1(t) d2(t)]^T
d1 = 0.2 * sin(3 * t) + 0.02 * sin(26 * pi * t);
d2 = 0.1 * sin(2 * t) + 0.01 * sin(26 * pi * t);

% Arbitrary positive values in adaptation laws
x0 = 300;
x1 = 400;
x2 = 200;

% Mass Variation w.r.t. time from Fig.1 and Fig.2
if t <= 3
    m1 = 0.44;
    m2 = 1.32;
elseif t <= 5
    m1 = 0.4;
    m2 = 1.2;
elseif t <= 6
    m1 = 0.36;
    m2 = 1;
elseif t <= 8
    m1 = 0.44;
    m2 = 1.32;
else
    m1 = 0.4;
    m2 = 1.2;
end

% Inertia Components
M11 = (m1 + m2) * L1^2 + m2 * L2^2 + 2 * m2 * L1 * L2 * cos(x(2)) + J1;
M12 = m2 * L2^2 + m2 * L1 * L2 * cos(x(2));
M22 = m2 * L2^2 + J2;

% Inertia Components with Nominal Values
M110 = (m10 + m20) * L1^2 + m20 * L2^2 + 2 * m20 * L1 * L2 * cos(x(2)) + J1;
M120 = m20 * L2^2 + m20 * L1 * L2 * cos(x(2));
M220 = m20 * L2^2 + J2;

% Centrifugal Components
C1 = -m2 * L1 * L2 * sin(x(2)) * x(3)^2 - 2 * m2 * L1 * L2 * sin(x(2)) * x(3) * x(4);
C2 = m2 * L1 * L2 * sin(x(2)) * x(4);

% Centrifugal Components with Nominal Values
C10 = -m20 * L1 * L2 * sin(x(2)) * x(3)^2 - 2 * m20 * L1 * L2 * sin(x(2)) * x(3) * x(4);
C20 = m20 * L1 * L2 * sin(x(2)) * x(4);

% Gravitational Components
G1 = (m1 + m2) * L1 * cos(x(2)) + m2 * L2 * cos(x(1) + x(2));
G2 = m2 * L2 * cos(x(1) + x(2));

% Gravitational Components with Nominal Values
G10 = (m10 + m20) * L1 * cos(x(2)) + m20 * L2 * cos(x(1) + x(2));
G20 = m20 * L2 * cos(x(1) + x(2));

% Inertia Matrix
M = [M11, M12; M12, M22];
M0 = [M110, M120; M120, M220]; % With Nominal Values

% Centrifugal Matrix
C = [C1; C2];
C0 = [C10; C20]; % With Nominal Values

% Gravitational Matrix
G = [G1; G2];
G0 = [G10; G20]; % With Nominal Values

% Disturbance Matrix
D = [d1; d2];

% Desired angular positions
qd1 = 1.25 - 7/5 * exp(-t) + 7/20 * exp(-4 * t);
qd2 = 1.4 - 7/5 * exp(-t) + 7/20 * exp(-4 * t);
qd = [qd1; qd2];

% Desired qddot
qddot1 = 7/5 * exp(-t) - 7/5 * exp(-4 * t);
qddot = [qddot1; qddot1];

% Desired qddotdot
qddotdot1 = -7/5 * exp(-t) + 28/5 * exp(-4 * t);
qddotdot = [qddotdot1; qddotdot1];

% Surface Parameters
a = 5;
b = 7;
c = diag([2, 2]);
del = 0.0005;

% Error in angular Position
e1 = [x(1); x(2)] - qd;

% Error in qdot
e2 = [x(3); x(4)] - qddot;

% Surface Equation
S = e2 + c * e1.^(a / b);

% Equivalent Control
ueq = M0 * (qddotdot - (a / b) * c * (e1.^(a / b - 1))) + C0 + G0;

%arbitary constants b_0, b_1, and b_2
b0 = 300;
b1 = 400;
b2 = 200;

% Discontinuous control du
if norm(S' * inv(M0)) >= del
    du = -(S' * inv(M0))' / (norm(S' * inv(M0))^2) * norm(S) * norm(inv(M0)) * (b0 + b1 * norm([x(1); x(2)]) + b2 * norm([x(3); x(4)]));
else
    du = -(S' * inv(M0))' / (del^2) * norm(S) * norm(inv(M0)) * (b0 + b1 * norm([x(1); x(2)]) + b2 * norm([x(3); x(4)]));
end

U = ueq + du;

% qdot
qdot = [x(3); x(4)];

% q2dot
q2dot = inv(M) * (U + D - G - C);


dx = [qdot; q2dot];
