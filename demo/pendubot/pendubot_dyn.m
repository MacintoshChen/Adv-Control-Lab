function ddq = pendubot_dyn(q, dq, tau)
% PENDUBOT_DYN  Pendubot dynamics: D(q) ddq + C(q,dq) dq + G(q) = [tau; 0]
% q  : [q1; q2]
% dq : [dq1; dq2]
% tau: scalar torque at joint 1
% ddq: [ddq1; ddq2]

% ---------- Parameters ----------
dyn.m1 = 1;       % kg
dyn.m2 = 1;       % kg
dyn.l1 = 0.5;     % m
dyn.l2 = 0.5;     % m
dyn.g  = 9.81;    % m/s^2

m1 = dyn.m1; m2 = dyn.m2;
l1 = dyn.l1; l2 = dyn.l2;
g  = dyn.g;

% assume uniform rods
lc1 = l1/2; lc2 = l2/2;
I1  = (1/12)*m1*l1^2;
I2  = (1/12)*m2*l2^2;

% shorthand
q1  = q(1);  q2  = q(2);
dq1 = dq(1); dq2 = dq(2);

% ---------- Inertia matrix D(q) ----------
a = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2);
b = m2*l1*lc2;
d = I2 + m2*lc2^2;

D = [a + 2*b*cos(q2), d + b*cos(q2);
     d + b*cos(q2),   d];

% ---------- Coriolis / centrifugal term C(q,dq)*dq (as a vector) ----------
Cvec = [-b*sin(q2)*(2*dq1 + dq2)*dq2;
         b*sin(q2)*dq1^2];

% ---------- Gravity term G(q) ----------
e = m1*lc1 + m2*l1;
f = m2*lc2;

Gvec = g * [ e*cos(q1) + f*cos(q1+q2);
             f*cos(q1+q2) ];

% ---------- Solve for ddq ----------
Tau = [tau; 0];          % only joint 1 is actuated
ddq = D \ (Tau - Cvec - Gvec);
end
