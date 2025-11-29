function x = pendubot_f(x,u)
% x = [q1; q2; dq1; dq2]
% u = scalar torque
dyn.m1 = 1;  % elbow mass
dyn.m2 = 1; % wrist mass
dyn.g = 9.81;  % gravity
dyn.l1 = 0.5;   % length of first link
dyn.l2 = 0.5;   % length of second link
dt = 0.01;
k1 = pendubotDynamics(x, u, dyn);
k2 = pendubotDynamics(x + 0.5*dt*k1, u, dyn);
k3 = pendubotDynamics(x + 0.5*dt*k2, u, dyn);
k4 = pendubotDynamics(x + dt*k3, u, dyn);
x = x + dt*(k1+2*k2+2*k3+k4)/6;
end