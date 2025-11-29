clear; clc;

%% Physical Parameters
m1 = 1; m2 = 1;
l1 = 0.5; l2 = 0.5;
g  = 9.81;
dyn.m1 = 1;  % elbow mass
dyn.m2 = 1; % wrist mass
dyn.g = 9.81;  % gravity
dyn.l1 = 0.5;   % length of first link
dyn.l2 = 0.5;   % length of second link

%% Equilibrium definitions
x_eq1 = [pi; 0; 0; 0];       % fully upright
x_eq2 = [pi; 0.15; 0; 0];    % nearby second equilibrium
u_eq  = 0;

%% Linearization at upright equilibrium
[A,B] = autoGen_acrobotLinearization( ...
        x_eq1(1), x_eq1(2), x_eq1(3), x_eq1(4), u_eq, ...
        m1, m2, g, l1, l2);

%% Discretization
dt = 0.02;
Ad = eye(4) + A*dt; 
Bd = B*dt;

%% Prediction horizon
N = 10;        % few time steps, as assignment requires

%% Reference Traj
xRef = zeros(4, N);
for k = 1:N
    alpha = (k-1)/(N-1);
    xRef(:,k) = x_eq1 + alpha*(x_eq2 - x_eq1);
end

%% Augmented state
% X_k = [x_k; x_k+1; ...; x_k+N-1]
n = 4;
nx = n*N;
nu = 1;

% Construct block-shift Abar and Bbar
Abar = zeros(nx,nx);
Bbar = zeros(nx,nu);

% First row block
Abar(1:n,1:n) = Ad;
Bbar(1:n,1)   = Bd;

% Remaining shifts
for i = 2:N
    Abar((i-1)*n+1:i*n, (i-2)*n+1:(i-1)*n) = eye(n);
end

Q = diag([50 50 1 1]);
R = 0.1;
Qbar = kron(eye(N), Q);   % block diagonal

%% Solve gain
[K_aug,~,~] = dlqr(Abar, Bbar, Qbar, R);

%% Simulation
x = x_eq1;     % initialize
traj = x;
uLog = [0];

for k = 1:100       % simulate several steps
    Xref = reshape(xRef, [], 1);

    Xk = zeros(nx,1);
    Xk(1:n) = x;

    u = -K_aug * (Xk - Xref);

    k1 = acrobotDynamics(x, u, dyn);
    k2 = acrobotDynamics(x + 0.5*dt*k1, u, dyn);
    k3 = acrobotDynamics(x + 0.5*dt*k2, u, dyn);
    k4 = acrobotDynamics(x + dt*k3, u, dyn);
    x = x + dt*(k1+2*k2+2*k3+k4)/6;

    traj(:,end+1) = x;
    uLog(end+1) = u;

end

%% --- Plot Results ---
t = 0:dt:dt*(size(traj,2)-1);

figure; clf;
subplot(5,1,1)
plot(t, traj(1,:), 'LineWidth',2); hold on;
yline(x_eq2(1),'--k');
ylabel('q1');

subplot(5,1,2)
plot(t, traj(2,:), 'LineWidth',2); hold on;
yline(x_eq2(2),'--k');
ylabel('q2');

subplot(5,1,3)
plot(t, traj(3,:), 'LineWidth',2); hold on;
ylabel('dq1');

subplot(5,1,4)
plot(t, traj(4,:), 'LineWidth',2); hold on;
ylabel('dq2');

subplot(5,1,5)
plot(t, uLog,'LineWidth',2);
ylabel('u'); xlabel('time');

sgtitle('MPC (DARE) Transition Between Equilibria');
