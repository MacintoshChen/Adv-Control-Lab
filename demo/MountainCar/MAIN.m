clc; clear;
addpath ../../   % <-- adjust this path to where optimTraj.m lives

function [ref_state, ref_idx] = advance_ref(current_state, ref_path, ref_idx, tol, lookahead)
    N = size(ref_path,1);
    ref_idx = min(max(ref_idx,1), N);

    dx = ref_path(ref_idx,1) - current_state(1);
    dy = ref_path(ref_idx,2) - current_state(2);
    d  = hypot(dx, dy);

    if (d < tol) && (ref_idx < N)
        ref_idx = ref_idx + 1;
    else
        i = ref_idx;
        while i < N
            dx = ref_path(i,1) - current_state(1);
            dy = ref_path(i,2) - current_state(2);
            if hypot(dx,dy) >= lookahead
                break;
            end
            i = i + 1;
        end
        ref_idx = i;
    end

    ref_state = ref_path(ref_idx, :)';
end


%% --- Parameters ---
p.g = 4;  % gravity

%% --- Dynamics and cost ---
problem.func.dynamics = @(t,x,u) dynamics(t,x,u,p);
problem.func.pathObj  = @(t,x,u) u.^2;   % minimize control effort

%% --- Bounds ---
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low   = 1.0;
problem.bounds.finalTime.upp   = 8.0;

problem.bounds.state.low  = [-2; -3];
problem.bounds.state.upp  = [ 2;  3];
problem.bounds.initialState.low = [0; 0];
problem.bounds.initialState.upp = [0; 0];
problem.bounds.finalState.low   = [2; -0.02];
problem.bounds.finalState.upp   = [2;  0.02];

problem.bounds.control.low = -2;
problem.bounds.control.upp =  2;

%% --- Initial guess (reverse then forward) ---
problem.guess.time   = [0, 2.0, 5.0];
problem.guess.state  = [0, -1.0, 2.0;    % position q
                        0,  0.0, 0.0];   % velocity dq
problem.guess.control = [-1.5, 1.5, 0.0];

%% --- Solver settings ---
problem.options.method = 'hermiteSimpson';
problem.options.defaultAccuracy = 'high';
problem.options.meshMaxIterations = 5;
problem.options.useScaling = true;

%% --- Solve ---
soln = optimTraj(problem);

%% --- Extract interpolated solution ---
tFine = linspace(soln.grid.time(1), soln.grid.time(end), 600);
xFine = soln.interp.state(tFine);
uFine = soln.interp.control(tFine);

q  = xFine(1,:);
dq = xFine(2,:);
u  = uFine;

%% --- Plots ---
figure(1); clf;
subplot(3,1,1)
plot(tFine,q,'LineWidth',1.5);
ylabel('q (position)'); title('Mountain Car Optimal Control');
subplot(3,1,2)
plot(tFine,dq,'LineWidth',1.5);
ylabel('dq (velocity)');
subplot(3,1,3)
plot(tFine,u,'LineWidth',1.5);
ylabel('u (control)'); xlabel('t');

%% --- Animation ---
figure(2); clf; axis equal; hold on; grid on;
xlabel('x'); ylabel('Height');
title('Mountain Car Animation');
set(gca,'XLim',[-4.5,5],'YLim',[-1.5,1.5]);

% Terrain
xTerrain = linspace(-4.5,5,400);
yTerrain = -cos(pi/2 * min(max(xTerrain,-2),2));  % flat outside [-2,2]
plot(xTerrain, yTerrain, 'k', 'LineWidth', 1.5);

% Car marker
yCar = -cos(pi/2 * min(max(q(1),-2),2));
carMarker = plot(q(1), yCar, 'ro', 'MarkerFaceColor','r', 'MarkerSize',10);
trail = plot(q(1), yCar, 'b.');

for k = 1:length(tFine)
    qk = q(k);
    yCar = -cos(pi/2 * min(max(qk,-2),2));

    set(carMarker,'XData',qk,'YData',yCar);
    set(trail,'XData',q(1:k),'YData',-cos(pi/2 * min(max(q(1:k),-2),2)));

    title(sprintf('t = %.2f s, dq = %.2f, u = %.2f', tFine(k), dq(k), u(k)));
    drawnow;
    pause(0.02);
end

%% ---Simulation---
% Simulate with ODE45 using the optimal control
x0 = soln.interp.state(0);      % initial state from solution
tSpan = [soln.grid.time(1), soln.grid.time(end)];

dynOpt = @(t,x) mountainCarDynamics(t, x, soln.interp.control(t), p);

opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[tSim, xSim] = ode45(dynOpt, tSpan, x0, opts);

% Extract results
qSim  = xSim(:,1);
dqSim = xSim(:,2);
uSim  = arrayfun(@(ti) soln.interp.control(ti), tSim);

% Plot
figure; clf;
subplot(3,1,1)
plot(tSim,qSim,'LineWidth',1.5);
ylabel('q'); title('ODE45 Simulation with Optimal Control');

subplot(3,1,2)
plot(tSim,dqSim,'LineWidth',1.5);
ylabel('dq');

subplot(3,1,3)
plot(tSim,uSim,'LineWidth',1.5);
ylabel('u'); xlabel('t');


%% ---LQR Controller---
% --- parameters ---
dt = 0.1;
xGrid = soln.grid.state;
tGrid = soln.grid.time;
current_state = xGrid(:,1);
goal = xGrid(:,end);
ref_idx = 1;
tol = 0.75;
lookahead = 3.0;

% --- logs for plotting ---
traj = current_state;
u_log = 0;      % first control
t_log = 0;      % simulation time

t = 0;
while norm(current_state - goal) >= 0.2 && ref_idx < size(xGrid,2)
    % Pick the closest reference point ahead
    [ref_state, ref_idx] = advance_ref(current_state, xGrid.', ref_idx, tol, lookahead);

    u = LQR(current_state, ref_state, p.g);
    % Euler simulation for next step
    dx = mountainCarDynamics(0, current_state, u, p);
    current_state = current_state + dt * dx;

    t = t + dt;
    traj(:,end+1) = current_state;
    u_log(end+1) = u;
    t_log(end+1) = t;
end

% --- plotting results ---
figure;
subplot(3,1,1);
plot(t_log, traj(1,:),'LineWidth',1.5);
ylabel('q (position)');
title('Closed-loop LQR Simulation');

subplot(3,1,2);
plot(t_log, traj(2,:),'LineWidth',1.5);
ylabel('dq (velocity)');

subplot(3,1,3);
plot(t_log, u_log,'LineWidth',1.5);
ylabel('u (control)');
xlabel('Time (s)');
