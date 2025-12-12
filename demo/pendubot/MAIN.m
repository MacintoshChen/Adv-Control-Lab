%MAIN.m  --  solve swing-up problem for acrobot
%
% This script finds the minimum torque-squared trajectory to swing up the
% acrobot robot: a double pendulum with a motor between the links
%
%

clc; clear;
addpath ../../


function [ref_state, ref_idx] = advance_ref(current_state, ref_path, ref_idx, tol, lookahead)
% ref_path: state_dim × N
% current_state: state_dim × 1

    N = size(ref_path, 2);
    ref_idx = min(max(ref_idx,1), N);

    d = norm(current_state - ref_path(:, ref_idx));

    if (d < tol) && (ref_idx < N)
        ref_idx = ref_idx + 1;
    else
        i = ref_idx;
        while i < N
            if norm(ref_path(:, i) - current_state) >= lookahead
                break;
            end
            i = i + 1;
        end
        ref_idx = i;
    end

    ref_state = ref_path(:, ref_idx);
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Parameters for the dynamics function                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
dyn.m1 = 1;  % elbow mass
dyn.m2 = 1; % wrist mass
dyn.g = 9.81;  % gravity
dyn.l1 = 0.5;   % length of first link
dyn.l2 = 0.5;   % length of second link

t0 = 0;
tF = 2.0;  %For now, force it to take exactly this much time.
x0 = [0;0];   %[q1;q2];  %initial angles   %Stable equilibrium
xF = [pi;pi];  %[q1;q2];  %final angles    %Inverted balance
dx0 = [0;0];   %[dq1;dq2];  %initial angle rates
dxF = [0;0];  %[dq1;dq2];  %final angle rates
maxTorque = 10;  % Max torque at the elbow  (GPOPS goes crazy without this)

%  * The optimal trajectory is not actually constrained by the maximum
%  torque. That being said, GPOPS goes numerically unstable if the torque
%  is not bounded. This does not seem to be a problem with the other
%  methods.

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( pendubotDynamics(x,u,dyn) );

problem.func.pathObj = @(t,x,u)( u.^2 );  %Simple torque-squared

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low = tF;
problem.bounds.finalTime.upp = tF;

% State: [q1;q2;dq1;dq2];

problem.bounds.state.low = [-2*pi; -2*pi; -inf(2,1)];
problem.bounds.state.upp = [ 2*pi;  2*pi;  inf(2,1)];

problem.bounds.initialState.low = [x0; dx0];
problem.bounds.initialState.upp = [x0; dx0];
problem.bounds.finalState.low = [xF; dxF];
problem.bounds.finalState.upp = [xF; dxF];

problem.bounds.control.low = -maxTorque;
problem.bounds.control.upp = maxTorque;



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


%%%% Run the optimization twice: once on a rough grid with a low tolerance,
%%%% and then again on a fine grid with a tight tolerance.

method = 'trapezoid'; %  <-- this is robust, but less accurate
% method = 'direct'; %  <-- this is robust, but some numerical artifacts
% method = 'rungeKutta';  % <-- slow, gets a reasonable, but sub-optimal soln
% method = 'orthogonal';    %  <-- this usually finds bad local minimum
% method = 'gpops';      %  <-- fast, but numerical problem is maxTorque is large

switch method
    case 'direct'
        problem.options(1).method = 'trapezoid';
        problem.options(1).trapezoid.nGrid = 20;
        
        problem.options(2).method = 'trapezoid';
        problem.options(2).trapezoid.nGrid = 40;
        
        problem.options(3).method = 'hermiteSimpson';
        problem.options(3).hermiteSimpson.nSegment = 20;
        
    case 'trapezoid'
        problem.options(1).method = 'trapezoid';
        problem.options(1).trapezoid.nGrid = 20;
        problem.options(2).method = 'trapezoid';
        problem.options(2).trapezoid.nGrid = 40;
        problem.options(3).method = 'trapezoid';
        problem.options(3).trapezoid.nGrid = 60;
        
    case 'rungeKutta'
        problem.options(1).method = 'rungeKutta';
        problem.options(1).defaultAccuracy = 'low';
        
        problem.options(2).method = 'rungeKutta';
        problem.options(2).defaultAccuracy = 'medium';
        
    case 'orthogonal'
        problem.options(1).method = 'chebyshev';
        problem.options(1).chebyshev.nColPts = 9;
        
        problem.options(2).method = 'chebyshev';
        problem.options(2).chebyshev.nColPts = 18;
    case 'gpops'
        problem.options(1).method = 'gpops';
        
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Start with a linear trajectory between four key frames:
% 0  --  initial configuration
% A  --  back swing
% B  --  front swing
% F  --  final configuration
%

tA = t0 + 0.25*(tF-t0);
xA = [-pi/2; 0];
dxA = [0;0];

tB = t0 + 0.75*(tF-t0);
xB = [pi/2; pi];
dxB = [0;0];

problem.guess.time = [t0, tA, tB, tF];
problem.guess.state = [[x0;dx0], [xA; dxA],[xB; dxB], [xF;dxF]];
problem.guess.control = [0, 0, 0, 0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

% Interpolate the solution on a uniform grid for plotting and animation:
tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);


%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%HINT:  type help animate to figure out how to use the keyboard to interact
%with the animation (slow motion, pause, jump forward / backward...)

% Animate the results:
A.plotFunc = @(t,z)( drawAcrobot(t,z,dyn) );
A.speed = 0.25;
A.figNum = 101;
animate(t,z,A)

% Plot the results:
figure(1337); clf; plotAcrobot(t,z,u,dyn);

% Draw a stop-action animation:
figure(1338); clf; drawStopActionAcrobot(soln(end),dyn);


%% ODE45 simulation
x0 = soln(end).interp.state(0);
tSpan = [soln(end).grid.time(1), soln(end).grid.time(end)];

dynOpt = @(t,x) pendubotDynamics(x, soln(end).interp.control(t), dyn);

opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[tSim, xSim] = ode45(dynOpt, tSpan, x0, opts);

% Extract results correctly
q1Sim  = xSim(:,1);
q2Sim  = xSim(:,2);
dq1Sim = xSim(:,3);
dq2Sim = xSim(:,4);

uSim  = arrayfun(@(ti) soln(end).interp.control(ti), tSim);

%% Plot
figure; clf;

subplot(4,1,1)
plot(tSim,q1Sim,'LineWidth',1.5);
ylabel('q_1');
title('Pendubot ODE45 simulation with optimal control');

subplot(4,1,2)
plot(tSim,q2Sim,'LineWidth',1.5);
ylabel('q_2');

subplot(4,1,3)
plot(tSim,dq1Sim,'LineWidth',1.5);
ylabel('dq_1');

subplot(4,1,4)
plot(tSim,uSim,'LineWidth',1.5);
ylabel('u');
xlabel('t');


%% LQR Controller
t = 0;
dt = 0.01;
current_state = soln(end).grid.state(:,1);
traj = current_state;
u_log = [];
t_log = [];
delta_t = soln(end).grid.time(end)-soln(end).grid.time(end-1);
xGrid = soln(end).grid.state;
uGrid = soln(end).grid.control;

K_prev = zeros(1, 4);
ref_state_ts = timeseries(xGrid', tGrid);
ref_u_ts     = timeseries(uGrid', tGrid);
k_log = [];
while t <= soln(end).grid.time(end)
    % Pick Reference State and Input at time t, use interpolation
    ref_state = interp1(tGrid, xGrid.', t).';    % --> column vector
    ref_u     = interp1(tGrid, uGrid.', t).';


    % control
    [K, ok] = LQR(ref_state, ref_u, dyn, K_prev);
    % If K not valid, use previous one
    if ok
        K_prev = K;
    else
        K = K_prev;
    end
    u = ref_u - K*(current_state-ref_state);
    u = max(min(u, maxTorque), -maxTorque);   % clamp
    k_log(:, end+1) = K;

    % integrate (RK4) for better accuracy
    k1 = pendubotDynamics(current_state, u, dyn);
    k2 = pendubotDynamics(current_state + 0.5*dt*k1, u, dyn);
    k3 = pendubotDynamics(current_state + 0.5*dt*k2, u, dyn);
    k4 = pendubotDynamics(current_state + dt*k3, u, dyn);
    current_state = current_state + dt*(k1+2*k2+2*k3+k4)/6;

    % log
    t = t + dt;
    traj(:,end+1) = current_state;
    u_log(end+1) = u;
    t_log(end+1) = t;

end
k_log_ref = timeseries(k_log', t_log');
figure;
subplot(3,1,3);
plot(t_log, u_log,'LineWidth',1.5);
ylabel('u (control)');
xlabel('Time (s)');
