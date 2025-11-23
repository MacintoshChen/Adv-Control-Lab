params.m1 = 1.0;
params.m2 = 1.0;
params.l1 = 1.0;
params.l2 = 2.0;
params.lc1 = params.l1/2;
params.lc2 = params.l2/2;
params.I1 = params.m1*params.l1*params.l1/12;
params.I2 = params.m2*params.l2*params.l2/12;
params.g = 9.8;

params.Kp = 30;
params.Kd = 6;

options = odeset( ...
    'RelTol', 1e-8, ...     % relative tolerance
    'AbsTol', 1e-10, ...    % absolute tolerance
    'MaxStep', 1e-3);       % maximum step size

% Desired position for q2 (e.g. upright: pi)
params.q2_des  = pi/3;
params.dq2_des = 0;
x0 = [-pi/2; 0;0;0];
tSpan = [0 10];
[t, x] = ode45(@(t,x) acrobot_pfl_ode(t, x, params), tSpan, x0, options);

% Plot
figure; 
subplot(2,1,1);
plot(t, wrapTo2Pi(x(:,1)), t, wrapTo2Pi(x(:,2))); grid on;
legend('q1','q2');
ylabel('Angle (rad)');
title('Acrobot angles');

subplot(2,1,2);
plot(t, x(:,3), t, x(:,4)); grid on;
legend('dq1','dq2');
ylabel('Angular velocity (rad/s)');
xlabel('Time (s)');

animate_acrobot(t,x,params);