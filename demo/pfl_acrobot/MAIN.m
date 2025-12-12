% params.m1 = 1.0;
% params.m2 = 1.0;
% params.l1 = 1.0;
% params.l2 = 2.0;
% params.lc1 = params.l1/2;
% params.lc2 = params.l2/2;
% params.I1 = params.m1*params.l1*params.l1/12;
% params.I2 = params.m2*params.l2*params.l2/12;
% params.g = 9.8;
% 
% params.Kp = 30;
% params.Kd = 6;
% 
% options = odeset( ...
%     'RelTol', 1e-8, ...     % relative tolerance
%     'AbsTol', 1e-10, ...    % absolute tolerance
%     'MaxStep', 1e-3);       % maximum step size
% 
% % Desired position for q2 (e.g. upright: pi)
% params.q1_des  = pi/2;
% params.dq1_des = 0;
% x0 = [-pi/2; 0;0;0];
% tSpan = [0 10];
% [t, x] = ode45(@(t,x) acrobot_pfl_ode(t, x, params), tSpan, x0, options);
% 
% % Plot
% figure; 
% subplot(2,1,1);
% plot(t, wrapTo2Pi(x(:,1)), t, wrapTo2Pi(x(:,2))); grid on;
% legend('q1','q2');
% ylabel('Angle (rad)');
% title('Acrobot angles');
% 
% subplot(2,1,2);
% plot(t, x(:,3), t, x(:,4)); grid on;
% legend('dq1','dq2');
% ylabel('Angular velocity (rad/s)');
% xlabel('Time (s)');
% 
% animate_acrobot(t,x,params);
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

% target position
params.q1_des  = pi/2;
params.q2_des  = 0;
params.dq1_des = 0;
params.dq2_des = 0;

% LQR
params.use_lqr = true;
params.lqr_switch_threshold = 0.6;

% LQR Controller for equilibrium
params = design_lqr_controller_symbolic(params);

options = odeset( ...
    'RelTol', 1e-8, ...     % relative tolerance
    'AbsTol', 1e-10, ...    % absolute tolerance
    'MaxStep', 1e-3);       % maximum step size

x0 = [-pi/2; 0; 0; 0];  % init pos
tSpan = [0 10];
[t, x] = ode45(@(t,x) acrobot_pfl_lqr_ode(t, x, params), tSpan, x0, options);

u = zeros(length(t), 1);
for i = 1:length(t)
    [~, u_i] = acrobot_pfl_lqr_ode(t(i), x(i,:)', params);
    u(i) = u_i;
end

% Plot
figure; 
subplot(3,1,1);
plot(t, wrapTo2Pi(x(:,1)), t, wrapTo2Pi(x(:,2))); grid on;
hold on; 
plot([t(1) t(end)], [params.q1_des params.q1_des], 'r--');
plot([t(1) t(end)], [params.q2_des params.q2_des], 'b--');
legend('q1', 'q2', 'q1_{des}', 'q2_{des}');
ylabel('Angle (rad)');
title('Acrobot Angles with LQR Control');

subplot(3,1,2);
plot(t, x(:,3), t, x(:,4)); grid on;
legend('dq1','dq2');
ylabel('Angular velocity (rad/s)');

subplot(3,1,3);
plot(t, u); grid on;
ylabel('Control Torque (Nm)');
xlabel('Time (s)');
title('Control Input');

animate_acrobot(t,x,params);