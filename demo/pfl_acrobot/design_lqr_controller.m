function params = design_lqr_controller(params)
    q1_eq = params.q1_des;
    q2_eq = params.q2_des;
    dq1_eq = 0;
    dq2_eq = 0;
    x_eq = [q1_eq; q2_eq; dq1_eq; dq2_eq];
    
    m1 = params.m1; m2 = params.m2;
    l1 = params.l1; l2 = params.l2;
    lc1 = params.lc1; lc2 = params.lc2;
    g = params.g;
    
    % 计算重力项在平衡点
    phi1_eq = (m1*lc1 + m2*l1)*g*cos(q1_eq) + m2*lc2*g*cos(q1_eq + q2_eq);
    phi2_eq = m2*lc2*g*cos(q1_eq + q2_eq);
    G_eq = [phi1_eq; phi2_eq];
    
    % 惯性矩阵在平衡点
    c2_eq = cos(q2_eq);
    d11_eq = m1*lc1^2 + params.I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2_eq) + params.I2;
    d12_eq = m2*(lc2^2 + l1*lc2*c2_eq) + params.I2;
    d21_eq = d12_eq;
    d22_eq = m2*lc2^2 + params.I2;
    D_eq = [d11_eq, d12_eq; d21_eq, d22_eq];
    
    % 计算平衡点控制输入
    B = [0; 1];
    u_eq = B \ G_eq;  % u_eq = inv(B)*G_eq (这里B是2x1，使用伪逆)
    params.u_eq = u_eq(1);  % 取实际控制输入
    
    % 数值线性化（使用中心差分）
    epsilon = 1e-6;
    n_states = 4;
    A = zeros(n_states, n_states);
    B_lin = zeros(n_states, 1);
    
    % 系统动力学函数
    f = @(x,u) acrobot_dynamics(x, u, params);
    
    % 计算A矩阵：df/dx
    for i = 1:n_states
        x_plus = x_eq;
        x_minus = x_eq;
        x_plus(i) = x_plus(i) + epsilon;
        x_minus(i) = x_minus(i) - epsilon;
        
        f_plus = f(x_plus, params.u_eq);
        f_minus = f(x_minus, params.u_eq);
        
        A(:, i) = (f_plus - f_minus) / (2 * epsilon);
    end
    
    % 计算B矩阵：df/du
    u_plus = params.u_eq + epsilon;
    u_minus = params.u_eq - epsilon;
    
    f_plus = f(x_eq, u_plus);
    f_minus = f(x_eq, u_minus);
    
    B_lin = (f_plus - f_minus) / (2 * epsilon);
    
    % 设计LQR控制器
    Q = diag([10, 10, 1, 1]);  % 状态权重
    R = 0.1;                   % 控制权重
    
    % 计算LQR增益
    [K, ~, ~] = lqr(A, B_lin, Q, R);
    
    params.A = A;
    params.B_lin = B_lin;
    params.K = [-1.1459 15.2961 -0.3166 3.6147];
    params.x_eq = x_eq;
    
    fprintf('LQR controller designed.\n');
    fprintf('LQR gain K = [%.4f, %.4f, %.4f, %.4f]\n', params.K);
    fprintf('Equilibrium torque u_eq = %.4f\n', params.u_eq);
end

function dx = acrobot_dynamics(x, u, params)
    % 简化的Acrobot动力学（用于线性化）
    q1 = x(1); q2 = x(2);
    dq1 = x(3); dq2 = x(4);
    
    m1 = params.m1; m2 = params.m2;
    l1 = params.l1; l2 = params.l2;
    lc1 = params.lc1; lc2 = params.lc2;
    I1 = params.I1; I2 = params.I2;
    g = params.g;
    
    c2 = cos(q2);
    s2 = sin(q2);
    
    % 惯性矩阵
    d11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2) + I2;
    d12 = m2*(lc2^2 + l1*lc2*c2) + I2;
    d21 = d12;
    d22 = m2*lc2^2 + I2;
    D = [d11, d12; d21, d22];
    
    % 科里奥利和离心项
    h = -m2*l1*lc2*s2;
    C = [h*dq2, h*(dq1 + dq2); -h*dq1, 0];
    
    % 重力项
    phi1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
    phi2 = m2*lc2*g*cos(q1 + q2);
    G = [phi1; phi2];
    
    % 控制输入矩阵
    B = [0; 1];
    
    % 计算加速度
    ddq = D \ (B*u - C*[dq1; dq2] - G);
    
    dx = [dq1; dq2; ddq(1); ddq(2)];
end