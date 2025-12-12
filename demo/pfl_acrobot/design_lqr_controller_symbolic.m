function params = design_lqr_controller_symbolic(params)
    % use symbolic toolbox
    
    syms q1 q2 dq1 dq2 u real
    
    m1 = params.m1; m2 = params.m2;
    l1 = params.l1; l2 = params.l2;
    lc1 = params.lc1; lc2 = params.lc2;
    I1 = params.I1; I2 = params.I2;
    g = params.g;
    
    x = [q1; q2; dq1; dq2];
    
    c2 = cos(q2);
    d11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2) + I2;
    d12 = m2*(lc2^2 + l1*lc2*c2) + I2;
    d21 = d12;
    d22 = m2*lc2^2 + I2;
    D = [d11, d12; d21, d22];
    
    h = -m2*l1*lc2*sin(q2);
    C = [h*dq2, h*(dq1 + dq2); -h*dq1, 0];
    
    phi1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
    phi2 = m2*lc2*g*cos(q1 + q2);
    G = [phi1; phi2];
    
    B_mat = [0; 1];
    
    ddq = D \ (B_mat*u - C*[dq1; dq2] - G);
    
    f = [dq1; dq2; ddq(1); ddq(2)];
    
    q1_eq = params.q1_des;
    q2_eq = params.q2_des;
    
    G_eq = subs(G, {q1, q2}, {q1_eq, q2_eq});
    params.u_eq = double(G_eq(2));  % u_eq = G_eq(2)，因为B=[0;1]
    
    A_sym = jacobian(f, x);
    B_sym = jacobian(f, u);
    
    A = double(subs(A_sym, {q1, q2, dq1, dq2, u}, ...
                   {q1_eq, q2_eq, 0, 0, params.u_eq}));
    B = double(subs(B_sym, {q1, q2, dq1, dq2, u}, ...
                   {q1_eq, q2_eq, 0, 0, params.u_eq}));
    
    % Q and R matrix
    Q = diag([50, 50, 15, 15]);
    R = 10;
    
    [K, S, E] = lqr(A, B, Q, R);
    
    params.A = A;
    params.B_lin = B;
    params.K = K;
    params.x_eq = [q1_eq; q2_eq; 0; 0];
    
    fprintf('Symbolic LQR controller designed.\n');
    fprintf('LQR gain K = [%.4f, %.4f, %.4f, %.4f]\n', K);
    fprintf('Equilibrium torque u_eq = %.4f\n', params.u_eq);
end