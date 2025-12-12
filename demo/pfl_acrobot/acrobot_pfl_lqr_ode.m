function [dx, u] = acrobot_pfl_lqr_ode(t, x, p)
    q1 = x(1); q2 = x(2);
    dq1 = x(3); dq2 = x(4);
    
    m1 = p.m1; m2 = p.m2;
    l1 = p.l1; l2 = p.l2;
    lc1 = p.lc1; lc2 = p.lc2;
    I1 = p.I1; I2 = p.I2;
    g  = p.g;
    
    c2 = cos(q2);
    s2 = sin(q2);
    
    d11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2) + I2;
    d12 = m2*(lc2^2 + l1*lc2*c2) + I2;
    d21 = d12;
    d22 = m2*lc2^2 + I2;
    D = [d11 d12; d21 d22];
    
    h = -m2*l1*lc2*s2;
    C = [h*dq2, h*(dq1 + dq2); -h*dq1, 0];
    
    phi1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
    phi2 = m2*lc2*g*cos(q1 + q2);
    G = [phi1; phi2];
    
    B = [0; 1];
    
    % check if use lqr
    use_lqr = false;
    if isfield(p, 'use_lqr') && p.use_lqr
        q1_error = wrapToPi(q1 - p.q1_des);
        q2_error = wrapToPi(q2 - p.q2_des);
        error_norm = sqrt(q1_error^2 + q2_error^2);
        
        % switch to lqr
        if error_norm < p.lqr_switch_threshold
            use_lqr = true;
        end
    end
    
    if use_lqr
        x_error = [wrapToPi(q1 - p.q1_des);
                   wrapToPi(q2 - p.q2_des);
                   dq1 - p.dq1_des;
                   dq2 - p.dq2_des];
        u_lqr = p.u_eq - p.K * x_error;
        u = u_lqr;
        u_max = 10;
        u = min(max(u, -u_max), u_max); % clip u
    else
        % partial feedback
        e1 = [1, 0];
        
        a = e1 * (D \ (-C*[dq1; dq2] - G));   % scalar
        b = e1 * (D \ B);                     % scalar
        
        q1_error = wrapToPi(q1 - p.q1_des);
        dq1_error = dq1 - p.dq1_des;
        
        v = -p.Kp * q1_error - p.Kd * dq1_error;
        u = (v - a) / b;
    end
    
    ddq = D \ (B*u - C*[dq1; dq2] - G);
    
    dx = [dq1; dq2; ddq(1); ddq(2)];
end
