% function dx = acrobot_pfl_ode(t, x, p)
%     q1 = x(1); q2 = x(2);
%     dq1 = x(3); dq2 = x(4);
%     q = [q1;q2]; dq = [dq1;dq2];
%     m1 = p.m1; m2 = p.m2;
%     l1 = p.l1; l2 = p.l2;
%     lc1 = p.lc1; lc2 = p.lc2;
%     I1 = p.I1; I2 = p.I2;
%     g  = p.g; e2 = [0 1];
% 
%     c2 = cos(q2);
%     s2 = sin(q2);
% 
%     % D(q)ddq+C(q, dq)dq+G(q)=Bu
%     % Inertia matrix D(q)
%     d11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2) + I2;
%     d12 = m2*(lc2^2 + l1*lc2*c2) + I2;
%     d21 = d12;
%     d22 = m2*lc2^2 + I2;
%     D = [d11 d12; d21 d22];
% 
%     % C(q,dq)
%     h = -m2*l1*lc2*s2;
%     C = [h*dq2, h*(dq1 + dq2);
%          -h*dq1,      0        ];
% 
%     % Gravity term G(q)
%     phi1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
%     phi2 = m2*lc2*g*cos(q1 + q2);
%     G = [phi1; phi2];
% 
%     B = [0;1];
% 
%     a = e2 * (D \ (-C*dq - G));   % scalar
%     b = e2 * (D \ B);             % scalar
% 
%     % Virtual control v: PD on q2
%     q2_des  = p.q2_des;
%     dq2_des = p.dq2_des;
%     Kp      = p.Kp;
%     Kd      = p.Kd;
% 
%     v = -Kp*(q2 - q2_des) - Kd*(dq2 - dq2_des);
% 
%     % Solve for real torque u
%     u = (v - a) / b;
% 
%     % Full dynamics
%     ddq = D \ (B*u - C*dq - G);
% 
%     dx = [dq; ddq];
% end
% 
function dx = acrobot_pfl_ode(t, x, p)
    q1 = x(1); q2 = x(2);
    dq1 = x(3); dq2 = x(4);
    q = [q1;q2]; dq = [dq1;dq2];
    m1 = p.m1; m2 = p.m2;
    l1 = p.l1; l2 = p.l2;
    lc1 = p.lc1; lc2 = p.lc2;
    I1 = p.I1; I2 = p.I2;
    g  = p.g; e1 = [1 0];  % use first joint

    c2 = cos(q2);
    s2 = sin(q2);

    % D(q)ddq+C(q, dq)dq+G(q)=Bu
    % Inertia matrix D(q)
    d11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2) + I2;
    d12 = m2*(lc2^2 + l1*lc2*c2) + I2;
    d21 = d12;
    d22 = m2*lc2^2 + I2;
    D = [d11 d12; d21 d22];

    % C(q,dq)
    h = -m2*l1*lc2*s2;
    C = [h*dq2, h*(dq1 + dq2);
         -h*dq1,      0        ];

    % Gravity term G(q)
    phi1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
    phi2 = m2*lc2*g*cos(q1 + q2);
    G = [phi1; phi2];

    B = [0;1];

    % pfl on q1
    a = e1 * (D \ (-C*dq - G));   % scalar
    b = e1 * (D \ B);             % scalar

    % Virtual control v: PD on q1
    q1_des  = p.q1_des;
    dq1_des = p.dq1_des;
    Kp      = p.Kp;
    Kd      = p.Kd;

    v = -Kp*(q1 - q1_des) - Kd*(dq1 - dq1_des);

    % Solve for real torque u
    u = (v - a) / b;

    % Full dynamics
    ddq = D \ (B*u - C*dq - G);

    dx = [dq; ddq];
end