function u = LQR(current_state, ref_state, g)
    [A,B] = mc_linearize(ref_state(1), g);
    Q = diag([1, 1]);
    R = 1;
    K = lqr(A,B,Q,R);
    u = -K*(current_state-ref_state);
end

function [A,B] = mc_linearize(qstar, g)
s   = (pi/2)*sin(pi*qstar/2);
sp  = (pi^2/4)*cos(pi*qstar/2);
A21 = -g * sp / (1 + s^2)^(3/2);
A = [0 1; A21 0];
B = [0; 1];
end

