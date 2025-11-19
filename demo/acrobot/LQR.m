function u = LQR(current_state, ref_state, ref_u, dyn)
    [A,B] = acrobotLinearization(ref_state, ref_u, dyn);
    Q = diag([5, 5, 1, 1]);
    R = 1;
    K = lqr(A,B,Q,R);
    u = ref_u-K*(current_state-ref_state);
end
