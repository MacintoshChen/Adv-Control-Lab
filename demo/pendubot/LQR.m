function u = LQR(current_state, ref_state, ref_u, dyn)
    [A,B] = pendubotLinearization(ref_state, ref_u, dyn);
    Q = diag([1, 1, 1, 1]);
    R = 1;
    K = lqr(A,B,Q,R);
    u = ref_u-K*(current_state-ref_state);
end
