function [K, ok] = LQR(ref_state, ref_u, dyn, K_prev)
    [A,B] = pendubotLinearization(ref_state, ref_u, dyn);
    ok = true;
    Co = ctrb(A,B);
    n = size(A, 1);
    ok = true;
    if rank(Co) < n
        K = K_prev;
        ok = false;
        return;
    end
    Q = diag([5, 5, 1, 1]);
    R = 1;
    K = lqr(A,B,Q,R);
    if max(abs(K)) > 100 || max(abs(K ./ (K_prev + 1e-9))) > 5
        ok = false;
    end
end
