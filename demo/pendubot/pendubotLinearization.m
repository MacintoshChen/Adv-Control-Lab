function [A, B] = pendubotLinearization(z,u,p)
    q1 = z(1,:);
    q2 = z(2,:);
    dq1 = z(3,:);
    dq2 = z(4,:);

% Vectorized call to the dynamics
    [A, B] = autoGen_pendubotLinearization(...
        q1, q2, dq1, dq2, u,...
        p.m1, p.m2, p.g, p.l1, p.l2);
end