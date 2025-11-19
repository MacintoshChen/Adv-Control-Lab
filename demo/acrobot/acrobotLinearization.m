function [A, B] = acrobotLinearization(z,u,p)
    q1 = z(1,:);
    q2 = z(2,:);
    dq1 = z(3,:);
    dq2 = z(4,:);
    
    [A, B] = autoGen_acrobotLinearization(...
        q1, q2, dq1, dq2, u,...
        p.m1, p.m2, p.g, p.l1, p.l2);
end