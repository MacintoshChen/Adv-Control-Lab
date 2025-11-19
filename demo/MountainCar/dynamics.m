function dx = dynamics(t,x, u, p)
q  = x(1,:);
v  = x(2,:);                               % along-track speed sdot

qClip = min(max(q, -2), 2);
yp    = (pi/2) .* sin(pi*qClip/2);         % y'(q)
den   = sqrt(1 + yp.^2);

qdot = v;                           % qdot = v * cos(theta)
vdot = -p.g .* (yp ./ den) + u;            % vdot = -g*sin(theta) + u

dx = [qdot; vdot];
end
