function dx = mountainCarDynamics(t, x, u, p)
% mountainCarDynamics: Computes [qdot; dqdot]
%   x = [q; dq], u = control input, p.g = gravity

q  = x(1);
dq = x(2);

% Terrain slope y'(q) = pi/2 * sin(pi*q/2) for |q|<=2, flat otherwise
qClip = min(max(q, -2), 2);
yp    = (pi/2) * sin(pi*qClip/2);

% Tangential acceleration due to gravity
grav  = -p.g * yp / sqrt(1 + yp^2);

% Equation of motion: ddq = grav + u
ddq = grav + u;

dx = [dq; ddq];
end
