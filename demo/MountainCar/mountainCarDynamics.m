function dx = mountainCarDynamics(t, x, u, p)
q  = x(1);
dq = x(2);

qClip = min(max(q, -2), 2);
yp    = (pi/2) * sin(pi*qClip/2);

grav  = -p.g * yp / sqrt(1 + yp^2);

ddq = grav + u;

dx = [dq; ddq];
end
