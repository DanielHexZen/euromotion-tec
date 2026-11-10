function [s, L, ey, epsi, tx, ty] = map_line_frame(c, k, x, y, th)
%MAP_LINE_FRAME  Frame quantities of LINE segment k (spec §7.18.2).
%   t = (p1 - p0)/L, left normal n = (-t_y, t_x), s = (p - p0).t,
%   e_y = (p - p0).n (left positive), e_psi = wrap(theta - atan2(t_y, t_x)).
%#codegen
dx = c.x1(k) - c.x0(k);
dy = c.y1(k) - c.y0(k);
L  = sqrt(dx*dx + dy*dy);
tx = dx/L;  ty = dy/L;
rx = x - c.x0(k);  ry = y - c.y0(k);
s    = rx*tx + ry*ty;
ey   = -rx*ty + ry*tx;
epsi = ekf_wrap(th - atan2(ty, tx));
end
