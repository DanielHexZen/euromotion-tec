function [ln, ok] = map_boundary(c, k, side)
%MAP_BOUNDARY  Boundary line of LINE segment k on side +1 (left) / -1 (right) (§7.18.2).
%   Left boundary through p0 + w_left*n with inward normal -n; right boundary
%   through p0 - w_right*n with inward normal +n; d = n'*(point).
%   ln = [nx; ny; d]; ok = 0 if the segment is not a LINE or has no boundary there.
%#codegen
ln = zeros(3, 1);
ok = 0;
if k < 1 || k > c.n_seg || c.type(k) ~= 1
    return;
end
dx = c.x1(k) - c.x0(k);  dy = c.y1(k) - c.y0(k);
L  = sqrt(dx*dx + dy*dy);
tx = dx/L;  ty = dy/L;
nx = -ty;   ny = tx;
if side > 0
    if c.bl(k) == 0, return; end
    px = c.x0(k) + c.wl(k)*nx;  py = c.y0(k) + c.wl(k)*ny;
    ln = [-nx; -ny; -(nx*px + ny*py)];
else
    if c.br(k) == 0, return; end
    px = c.x0(k) - c.wr(k)*nx;  py = c.y0(k) - c.wr(k)*ny;
    ln = [nx; ny; nx*px + ny*py];
end
ok = 1;
end
