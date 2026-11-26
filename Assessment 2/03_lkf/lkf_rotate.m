function u = lkf_rotate(ax, ay, psi)
%LKF_ROTATE  Body-frame acceleration -> global frame: [aX;aY] = R(psi)[ax;ay].
ax = ax(:)'; ay = ay(:)'; psi = psi(:)';
u = [cos(psi).*ax - sin(psi).*ay; sin(psi).*ax + cos(psi).*ay];
end
