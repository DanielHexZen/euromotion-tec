function xp = nav_plant_init(trk, offset0)
%NAV_PLANT_INIT  Plant state [px py psi v r r(k-1) u(k-1) u(k-2) delta_applied(k-1)] at rest, on the centreline
%   shifted by offset0 to the left of the start point, heading along the path.
nx = -sin(trk.psi(1)); ny = cos(trk.psi(1));
xp = [trk.x(1) + offset0*nx; trk.y(1) + offset0*ny; trk.psi(1); 0; 0; 0; 0; 0; 0];
end
