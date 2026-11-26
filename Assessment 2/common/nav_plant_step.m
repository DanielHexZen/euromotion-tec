function xn = nav_plant_step(xp, v_cmd, delta, P) %#codegen
%NAV_PLANT_STEP  One sample of the simulation plant: identified Gv, Gr + kinematic bicycle.
%   xp = [px py psi v r r1 u1 u2 da]' (r = r(k), r1 = r(k-1), u1 = u_lat(k-1), u2 = u_lat(k-2),
%   da = steering angle applied at k-1).
%   v(k+1) = av v(k) + bv v_cmd(k);  r(k+1) = a1r r(k) + a2r r(k-1) + br u_lat(k-2) (3-sample delay)
%   u_lat(k) = lat_input(v(k), delta(k));  psi' = r, x' = v cos(psi), y' = v sin(psi)
%   (midpoint integration of the position). Saturations: 0 <= v_cmd <= 3, |delta| <= delta_max,
%   steering-servo rate limit |d delta/dt| <= P.servo_rate (applied before lat_input).
T = P.T;
vc = min(max(v_cmd, 0), P.vcmd_max);
dl = min(max(delta, -P.par.delta_max), P.par.delta_max);
v = xp(4); r = xp(5);
rs = P.servo_rate*T;
da = xp(9) + min(max(dl - xp(9), -rs), rs);
ul = lat_input(v, da, P.par);
vn = max(P.av*v + P.bv*P.kv_scale*vc, 0);
rn = P.a1r*r + P.a2r*xp(6) + P.br*P.kr_scale*xp(8);
vav = 0.5*(v + vn);
hm = xp(3) + 0.5*T*r;
xn = zeros(9,1);
xn(1) = xp(1) + T*vav*cos(hm);
xn(2) = xp(2) + T*vav*sin(hm);
xn(3) = xp(3) + T*r;
xn(4) = vn; xn(5) = rn; xn(6) = r; xn(7) = ul; xn(8) = xp(7); xn(9) = da;
end
