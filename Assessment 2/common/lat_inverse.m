function delta = lat_inverse(ul_des, v, par)
%LAT_INVERSE  Steering angle that realises the yaw-rate demand ul_des at speed v.
%   Inverts lat_input(v, delta, par) over |delta| <= par.delta_max by bisection
%   (the map is monotone in delta on that interval). Vectorised; the result is
%   saturated at +-delta_max when the demand is not reachable.
%   Use: delta_cmd = lat_inverse(u_lat_from_PID, v, par).
ul_des = ul_des(:); v = v(:) .* ones(size(ul_des));
lo = -par.delta_max*ones(size(ul_des)); hi = par.delta_max*ones(size(ul_des));
for it = 1:50
    mid = 0.5*(lo+hi);
    f = lat_input(v, mid, par) - ul_des;
    up = f < 0;                       % map still too small -> move up
    lo(up) = mid(up); hi(~up) = mid(~up);
end
delta = 0.5*(lo+hi);
end
