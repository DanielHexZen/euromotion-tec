function ul = pid_ulat_limit(v, par)
%PID_ULAT_LIMIT  Speed-dependent limit of the yaw-rate demand u_lat from |delta| <= delta_max.
%   lat_input is odd and monotone in delta, so the limit is lat_input(v, delta_max).
ul = lat_input(v, par.delta_max, par);
end
