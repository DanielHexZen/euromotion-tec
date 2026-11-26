function ul = lat_input(v, delta, par)
%LAT_INPUT  Static front end of the lateral model (Hammerstein / LPV scheduling).
%   ul = v * g(delta) / (L * (1 + Kus*v^2)),   g(delta) = tan(delta)*(1 - cs*delta^2)
%   Kus = 0, cs = 0 gives the pure kinematic yaw-rate demand v*tan(delta)/L.
%   Kus [s^2/m^2] : understeer-type loss of yaw-rate gain with speed
%   cs  [1/rad^2] : saturation of the effective steering angle (servo / tyre)
%   par needs fields L, Kus, cs.
ul = v .* tan(delta) .* (1 - par.cs*delta.^2) ./ (par.L*(1 + par.Kus*v.^2));
end
