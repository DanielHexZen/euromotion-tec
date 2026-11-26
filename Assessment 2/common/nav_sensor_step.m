function [y, vprev] = nav_sensor_step(xp, vprev, e, k, P) %#codegen
%NAV_SENSOR_STEP  Sensor readings at sample k from the true plant state and error sample e (1x7).
%   y = [ax ay wz zx zy psi_fix fixvalid vwheel]:  ax = (v(k)-v(k-1))/T, ay = v r, wz = r (+ errors),
%   position fix z = p + noise and heading fix every P.fix_step samples (valid flag y(7)).
ax = (xp(4) - vprev)/P.T;
ay = xp(4)*xp(5);
valid = double(mod(k-1, P.fix_step) == 0);
y = [ax + e(1); ay + e(2); xp(5) + e(3); xp(1) + e(4); xp(2) + e(5); xp(3) + e(6); valid; xp(4) + e(7)];
vprev = xp(4);
end
