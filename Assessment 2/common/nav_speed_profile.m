function [v, vlim] = nav_speed_profile(kap, ds, vmax, alat, aacc, adec, par)
%NAV_SPEED_PROFILE  Periodic (closed-loop) speed profile v(s) on the centreline table.
%   vlim = min(vmax, sqrt(alat/|kap|), steering-feasible speed) with
%   steering feasibility |kap| <= 0.8*k_delta*lat_input(v,delta_max)/v (80 % of the yaw-rate
%   capacity); then forward (aacc) and backward (adec) acceleration limits applied cyclically.
N = numel(kap);
ka = abs(kap(:));
vlim = min(vmax, sqrt(alat./max(ka, 1e-6)));
vg = (0.3:0.01:3.5)';
kc = 0.8*par.k_delta*lat_input(vg, par.delta_max, par)./vg;   % decreasing in v
vcap = interp1(flipud(kc), flipud(vg), ka, 'linear', 'extrap');
vcap = min(max(vcap, vg(1)), vg(end));
vlim = min(vlim, vcap);
v = vlim;
for it = 1:3
    for idx = 2:2*N
        i = mod(idx-1, N) + 1; ip = mod(idx-2, N) + 1;
        v(i) = min(v(i), sqrt(v(ip)^2 + 2*aacc*ds));
    end
    for idx = 2*N-1:-1:1
        i = mod(idx-1, N) + 1; ip = mod(idx, N) + 1;
        v(i) = min(v(i), sqrt(v(ip)^2 + 2*adec*ds));
    end
end
end
