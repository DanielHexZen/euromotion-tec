function ref = nav_reference(trk, vper, T, aacc, runout)
%NAV_REFERENCE  Time-parametrised reference s_ref(t), v_ref(t) from rest at s = 0.
%   Time stepping: v(k+1) = min(vper(s(k) + T v(k)), v(k) + aacc T), s(k+1) = s(k) + T (v(k)+v(k+1))/2,
%   until s >= L + runout (the speed profile is periodic: the run-out follows lap 2).
%   ref.s, ref.v : column vectors on the T grid (sample 1 = t 0 = start of the lap).
nmax = ceil(3*trk.L/0.5/T);
s = zeros(nmax,1); v = zeros(nmax,1);
n = 1;
for k = 1:nmax-1
    vl = nav_lookup(vper, trk.ds, s(k) + T*v(k));
    v(k+1) = min(vl, v(k) + aacc*T);
    s(k+1) = s(k) + 0.5*T*(v(k) + v(k+1));
    n = k + 1;
    if s(k+1) >= trk.L + runout, break; end
end
ref.s = s(1:n); ref.v = v(1:n); ref.T = T;
ref.k_lap = find(ref.s >= trk.L, 1, 'first');
ref.t_lap = (ref.k_lap - 1)*T;
end
