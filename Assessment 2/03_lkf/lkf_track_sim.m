function tr = lkf_track_sim(csvfile, T, t_stand, scale, v_max, a_lat_max, a_lon_max)
%LKF_TRACK_SIM  Ground-truth lap on the scaled Mexico City centreline.
%   Centreline x,y [m] * scale (1:10 -> 0.1), periodic cubic spline resampled
%   uniformly in arc length, Gaussian low-pass (periodic, FFT) to remove the
%   polyline kinks, speed profile v(s) = min(v_max, sqrt(a_lat_max/|kappa|))
%   with forward/backward limits a_lon_max, starting and ending at standstill.
%   A standstill phase of t_stand seconds precedes the lap (bias calibration).
%   Returns truth on the T grid: t, px,py,vx,vy,psi (unwrapped), at (tangential
%   acc = body a_x), kappa, ay_b (= v^2 kappa, body a_y), r (= v kappa), s.
M = dlmread(csvfile, ',', 1, 0);
xy = M(:,1:2)*scale;
if norm(xy(1,:)-xy(end,:)) < 1e-9, xy(end,:) = []; end
n = size(xy,1);
seg = sqrt(sum(diff([xy; xy(1,:)]).^2,2));
s0 = [0; cumsum(seg)]; L = s0(end);
% periodic spline: wrap w points on each side
w = 4;
ds = 0.02;
Ns = floor(L/ds); ds = L/Ns;
s = (0:Ns-1)'*ds;
sxx = [s0(n-w+1:n)-L; s0(1:n); s0(1:w)+L];
xx = [xy(n-w+1:n,1); xy(:,1); xy(1:w,1)];
yy = [xy(n-w+1:n,2); xy(:,2); xy(1:w,2)];
px = interp1(sxx, xx, s, 'spline');
py = interp1(sxx, yy, s, 'spline');
% periodic Gaussian smoothing + spectral differentiation
sig = 0.4;                                    % m smoothing length (scaled)
kf = [0:Ns/2, -Ns/2+1:-1]'*(2*pi/(Ns*ds));    % angular wavenumber
if mod(Ns,2) == 1, kf = [0:(Ns-1)/2, -(Ns-1)/2:-1]'*(2*pi/(Ns*ds)); end
G = exp(-0.5*(kf*sig).^2);
sm = @(q) real(ifft(fft(q).*G));
px = sm(px); py = sm(py);
dx  = real(ifft(1i*kf.*fft(px))); dy  = real(ifft(1i*kf.*fft(py)));
sp = sqrt(dx.^2 + dy.^2);
% re-sample uniformly in the ACTUAL arc length of the smoothed curve
sa = [0; cumsum(0.5*(sp(1:end-1)+sp(2:end))*ds)]; La = sa(end) + 0.5*(sp(end)+sp(1))*ds;
Ns2 = Ns; ds = La/Ns2; s = (0:Ns2-1)'*ds; L = La;
sxx = [sa(end-w+1:end)-La; sa; sa(1:w)+La];
px = interp1(sxx, [px(end-w+1:end); px; px(1:w)], s, 'spline');
py = interp1(sxx, [py(end-w+1:end); py; py(1:w)], s, 'spline');
dx  = real(ifft(1i*kf.*fft(px))); dy  = real(ifft(1i*kf.*fft(py)));
ddx = real(ifft(-(kf.^2).*fft(px))); ddy = real(ifft(-(kf.^2).*fft(py)));
sp = sqrt(dx.^2 + dy.^2);                    % ~1 (arc-length parameter)
kap = (dx.*ddy - dy.*ddx)./sp.^3;
psi_s = unwrap(atan2(dy,dx));
dsa = ds*sp;                                 % true path length per grid step
% speed profile on the (open) lap s in [0,L): v(0)=v(end)=0
v = min(v_max, sqrt(a_lat_max./max(abs(kap),1e-6)));
v(1) = 0; v(end) = 0;
for it = 1:2
    for i = 2:Ns, v(i) = min(v(i), sqrt(v(i-1)^2 + 2*a_lon_max*dsa(i))); end
    for i = Ns-1:-1:1, v(i) = min(v(i), sqrt(v(i+1)^2 + 2*a_lon_max*dsa(i))); end
end
at_s = 0.5*gradient(v.^2, ds)./sp;           % dv/dt = v dv/dl
dt_i = 2*dsa(1:end-1)./max(v(1:end-1)+v(2:end), 1e-9);
t_s = [0; cumsum(dt_i)];
t_lap = t_s(end);
% Time grid (standstill first). The kinematics are integrated on the T grid in
% closed loop: the tangential acceleration is looked up at the CURRENT arc
% length, v and s follow by exact constant-acceleration steps, so v, s and the
% analytic accelerations are mutually consistent and the lap profile is respected.
Nmax = floor((t_stand + 1.5*t_lap)/T) + 1;
nst = round(t_stand/T);
at = zeros(Nmax,1); vq = zeros(Nmax,1); sq = zeros(Nmax,1);
Ls = s(end);
Nt = Nmax;
for k = 1:Nmax-1
    if k > nst
        j = min(max(sq(k)/ds, 0), Ns-1.001);
        i0 = floor(j); fr = j - i0;
        at(k) = (1-fr)*at_s(i0+1) + fr*at_s(i0+2);
    end
    vq(k+1) = max(vq(k) + at(k)*T, 0);
    sq(k+1) = sq(k) + vq(k)*T + 0.5*at(k)*T^2;
    if sq(k+1) >= Ls - ds
        Nt = k+1; break;
    end
end
at = at(1:Nt); vq = vq(1:Nt); sq = sq(1:Nt);
t = (0:Nt-1)'*T;
t_lap = t(end) - t_stand;
tr.t = t; tr.s = sq;
tr.px = interp1(s, px, sq, 'spline'); tr.py = interp1(s, py, sq, 'spline');
tr.psi = interp1(s, psi_s, sq, 'linear');
tr.kappa = interp1(s, kap, sq, 'linear');
% Truth derivatives from the truth positions themselves (central differences), so
% that double integration of the noise-free body-frame accelerometer signal
% reproduces the trajectory up to O((omega*T)^2) (no discretisation artefacts).
pxd = tr.px; pyd = tr.py;
vxc = zeros(Nt,1); vyc = vxc; Ax = vxc; Ay = vxc;
vxc(2:end-1) = (pxd(3:end)-pxd(1:end-2))/(2*T); vyc(2:end-1) = (pyd(3:end)-pyd(1:end-2))/(2*T);
Ax(2:end-1)  = (pxd(3:end)-2*pxd(2:end-1)+pxd(1:end-2))/T^2;
Ay(2:end-1)  = (pyd(3:end)-2*pyd(2:end-1)+pyd(1:end-2))/T^2;
Ax(end) = Ax(end-1); Ay(end) = Ay(end-1);
vxc(end) = vxc(end-1); vyc(end) = vyc(end-1);
tr.vx = vxc; tr.vy = vyc; tr.v = hypot(vxc,vyc);
tr.at   =  cos(tr.psi).*Ax + sin(tr.psi).*Ay;      % body a_x (tangential)
tr.ay_b = -sin(tr.psi).*Ax + cos(tr.psi).*Ay;      % body a_y (= v^2 kappa)
tr.r = zeros(Nt,1); tr.r(2:end-1) = (tr.psi(3:end)-tr.psi(1:end-2))/(2*T); tr.r(end) = tr.r(end-1);
tr.lap_time = t_lap; tr.length = L; tr.max_alat = max(abs(tr.ay_b));
tr.centre = [px py]; tr.sline = s;
tr.params = struct('scale',scale,'v_max',v_max,'a_lat_max',a_lat_max,'a_lon_max',a_lon_max);
end
