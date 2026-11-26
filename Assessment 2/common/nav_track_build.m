function trk = nav_track_build(csvfile, scale, ds, sig_pos, sig_kap)
%NAV_TRACK_BUILD  Scaled, smoothed, uniformly resampled centreline of a closed track.
%   csvfile : TUMFTM racetrack csv (x_m, y_m, w_tr_right_m, w_tr_left_m)
%   scale   : geometric scale (1:10 -> 0.1) applied to coordinates AND widths
%   ds      : target arc-length step [m] (actual step is L/N)
%   sig_pos : std of the periodic Gaussian smoothing of the polyline [m]
%   sig_kap : std of an additional Gaussian smoothing of the curvature [m]
%   trk fields (column vectors, N points, s = (0:N-1)'*ds, closed: point N+1 = point 1):
%     x y psi kap wl wr xl yl xr yr (left/right boundary), s, ds, L, N
M = dlmread(csvfile, ',', 1, 0);
xy = M(:,1:2)*scale; wr0 = M(:,3)*scale; wl0 = M(:,4)*scale;
if norm(xy(1,:)-xy(end,:)) < 1e-9
    xy(end,:) = []; wr0(end) = []; wl0(end) = [];
end
n = size(xy,1);
seg = sqrt(sum(diff([xy; xy(1,:)]).^2, 2));
s0 = [0; cumsum(seg)]; L0 = s0(end);
w = 4;
N = floor(L0/ds); d = L0/N;
s = (0:N-1)'*d;
sxx = [s0(n-w+1:n)-L0; s0(1:n); s0(1:w)+L0];
wrap = @(q) [q(n-w+1:n); q; q(1:w)];
px = interp1(sxx, wrap(xy(:,1)), s, 'spline');
py = interp1(sxx, wrap(xy(:,2)), s, 'spline');
pwr = interp1(sxx, wrap(wr0), s, 'spline');
pwl = interp1(sxx, wrap(wl0), s, 'spline');
% periodic Gaussian smoothing and spectral differentiation
kf = nav_wavenumber(N, d);
Gs = exp(-0.5*(kf*sig_pos).^2);
sm = @(q) real(ifft(fft(q).*Gs));
px = sm(px); py = sm(py); pwr = sm(pwr); pwl = sm(pwl);
dx = real(ifft(1i*kf.*fft(px))); dy = real(ifft(1i*kf.*fft(py)));
sp = sqrt(dx.^2 + dy.^2);
% re-parametrise with the true arc length of the smoothed curve
sa = [0; cumsum(0.5*(sp(1:end-1)+sp(2:end))*d)];
La = sa(end) + 0.5*(sp(end)+sp(1))*d;
d2 = La/N; s = (0:N-1)'*d2;
sxx = [sa(end-w+1:end)-La; sa; sa(1:w)+La];
wrap2 = @(q) [q(end-w+1:end); q; q(1:w)];
px = interp1(sxx, wrap2(px), s, 'spline'); py = interp1(sxx, wrap2(py), s, 'spline');
pwr = interp1(sxx, wrap2(pwr), s, 'spline'); pwl = interp1(sxx, wrap2(pwl), s, 'spline');
kf = nav_wavenumber(N, d2);
dx  = real(ifft(1i*kf.*fft(px)));   dy  = real(ifft(1i*kf.*fft(py)));
ddx = real(ifft(-(kf.^2).*fft(px))); ddy = real(ifft(-(kf.^2).*fft(py)));
sp = sqrt(dx.^2 + dy.^2);
kap = (dx.*ddy - dy.*ddx)./sp.^3;
if sig_kap > 0
    Gk = exp(-0.5*(kf*sig_kap).^2);
    kap = real(ifft(fft(kap).*Gk));
end
psi = atan2(dy, dx);
nx = -sin(psi); ny = cos(psi);                 % left normal
trk.N = N; trk.ds = d2; trk.L = La; trk.s = s;
trk.x = px; trk.y = py; trk.psi = psi; trk.kap = kap;
trk.wl = pwl; trk.wr = pwr;
trk.xl = px + pwl.*nx; trk.yl = py + pwl.*ny;
trk.xr = px - pwr.*nx; trk.yr = py - pwr.*ny;
end
