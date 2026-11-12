function [R, out] = verify_wp_levers(variant, runs)
%VERIFY_WP_LEVERS  One-at-a-time study of what moves the waypoint accuracy (F-12, D-44).
%
%   [R, out] = verify_wp_levers(variant, runs)
%   variant  'a' baseline: rect10, default parameters
%            'b' start heading 0.1 deg   (ekf_p0_th; the true start heading is drawn from it)
%            'c' odometry scale 0.1 %    (ekf_p0_s; the true scale is drawn from it)
%            'd' ekf_kd = 1e-5 m         (filter only: the plant has no random distance
%                                         noise, only the encoder quantisation)
%            'e' boundary on every leg   (course rect10_all)
%            'f' b + c                   (start jig + phase-1 calibration, course unchanged)
%            'g' f + one post 1.5 m before the end of every leg (course rect10_post)
%   runs     run indices, seed = 100 + index (default 1:5); the same seeds for
%            every variant, so the noise sequences are identical
%
%   Every run is cached under the tag 'lev_<variant>' (see verify_monte_carlo),
%   so an interrupted process continues where it stopped. Delete
%   results/mc_cache/lev_* after any code or parameter change.
if nargin < 2 || isempty(runs), runs = 1:5; end
p = ekf_params();
scen = 'rect10';
switch variant
    case 'a'
    case 'b', p.ekf_p0_th = 0.1*pi/180;
    case 'c', p.ekf_p0_s = 0.001;
    case 'd', p.ekf_kd = 1.0e-5;
    case 'e', scen = 'rect10_all';
    case 'f', p.ekf_p0_th = 0.1*pi/180;  p.ekf_p0_s = 0.001;
    case 'g', p.ekf_p0_th = 0.1*pi/180;  p.ekf_p0_s = 0.001;  scen = 'rect10_post';
    otherwise, error('verify_wp_levers: variant %s', variant);
end
fprintf('=== waypoint lever study, variant %s, course %s, runs %s ===\n', variant, scen, mat2str(runs));
[R, out] = verify_monte_carlo(runs, scen, 'p', p, 'tag', ['lev_' variant]);
for i = 1:numel(R), fprintf('%s\n', vres_line(R(i))); end
W = out.wpd;
m1 = W(:, 2) == 1;
en = hypot(W(:, 4), W(:, 5));
mw = W(:, 2) < 99;
[~, iw] = max(en.*mw);
fprintf('LEVER %s: N = %d, wp est max %.1f cm rms %.1f cm, wp true max %.1f cm, NEES %.1f %%, NIS %.1f %%\n', ...
    variant, numel(runs), 100*max(out.wpe), 100*sqrt(mean(out.wpe.^2)), 100*max(out.wpt), 100*out.f_nees, 100*out.f_nis);
fprintf('LEVER %s: WP1 along rms %.1f max %.1f mm, cross rms %.1f max %.1f mm\n', variant, ...
    1e3*sqrt(mean(W(m1,4).^2)), 1e3*max(abs(W(m1,4))), 1e3*sqrt(mean(W(m1,5).^2)), 1e3*max(abs(W(m1,5))));
fprintf('LEVER %s: worst waypoint: seed %d wp %d, along %.1f mm, cross %.1f mm (sigma %.1f / %.1f mm)\n', variant, ...
    W(iw,1), W(iw,2), 1e3*W(iw,4), 1e3*W(iw,5), 1e3*W(iw,8), 1e3*W(iw,9));
end

function s = vres_line(r)
f = fieldnames(r);
s = '';
for k = 1:numel(f)
    v = r.(f{k});
    if ischar(v), s = [s, v, ' | ']; else, s = [s, num2str(v), ' | ']; end %#ok<AGROW>
end
end
