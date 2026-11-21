function test_assessment1()
%TEST_ASSESSMENT1  Synthetic round trip: identify the known plant, check parser robustness.
here = fileparts(mfilename('fullpath'));
addpath(here);
tmp = tempname;  mkdir(tmp);
cleanup = onCleanup(@() rmdir(tmp, 's'));
data = fullfile(tmp, 'data');
truth = id_synth(data);

% truncated run next to the complete ones must be skipped with a warning
fid = fopen(fullfile(data, 'x_trunc.log'), 'w');
fprintf(fid, '# X S base_pct=40 diff_pct=0\r\n# X par m_per_cnt_l_nm=63638 m_per_cnt_r_nm=63638 wheelbase_um=223250 hz=50\r\nX,0,0,0,0,0\r\nX,20,400,400,3,3\r\n');
fclose(fid);
lastwarn('');
runs = id_read_log({fullfile(data, 'screenlog.0'), fullfile(data, 'x_trunc.log')});
[~, wid] = lastwarn;
assert(isequal([runs.profile], 'SPNDL'), 'parser: expected the 5 complete runs only');
assert(strcmp(wid, 'id_read_log:truncated'), 'parser: truncated run not reported');

S = run_assessment1('data', data, 'synthetic', false, 'label', 'synthetic', 'out', fullfile(tmp, 'out'));
% The dead zone enters the model as the Coulomb term c, so the slope K stays the plant's 1.2 m/s.
% Lateral: the e*u term absorbs the yaw offset that the wheel gain mismatch causes while driving.
chk = @(name, x, ref, tol) assert(abs(x / ref - 1) < tol, '%s = %g, plant %g (tol %g%%)', name, x, ref, 100*tol);
chk('lon tau', S.lon.tau, truth.tau, 0.10);   chk('lon K', S.lon.K, truth.K_lon, 0.05);
chk('lat tau', S.lat.tau, truth.tau, 0.08);   chk('lat K', S.lat.K, truth.K_lat, 0.05);
assert(S.lon.d == truth.delay && S.lat.d == truth.delay, 'delay not recovered');
R = S.R2;
assert(min([R.lonValS R.lonValN R.latVal]) > 0.9, 'validation R2 <= 0.9');
assert(S.lat.nu == 2, 'yaw-offset term e not selected');
assert(all(isfile(fullfile(tmp, 'out', {'results.tex', 'cv_table.tex'}))), 'tex files missing');
fprintf('test_assessment1: PASS\n');
end
