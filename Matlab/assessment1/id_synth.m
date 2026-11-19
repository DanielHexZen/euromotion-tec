function truth = id_synth(folder, seed)
%ID_SYNTH  Write synthetic calibration logs (all five profiles, firmware format) from a known plant.
%   truth = id_synth(folder, seed) writes folder/screenlog.0, a screen-session style file with
%   junk lines in between, and returns the plant's effective linearised values for checking.
if nargin < 2, seed = 1; end
if ~exist(folder, 'dir'), mkdir(folder); end
rs = RandStream('mt19937ar', 'Seed', seed);

hz = 50;  Ts = 1 / hz;
pl.tau = 0.15;                    % s, per-wheel motor + vehicle
pl.G   = [1.2*0.97, 1.2*1.03];    % m/s per unit duty, left/right mismatch
pl.dz  = 0.05;                    % Coulomb friction, expressed as duty that is lost (v_ss = G (u - dz))
pl.delay = 1;                     % ticks
pl.sigma = 0.005;                 % m/s process noise per tick while driven
pl.m   = [63638e-9, 63638e-9];    % m/count
pl.b   = 223250e-6;               % wheelbase, m

truth = struct('tau', pl.tau, 'K_lon', mean(pl.G), 'K_lat', sum(pl.G) / pl.b, 'delay', pl.delay);

spec = {'S', 40, 0; 'P', 40, 0; 'N', 40, 0; 'D', 30, 10; 'L', 30, 10};
fid = fopen(fullfile(folder, 'screenlog.0'), 'w');
cleanup = onCleanup(@() fclose(fid));
nl = @(varargin) fprintf(fid, [varargin{1} '\r\n'], varargin{2:end});
nl('root@vehicle:~# calib start');
for i = 1:size(spec, 1)
    [p, base, diff] = spec{i, :};
    [bs, ds] = profile(p, base / 100, diff / 100, seed);
    z0 = zeros(round(0.4 * hz), 1);  z1 = zeros(round(1.0 * hz), 1);
    bs = [z0; bs; z1];  ds = [z0; ds; z1];
    uL = round(1000 * (bs - ds)) / 1000;
    uR = round(1000 * (bs + ds)) / 1000;
    cL = wheel(uL, pl.G(1), pl, Ts, rs, pl.m(1));
    cR = wheel(uR, pl.G(2), pl, Ts, rs, pl.m(2));
    nl('# X %s base_pct=%d diff_pct=%d', p, base, diff);
    nl('# X par m_per_cnt_l_nm=%d m_per_cnt_r_nm=%d wheelbase_um=%d hz=%d', ...
        round(pl.m(1)*1e9), round(pl.m(2)*1e9), round(pl.b*1e6), hz);
    nl('# X t_ms,ul_milli,ur_milli,dcnt_l,dcnt_r (comment)');
    for k = 1:numel(uL)
        nl('X,%d,%d,%d,%d,%d', (k-1)*1000/hz, round(1000*uL(k)), round(1000*uR(k)), cL(k), cR(k));
        if k == 100, nl('[ 12.345] some kernel noise'); end
    end
    nl('DONE X');
    nl('root@vehicle:~# ');
end
end

function [b, d] = profile(p, base, diff, seed)
% Duty profile per 20 ms tick, exactly as the firmware generates it (without the zero padding).
switch p
    case 'S'
        b = repelem(base * [0.5 1 0.5 1 0]', 75);  d = zeros(size(b));
    case 'P'
        s = mod(37 * seed, 126) + 1;                 % 7-bit LFSR x^7+x^6+1, nonzero seed
        bits = zeros(100, 1);
        for k = 1:100
            bits(k) = bitget(s, 7);
            s = bitand(bitshift(s, 1), 127) + xor(bitget(s, 7), bitget(s, 6));
        end
        b = [repelem(0.75 * base, 50)'; repelem(base * (0.5 + 0.5 * bits), 4)];  d = zeros(size(b));
    case 'N'
        t = (0:399)' / 50;
        b = [repelem(0.75 * base, 50)'; base * (0.75 + 0.25 * sin(2*pi * (0.2*t + (3 - 0.2) * t.^2 / (2*8))))];
        d = zeros(size(b));
    case 'L'
        d = repelem([0 1 -1 0]' * diff, [50 40 40 75]');
        b = repelem(base, numel(d))';
    case 'D'
        d = repelem([0 1 -1 0 -1 1 0]' * diff, [50 35 35 50 35 35 75]');
        b = repelem(base, numel(d))';
end
end

function cnt = wheel(u, G, pl, Ts, rs, m)
% v(k): mean wheel speed in interval k. v(k+1) = a v(k) + (1-a) G u(k-delay) - Coulomb friction;
% friction opposes motion, cannot reverse it and must be overcome to start (dead zone).
a = exp(-Ts / pl.tau);
fc = (1 - a) * G * pl.dz;
N = numel(u);  v = zeros(N, 1);  cnt = zeros(N, 1);  pos = 0;
for k = 1:N
    cnt(k) = floor((pos + v(k) * Ts) / m) - floor(pos / m);     % quantised, remainder carried
    pos = pos + v(k) * Ts;
    if k < N
        ud = 0;  if k - pl.delay >= 1, ud = u(k - pl.delay); end
        x = a * v(k) + (1 - a) * G * ud;
        if x ~= 0, x = x + pl.sigma * randn(rs); end
        v(k+1) = sign(x) * max(abs(x) - fc, 0);
    end
end
end
