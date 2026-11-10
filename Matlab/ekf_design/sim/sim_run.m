function L = sim_run(cfg)
%SIM_RUN  Script (reference) implementation of the multirate vehicle loop.
%
%   L = sim_run(cfg)    cfg from sim_config
%
%   Scheduler, identical in order to the Simulink model of build_ekf_model.m:
%     IMU samples at imu_t0 + i/960 s (never on a 1 kHz tick) use the plant
%     output of the last tick (zero-order hold)
%     every 1 kHz tick k (t = k ms), in this order:
%       plant (Moore: output at t_k, then integrate with the command of tick k-1)
%       encoder snapshot, IMU drain (yaw ring), ground sensors, lateral sensors
%       estimator  : 50 Hz at tick phase 1 (D-30, inside S_CTRL)
%                    or 200 Hz at phase 0 (cfg.ekf_rate = 200, former S_EST)
%       guidance   : 50 Hz at tick phase 1, after the estimator
%     guidance -> plant through a one-tick delay; guidance status -> estimator
%     through a one-control-period delay (the drv status of the previous tick).
%
%   L (log, one row per estimator step):
%     t, est (N x 70), upd (N x 56), truth = [x y th bg s w_true], gdn (N x 10)
%     wp: waypoint records [id t xhat yhat xtrue ytrue]
%     bkq: back-shift queries [t_age dpos_est_err dth_err phase] (maneuver only)

p  = cfg.p;
pc = ekf_param_cast(p, cfg.cls);
c  = cfg.course;
tr = cfg.tr;
K  = round(cfg.T_end*1000);

plant = sim_plant_init(tr, c);
imu   = sim_imu_init(tr);
ys    = sim_yaw_init(pc);
gnd   = sim_gnd_init(tr, c, p);
lat   = sim_lat_init(tr, p);
es    = sim_est_init(pc, c);
gs    = sim_gdn_init(p, c);

div_est = round(1000/cfg.ekf_rate);
ph_est  = 1;
if cfg.ekf_rate ~= 50, ph_est = 0; end

cmd    = [0; 0];
y_imu  = zeros(5, 1);
y_gst  = [1; 1; 0; gs.state];
y_est  = zeros(70, 1);
y_gdn  = zeros(10, 1);
t_imu_next = tr.imu_t0;

Nmax = ceil(K/div_est) + 2;
L.t     = zeros(Nmax, 1);
L.est   = zeros(Nmax, 70);
L.upd   = zeros(Nmax, 56);
L.truth = zeros(Nmax, 6);
L.gdn   = zeros(Nmax, 10);
L.wp    = zeros(0, 6);
L.bkq   = zeros(0, 4);
n = 0;
% truth history for back-shift queries
TH = zeros(4, 256);
qrng = sim_rng_init(cfg.seed, 7);

for k = 0:K
    t = k*1e-3;
    while t_imu_next < t
        [imu, y_imu] = sim_imu_step(imu, plant.ylast, tr);
        t_imu_next = tr.imu_t0 + imu.i/tr.f_imu;
    end
    [plant, yp] = sim_plant_step(plant, cmd, tr);
    TH(:, mod(k, 256) + 1) = [t; yp(2); yp(3); yp(4)];
    y_enc = sim_enc_step(yp, p);
    [ys, y_yaw]  = sim_yaw_step(ys, y_imu, pc);
    [gnd, y_gnd] = sim_gnd_step(gnd, yp, tr, c, p);
    [lat, y_lat] = sim_lat_step(lat, yp, tr, c, p);
    if mod(k - ph_est, div_est) == 0 && k >= ph_est
        [es, y_est, y_upd] = sim_est_step(es, y_enc, y_yaw, y_gnd, y_lat, y_gst, pc, c);
        n = n + 1;
        L.t(n) = t;
        L.est(n, :) = y_est';
        L.upd(n, :) = y_upd';
        L.truth(n, :) = [yp(2), yp(3), yp(4), y_imu(4), 0.5*(yp(11) + yp(12)), y_imu(5)];
        L.gdn(n, :) = y_gdn';
        if cfg.bk_query && n > 60
            % back-shift query: a measurement 10..100 ms old (exact ms grid)
            [qrng, u] = sim_rand(qrng);
            kq  = k - (10 + floor(91*u));
            [bk, stb] = ekf_back_at(es.ekf, pc, kq*1000);
            if stb == 0
                dtrue = TH(2:4, mod(k, 256) + 1) - TH(2:4, mod(kq, 256) + 1);
                ep  = norm(double(bk(1:2)) - dtrue(1:2));
                eth = abs(ekf_wrap(double(bk(3)) - dtrue(3)));
                L.bkq(end+1, :) = [(k - kq)*1e-3, ep, eth, phase_of(y_gdn)];
            end
        end
    end
    if mod(k - 1, 20) == 0 && k >= 1
        [gs, y_gdn] = sim_gdn_step(gs, y_est, p, c);
        y_gst = y_gdn(5:8);
        if y_gdn(9) > 0
            L.wp(end+1, :) = [y_gdn(9), t, y_est(1), y_est(2), yp(2), yp(3)];
        end
    end
    cmd = y_gdn(1:2);
end
L.n = n;
L.t = L.t(1:n);  L.est = L.est(1:n, :);  L.upd = L.upd(1:n, :);
L.truth = L.truth(1:n, :);  L.gdn = L.gdn(1:n, :);
L.cfg = cfg;
L.gnd_samples = gnd.n_samples;
L.gnd_events  = gnd.seq;
end

function ph = phase_of(y_gdn)
% 1 straight, 2 arc, 3 in-place turn, 0 standing (from the commanded v, w)
v = y_gdn(3);  w = y_gdn(4);
if abs(v) < 1e-6 && abs(w) < 1e-6, ph = 0;
elseif abs(w) < 1e-6, ph = 1;
elseif abs(v) < 1e-6, ph = 3;
else, ph = 2; end
end
