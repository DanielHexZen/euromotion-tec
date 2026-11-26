function par = lkf_design(stats, qscale, rscale, nbias, usev, sig_w)
%LKF_DESIGN  Build the discrete LKF model (F,G,H,Q,R,T) from sensor statistics.
%
%   par = lkf_design(stats)               6-state design (with bias states), nominal
%   par = lkf_design(stats,qs,rs)         Q scaled by qs, R scaled by rs
%   par = lkf_design(stats,qs,rs,0)       4-state design (no bias states)
%   par = lkf_design(stats,qs,rs,nb,1)    additionally fuse the wheel speed projected on
%                                         the heading as velocity measurement (nz = 4)
%   par = lkf_design(stats,qs,rs,nb,uv,sw) override the white accel. noise std (e.g. the
%                                         in-motion value stats.sigma_a_mot)
%
%   stats : struct from lkf_noise_stats.
%
%   State x = [p_X; v_X; p_Y; v_Y (; b_x; b_y)]  (global/map frame kinematics),
%   input u = [a_X; a_Y] = R(psi)*(a_measured - calibrated bias), global frame.
%   Per axis (constant acceleration over one sample, zero-order hold):
%       Fa = [1 T; 0 1],  Ga = [T^2/2; T]
%   F = blkdiag(Fa,Fa) (4 states), G = blkdiag(Ga,Ga) (4x2), H = [1 0 0 0; 0 0 1 0].
%
%   Optional bias states b = residual accelerometer bias in the BODY frame
%   (random walk, b(k+1) = b(k) + w_b). The true acceleration is
%   R(psi)(a_m - b), so the bias columns of F are -G*R(psi_k): psi_k is a known
%   exogenous signal, hence the filter stays LINEAR (time-varying F, computed by
%   lkf_fk outside the filter). F0 in par holds the heading-free part.
%
%   Q (white-noise acceleration, discrete):
%        Q_axis = G sigma_w^2 G' = sigma_w^2 [T^4/4 T^3/2; T^3/2 T^2]
%        with ISOTROPIC sigma_w (same for X and Y so that Q stays constant while the
%        body-frame noise is rotated by the changing heading);
%        bias states: Q_b = q_b*T*I2 (random-walk intensity q_b from the standstill
%        bias repeatability).  Without bias states the residual bias is added to
%        the white noise instead: sigma_a^2 = sigma_w^2 + sigma_bres^2.
%   R  = sigma_z^2 * I2   (position-fix noise); with the speed option
%        R = diag(sigma_z^2, sigma_z^2, sigma_v^2, sigma_v^2), H gains the rows
%        [v_X; v_Y] = v_wheel*[cos psi; sin psi] (projection done outside).
if nargin < 2, qscale = 1; end
if nargin < 3, rscale = 1; end
if nargin < 4, nbias = 1; end
if nargin < 5, usev = 0; end
if nargin < 6, sig_w = stats.sigma_a_white; end
T  = stats.T;
Fa = [1 T; 0 1];
Ga = [T^2/2; T];
Z2 = zeros(2,1);
F4 = [Fa zeros(2); zeros(2) Fa];
G4 = [Ga Z2; Z2 Ga];
par.T = T;
par.nx = 4 + 2*nbias;
if nbias
    sw = sig_w;
    Qa = sw^2*(Ga*Ga');
    Q4 = [Qa zeros(2); zeros(2) Qa];
    par.F = [F4 zeros(4,2); zeros(2,4) eye(2)];
    par.G = [G4; zeros(2,2)];
    par.H = [1 0 0 0 0 0; 0 0 1 0 0 0];
    par.Q = qscale*blkdiag(Q4, stats.q_bias*T*eye(2));
    par.P0 = diag([stats.sigma_z^2, 0.05^2, stats.sigma_z^2, 0.05^2, ...
                   stats.sigma_bres_x^2, stats.sigma_bres_y^2]);
    par.sigma_a = sw;
else
    sa = sqrt(sig_w^2 + stats.sigma_bres^2);
    Qa = sa^2*(Ga*Ga');
    par.F = F4; par.G = G4;
    par.H = [1 0 0 0; 0 0 1 0];
    par.Q = qscale*[Qa zeros(2); zeros(2) Qa];
    par.P0 = diag([stats.sigma_z^2, 0.05^2, stats.sigma_z^2, 0.05^2]);
    par.sigma_a = sa;
end
par.R = rscale*stats.sigma_z^2*eye(2);
par.usev = usev;
if usev
    par.H = [par.H; zeros(2, par.nx)]; par.H(3,2) = 1; par.H(4,4) = 1;
    par.R = rscale*diag([stats.sigma_z^2, stats.sigma_z^2, stats.sigma_v^2, stats.sigma_v^2]);
end
par.sigma_z = stats.sigma_z;
end
