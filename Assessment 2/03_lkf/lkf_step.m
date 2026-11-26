function [x, P, nu, S] = lkf_step(x, P, u, z, zvalid, par) %#codegen
%LKF_STEP  One cycle of the discrete LINEAR Kalman filter (predict + update).
%
%   [x,P,nu,S] = lkf_step(x,P,u,z,zvalid,par)
%
%   Model (sampling time T, state x = [p_X; v_X; p_Y; v_Y] (4 states) or
%   x = [p_X; v_X; p_Y; v_Y; b_x; b_y] (6 states, b = body-frame accelerometer bias),
%   input u = [a_X; a_Y]):
%       x(k+1) = F(k) x(k) + G u(k) + w(k),   w ~ N(0,Q)  (process model)
%   F(k) is constant for the 4-state model. For the 6-state model its bias columns
%   are -G*R(psi_k) (known heading, set outside by lkf_fk) -> linear time-varying.
%       z(k)   = H x(k)        + e(k),     e ~ N(0,R)     (position fix, optionally
%                                                       + wheel-speed velocity: nz = 2 or 4)
%   u = global-frame acceleration = R(psi)*[a_x; a_y]_measured (bias removed). The
%   rotation by the heading psi is done OUTSIDE this function, so the filter
%   itself is linear with constant F,G,H,Q,R.
%
%   Predict:   x- = F x + G u
%              P- = F P F' + Q
%   Update (only if zvalid ~= 0, i.e. a fresh position fix is available):
%              nu = z - H x-                    innovation
%              S  = H P- H' + R                 innovation covariance
%              K  = P- H' S^-1                  Kalman gain
%              x  = x- + K nu
%              P  = (I-KH) P- (I-KH)' + K R K'  Joseph form (keeps P symmetric PSD)
%   Without a fix, x = x-, P = P-.
%
%   Inputs : x [nx x1], P [nx x nx], u [2x1], z [nz x1], zvalid (scalar, 0/1),
%            par  struct with fixed-size fields F[nx x nx] G[nx x 2] H[nz x nx]
%            Q[nx x nx] R[nz x nz]   (nz = 2: position; nz = 4: position + velocity)   (nx = 4 or 6)
%            (as returned by lkf_design).
%   Outputs: x, P posterior; nu [nz x1], S [nz x nz] innovation and its covariance
%            (zeros if no fix) for NIS / whiteness checks; may be ignored.
%
%   Code-generation friendly: fixed sizes, no dynamic allocation, no cell/strings.
%   Intended for use inside a Simulink MATLAB Function block (pass par as a
%   parameter/constant struct).

F = par.F; G = par.G; H = par.H; Q = par.Q; R = par.R;

% ---- predict -------------------------------------------------------------
x = F*x + G*u;
P = F*P*F' + Q;

nz = size(H,1);
nu = zeros(nz,1);
S  = zeros(nz,nz);

% ---- update --------------------------------------------------------------
if zvalid ~= 0
    nu  = z - H*x;
    S   = H*P*H' + R;
    S   = 0.5*(S + S');
    K   = (P*H') / S;
    x   = x + K*nu;
    IKH = eye(size(F,1)) - K*H;
    P   = IKH*P*IKH' + K*R*K';        % Joseph form
    P   = 0.5*(P + P');
end
end
