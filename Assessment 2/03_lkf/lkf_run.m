function out = lkf_run(par, u, z, zvalid, x0, P0, psi)
%LKF_RUN  Run lkf_step over a record.
%   u [2xN] global acceleration, z [nz x N] fixes (position, optionally + velocity), zvalid [1xN], psi [Nx1]
%   heading (only used by the 6-state model through lkf_fk). Sample 1 is the
%   initial state; step k predicts x(k) from x(k-1) with the interval-mean input
%   (u(k-1)+u(k))/2 (removes the half-sample bias of a left-Riemann ZOH input;
%   causal, needs only the current and previous accelerometer sample) and updates
%   with z(:,k).
%   out.X [nx x N], out.Pd (diag of P), out.nu [2xN], out.Sd [2xN].
N = size(u,2); nx = par.nx; nz = size(par.H,1);
X = zeros(nx,N); Pd = zeros(nx,N); NU = zeros(nz,N); Sd = zeros(nz,N); Sf = zeros(nz*nz,N);
x = x0; P = P0;
X(:,1) = x; Pd(:,1) = diag(P);
for k = 2:N
    ubar = 0.5*(u(:,k-1) + u(:,k));
    pk = lkf_fk(par, 0.5*(psi(k-1) + psi(k)));
    [x,P,nu,S] = lkf_step(x,P,ubar,z(:,k),zvalid(k),pk);
    X(:,k) = x; Pd(:,k) = diag(P);
    if zvalid(k), NU(:,k) = nu; Sd(:,k) = diag(S); Sf(:,k) = S(:); end
end
out.X = X; out.Pd = Pd; out.nu = NU; out.Sd = Sd; out.Sf = Sf; out.upd = logical(zvalid(:)');
end
