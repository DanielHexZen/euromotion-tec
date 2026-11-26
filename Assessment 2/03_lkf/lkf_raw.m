function X = lkf_raw(par, u, x0)
%LKF_RAW  Open-loop double integration of u (same kinematic F,G as the LKF, no
%   updates, interval-mean input). x0 is the 4-element initial kinematic state.
F4 = par.F(1:4,1:4); G4 = par.G(1:4,:);
N = size(u,2); X = zeros(4,N); X(:,1) = x0(1:4);
for k = 2:N, X(:,k) = F4*X(:,k-1) + G4*(0.5*(u(:,k-1)+u(:,k))); end
end
