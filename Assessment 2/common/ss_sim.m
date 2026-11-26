function y = ss_sim(A, B, C, D, u, x0)
%SS_SIM  Simulate x(k+1)=A x(k)+B u(k), y(k)=C x(k)+D u(k) (explicit recursion).
n = size(A,1); N = numel(u);
if nargin < 6, x0 = zeros(n,1); end
x = x0(:); y = zeros(N,1);
for k = 1:N
    y(k) = C*x + D*u(k);
    x = A*x + B*u(k);
end
end
