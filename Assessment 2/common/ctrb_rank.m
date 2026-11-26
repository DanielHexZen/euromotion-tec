function [rc, ro, Wc, Wo] = ctrb_rank(A, B, C)
%CTRB_RANK  Ranks of the controllability and observability matrices (no toolbox).
n = size(A,1);
Wc = zeros(n, n); Wo = zeros(n, n);
Ak = eye(n);
for k = 1:n
    Wc(:, k) = Ak*B;           % [B AB A^2B ...]
    Wo(k, :) = C*Ak;           % [C; CA; CA^2; ...]
    Ak = A*Ak;
end
rc = rank(Wc); ro = rank(Wo);
end
