function S = tf2canon(num, den)
%TF2CANON  Discrete tf -> controllable and observable canonical state space.
%   num, den : coefficients of G(z) = num(z)/den(z), DESCENDING powers of z,
%              same length n+1, den(1) = 1 (monic), num may contain leading zeros
%              (pure delay / strictly proper). Written out explicitly (no tf2ss):
%       G(z) = (b0 z^n + b1 z^(n-1) + ... + bn) / (z^n + a1 z^(n-1) + ... + an)
%   Controllable canonical form (companion matrix, last-row coefficients):
%       A_c = [0 1 0 ..0; 0 0 1 ..0; ...; -an ... -a2 -a1],  B_c = [0..0 1]'
%       C_c = [bn-b0*an ... b1-b0*a1],                         D_c = b0
%   Observable canonical form (dual):  A_o = A_c', B_o = C_c', C_o = B_c', D_o = D_c.
%   Both realise exactly the same transfer function.
num = num(:).'; den = den(:).';
if abs(den(1)-1) > 1e-12, num = num/den(1); den = den/den(1); end
n = numel(den) - 1;
if numel(num) ~= n+1, error('num and den must have equal length'); end
a = den(2:end);                 % a1..an
b0 = num(1); bb = num(2:end);   % b1..bn
Ac = zeros(n); Ac(1:n-1, 2:n) = eye(n-1); Ac(n, :) = -fliplr(a);
Bc = zeros(n,1); Bc(n) = 1;
Cc = fliplr(bb - b0*a);         % [bn-b0 an ... b1-b0 a1]
Dc = b0;
S.Ac = Ac; S.Bc = Bc; S.Cc = Cc; S.Dc = Dc;
S.Ao = Ac.'; S.Bo = Cc.'; S.Co = Bc.'; S.Do = Dc;
S.n = n; S.num = num; S.den = den;
end
