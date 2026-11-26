function c = lkf_consistency(out, err_pos, Pd, ixfix)
%LKF_CONSISTENCY  NIS and innovation whiteness (and position NEES).
%   out from lkf_run; err_pos [2xN] position error vs reference; ixfix indices of
%   updates. NIS = nu' S^-1 nu (nz dof, S diagonal here; expected value nz).
ixfix = ixfix(ixfix > 1);
nu = out.nu(:,ixfix); Sd = out.Sd(:,ixfix);
nz = size(nu,1);
n = numel(ixfix);
nis = zeros(1,n);
for i = 1:n
    Si = reshape(out.Sf(:,ixfix(i)), nz, nz);
    nis(i) = nu(:,i)'*(Si\nu(:,i));
end
c.nis_mean = mean(nis);
c.dof = nz;
[c.nis_lo, c.nis_hi] = lkf_chi2_bounds(nz*n, n);
% whiteness of the normalised innovation (lag 1..3, per axis)
w = nu ./ sqrt(Sd);
c.ac = zeros(2,3);
for a = 1:2
    s = w(a,:) - mean(w(a,:));
    for L = 1:3
        c.ac(a,L) = sum(s(1:end-L).*s(1+L:end)) / sum(s.^2);
    end
end
c.ac_bound = 1.96/sqrt(n);
c.n_fix = n;
% position NEES (2 dof, uses the diagonal of P; cross terms of P_xy are zero by structure)
nees = err_pos(1,:).^2 ./ Pd(1,:) + err_pos(2,:).^2 ./ Pd(3,:);
nees = nees(2:end);   % skip initial sample
c.nees_mean = mean(nees);
% effective samples are correlated -> bounds are only indicative; use block of fixes
[c.nees_lo, c.nees_hi] = lkf_chi2_bounds(2*numel(nees), numel(nees));
end
