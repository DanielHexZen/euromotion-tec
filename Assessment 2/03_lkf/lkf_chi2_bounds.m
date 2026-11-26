function [lo, hi] = lkf_chi2_bounds(dof_total, n)
%LKF_CHI2_BOUNDS  95 % two-sided bounds of the AVERAGE of n chi2 variables whose
%   sum has dof_total dof (Wilson-Hilferty approximation, no toolbox needed).
k = dof_total;
zl = -1.959964; zh = 1.959964;
lo = k*(1 - 2/(9*k) + zl*sqrt(2/(9*k)))^3 / n;
hi = k*(1 - 2/(9*k) + zh*sqrt(2/(9*k)))^3 / n;
end
