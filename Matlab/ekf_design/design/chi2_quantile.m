function x = chi2_quantile(P, k)
%CHI2_QUANTILE  Chi-square quantile without the Statistics Toolbox.
%
%   x = chi2_quantile(P, k)   P probability (scalar or array), k degrees of freedom
%
%   Start: Wilson-Hilferty approximation (A&S 26.4.17); refinement: bisection
%   on chi2_cdf (gammainc) to 1e-12 relative. Self-check against tabulated
%   values (A&S Table 26.8 / NIST handbook 1.3.6.7.4):
%     chi2_1(0.975) = 5.0239, chi2_1(0.997) = 8.8075 ... ekf_gate1 = 9.0 is P = 0.99730
%     chi2_2(0.99)  = 9.2103, chi2_5(0.025) = 0.8312, chi2_5(0.975) = 12.8325
%   (verify_gate.m prints the comparison).
x = zeros(size(P));
for i = 1:numel(P)
    pr = P(i);
    % Wilson-Hilferty start, z from the inverse normal via erfinv (base MATLAB)
    z  = sqrt(2)*erfinv(2*pr - 1);
    x0 = k*(1 - 2/(9*k) + z*sqrt(2/(9*k)))^3;
    lo = 0;  hi = max(2*x0, 10*k + 50);
    for it = 1:200
        mid = 0.5*(lo + hi);
        if chi2_cdf(mid, k) < pr, lo = mid; else, hi = mid; end
        if hi - lo < 1e-12*max(1, hi), break; end
    end
    x(i) = 0.5*(lo + hi);
end
end
