function F = chi2_cdf(x, k)
%CHI2_CDF  Chi-square CDF without toolboxes: F = P(k/2, x/2), the regularised
%   lower incomplete gamma function (Abramowitz & Stegun 26.4.19), evaluated with
%   base-MATLAB gammainc.
F = zeros(size(x));
m = x > 0;
F(m) = gammainc(x(m)/2, k/2);
end
