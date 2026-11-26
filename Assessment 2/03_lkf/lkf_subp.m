function ax = lkf_subp(n, i, wfrac)
%LKF_SUBP  Stacked axes with fixed margins (no clipped labels in Octave/gnuplot).
%   n panels in a column, panel i (1 = top), optional width fraction.
if nargin < 3, wfrac = 0.74; end
top = 0.93; bot = 0.12; gap = 0.13;
hgt = (top - bot - (n-1)*gap)/n;
y0 = top - i*hgt - (i-1)*gap;
ax = subplot('position', [0.20 y0 wfrac hgt]);
end
