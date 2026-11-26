function y = nav_lookup(arr, ds, s)
%NAV_LOOKUP  Linear interpolation of a periodic arc-length table arr(i) at s = (i-1)*ds.
%   s may be any real number (negative or beyond the lap length: wrapped).
N = numel(arr);
j = s/ds;
i0 = floor(j); fr = j - i0;
a = arr(mod(i0, N) + 1);
b = arr(mod(i0 + 1, N) + 1);
y = (1 - fr).*a + fr.*b;
end
