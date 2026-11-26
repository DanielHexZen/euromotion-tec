function z = nav_randn(n, m, seed)
%NAV_RANDN  Portable N(0,1) generator, identical in MATLAB and Octave (replaces randn/rng).
%   nav_randn(0, 0, seed)  resets the stream (seed = positive integer; empty result).
%   z = nav_randn(n, m)    n x m unit normals (column-major fill) continuing the stream.
%   z = nav_randn(n, m, seed) resets with seed first, then draws.
%   Generator: xorshift32 (13,17,5) on doubles (all integers < 2^53, exact), uniform
%   u = (x+0.5)/2^32 in (0,1), Box-Muller pairs z = sqrt(-2 ln u1) [cos, sin](2 pi u2).
%   An odd request discards the second value of the last pair (deterministic).
%   Only elementary functions (log, sqrt, cos, sin) enter: results agree across platforms
%   to ~1e-15 (libm rounding).
persistent x
M32 = 4294967296;
if nargin >= 3
    x = mod(seed*2654435769 + 12345, M32);
    if x == 0, x = 2463534242; end
    for i = 1:20, x = nav_xs32(x); end    % warm-up
end
if isempty(x), x = 2463534242; end
total = n*m;
z = zeros(n, m);
if total == 0, return; end
np = ceil(total/2);
u = zeros(2*np,1);
for i = 1:2*np
    x = bitxor(x, mod(bitshift(x, 13), M32));
    x = bitxor(x, bitshift(x, -17));
    x = bitxor(x, mod(bitshift(x, 5), M32));
    u(i) = (x + 0.5)/M32;
end
r = sqrt(-2*log(u(1:2:end)));
a = 2*pi*u(2:2:end);
g = zeros(2*np,1);
g(1:2:end) = r.*cos(a);
g(2:2:end) = r.*sin(a);
z(:) = g(1:total);
end
