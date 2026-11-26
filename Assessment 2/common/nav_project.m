function [s, ey, psip, kap, ic] = nav_project(px, py, iprev, trk, W) %#codegen
%NAV_PROJECT  Windowed nearest-point projection of (px,py) onto the closed centreline.
%   iprev : previous (unwrapped, integer) table index, start with 1; search window iprev-W..iprev+W.
%   Returns s  continuous (unwrapped) arc length, ey signed cross-track error (left positive),
%   psip path heading at the foot point, kap path curvature there, ic new unwrapped index.
%   Foot point refinement: first-order projection on the tangent at the nearest table point.
N = trk.N;
cand = (iprev - W):(iprev + W);
m = mod(cand - 1, N) + 1;
d2 = (trk.x(m) - px).^2 + (trk.y(m) - py).^2;
[~, j] = min(d2);
ic = cand(j); mm = m(j);
tx = cos(trk.psi(mm)); ty = sin(trk.psi(mm));
dx = px - trk.x(mm); dy = py - trk.y(mm);
es = dx*tx + dy*ty;
ey = -dx*ty + dy*tx;
s = (ic - 1)*trk.ds + es;
psip = trk.psi(mm) + trk.kap(mm)*es;
kap = trk.kap(mm);
end
