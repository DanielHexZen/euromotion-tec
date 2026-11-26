function E = nav_noise(P, N, seed)
%NAV_NOISE  Additive sensor error sequences [N x 7] from the PORTABLE generator nav_randn
%   (identical in MATLAB and Octave). Unit normals are drawn once as NMAX x 8 per seed:
%   columns 1 a_x white, 2 a_y white, 3/4 bias random-walk steps x/y, 5 gyro, 6/7 position fix
%   x/y, 8 heading fix. Result columns: 1,2 accelerometer a_x, a_y (bias random walk + white),
%   3 gyro (bias + white), 4,5 position fix noise, 6 heading fix noise, 7 wheel-speed noise
%   (z column 9). N <= NMAX = 12000.
S = P.sens; T = P.T;
Z = nav_noise_z(seed);
if N > size(Z,1), error('nav_noise: N exceeds NMAX'); end
sqb = sqrt(S.qb*T);
bx = S.bax + sqb*[0; cumsum(Z(1:N-1,3))];
by = S.bay + sqb*[0; cumsum(Z(1:N-1,4))];
E = zeros(N,7);
E(:,1) = bx + S.sax*Z(1:N,1);
E(:,2) = by + S.say*Z(1:N,2);
E(:,3) = S.bwz + S.swz*Z(1:N,5);
E(:,4) = S.sz*Z(1:N,6);
E(:,5) = S.sz*Z(1:N,7);
E(:,6) = S.spsi*Z(1:N,8);
E(:,7) = S.svw*Z(1:N,9);
end
