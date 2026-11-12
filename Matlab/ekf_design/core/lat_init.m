function l = lat_init(p)
%LAT_INIT  State of the lateral-landmark processor (simplified port of spec §7.15).
%#codegen
NM = 4;  N = p.lat_n_max;
l.act   = zeros(1, NM);          % window active
l.done  = zeros(1, NM);          % landmark processed on this pass
l.n     = zeros(1, NM);
l.s     = zeros(N, NM);
l.r     = zeros(N, NM);
l.t     = zeros(N, NM);
l.th0   = zeros(1, NM);
l.bg    = zeros(10, NM);         % post: background ring
l.nbg   = zeros(1, NM);
l.below = zeros(1, NM);
l.in    = zeros(1, NM);
l.ns    = zeros(1, NM);
l.ps    = zeros(200, NM);        % notch samples s, t, r
l.pt    = zeros(200, NM);
l.pr    = zeros(200, NM);
l.n_face = 0;  l.n_face_rej = 0;  l.n_post = 0;  l.n_post_rej = 0;
end
