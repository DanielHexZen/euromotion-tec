function Z = nav_noise_z(seed)
%NAV_NOISE_Z  Unit-normal noise matrix [12000 x 9] for a seed (cached within a session).
%   Reference values (seed 1) are stored in data/noise_nav.mat (see nav_make_noise.m).
persistent cs cz
if isempty(cs) || cs ~= seed
    cz = nav_randn(12000, 9, seed);
    cs = seed;
end
Z = cz;
end
