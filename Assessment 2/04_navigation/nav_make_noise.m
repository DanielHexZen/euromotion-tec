% NAV_MAKE_NOISE  Store the reference noise for cross-platform checks in data/noise_nav.mat:
%   Z1 = unit normals [12000 x 8] for seed 1 (used by the nominal lap) and per-seed checksums
%   (seeds 1..10). Run nav_check_noise in MATLAB/Octave to confirm identical streams.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'common'));
Z1 = nav_noise_z(1);
chk = zeros(10,4);
for s = 1:10
    Z = nav_noise_z(s);
    chk(s,:) = [Z(1,1), sum(Z(:,1)), sum(Z(:).^2), Z(end,8)];
end
note = 'Unit normals from common/nav_randn.m (xorshift32 + Box-Muller). chk columns: Z(1,1), sum(Z(:,1)), sum(Z(:).^2), Z(end,8) for seeds 1..10';
save(fullfile(root,'data','noise_nav.mat'), 'Z1', 'chk', 'note', '-v7');
