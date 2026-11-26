% NAV_CHECK_NOISE  Compare the portable generator output with the stored reference.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'common'));
R = load(fullfile(root,'data','noise_nav.mat'));
Z = nav_noise_z(1);
fprintf('max |Z(seed 1) - stored| = %.3e\n', max(abs(Z(:) - R.Z1(:))));
d = 0;
for s = 1:10
    Zs = nav_noise_z(s);
    c = [Zs(1,1), sum(Zs(:,1)), sum(Zs(:).^2), Zs(end,8)];
    d = max(d, max(abs(c - R.chk(s,:))));
end
fprintf('max |checksum difference| seeds 1..10 = %.3e\n', d);
