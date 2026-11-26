function rows = sysid_rows(segs, na, nb, nk)
%SYSID_ROWS  Regression row indices for a set of segments (each = independent experiment).
%   Rows start n0 = max(na, nk+nb-1) samples after each segment start so that every
%   regressor y(k-i), u(k-nk-j+1) lies inside the same segment.
n0 = max(na, nk + nb - 1);
rows = [];
for s = 1:size(segs,1)
    rows = [rows; (segs(s,1)+n0 : segs(s,2)).']; %#ok<AGROW>
end
end
