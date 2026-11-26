function [ym, yf, yo, segid] = arx_eval(a, b, u, y, segs)
%ARX_EVAL  Pooled measured / free-run / one-step-ahead outputs over several segments.
%   segs : K-by-2 matrix [k_start k_end] of CONTIGUOUS index ranges. In every
%          segment the free-run simulation is restarted from the measured
%          initial condition (first n0 samples), then driven by u only.
%          The first n0 samples of each segment are excluded from the pooled
%          vectors (they are initial conditions, not predictions).
%   a, b : ARX polynomials in z^-1 (a = [1 a1..], b = [0..0 b1..]).
n0 = max(numel(a)-1, numel(b)-1);
ym = []; yf = []; yo = []; segid = [];
for s = 1:size(segs,1)
    rr = (segs(s,1):segs(s,2)).';
    [yh, y1] = arx_sim(a, b, u, y, rr);
    ym = [ym; y(rr(n0+1:end))];       %#ok<AGROW>
    yf = [yf; yh(n0+1:end)];          %#ok<AGROW>
    yo = [yo; y1(n0+1:end)];          %#ok<AGROW>
    segid = [segid; s*ones(numel(rr)-n0,1)]; %#ok<AGROW>
end
end
