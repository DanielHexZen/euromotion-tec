function m = lkf_metrics(X, ref, Pd, idx)
%LKF_METRICS  2-D position RMSE = sqrt(mean(ex^2+ey^2)) over samples idx.
%   X [4xN] estimate (or [2xN] positions given as rows 1,3 for 4-row), ref [2xN].
if size(X,1) > 2, p = X([1 3],:); else, p = X; end
e = p(:,idx) - ref(:,idx);
m.rmse = sqrt(mean(sum(e.^2,1)));
m.max  = max(sqrt(sum(e.^2,1)));
m.err  = p - ref;
if nargin > 2 && ~isempty(Pd)
    m.cover3 = mean(abs(e(1,:)) <= 3*sqrt(Pd(1,idx)) & abs(e(2,:)) <= 3*sqrt(Pd(3,idx)));
end
end
