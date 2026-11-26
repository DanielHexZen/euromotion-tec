function m = fit_metrics(y, yhat)
%FIT_METRICS  Goodness-of-fit measures between measured y and model output yhat.
%   m.fit  : NRMSE fit [%] = 100*(1 - ||y-yhat|| / ||y-mean(y)||)
%   m.r2   : coefficient of determination 1 - SS_res/SS_tot
%   m.rmse : root-mean-square error (unit of y)
%   m.n    : number of samples used
y = y(:); yhat = yhat(:);
ok = isfinite(y) & isfinite(yhat);
y = y(ok); yhat = yhat(ok);
res = y - yhat;
sst = sum((y - mean(y)).^2);
m.fit  = 100*(1 - norm(res)/sqrt(sst));
m.r2   = 1 - sum(res.^2)/sst;
m.rmse = sqrt(mean(res.^2));
m.n    = numel(y);
end
