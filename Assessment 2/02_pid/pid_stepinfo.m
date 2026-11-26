function m = pid_stepinfo(t, y, yf)
%PID_STEPINFO  Step metrics (own implementation) for a response y(t) to a step of final value yf.
%   overshoot [%], 10-90 % rise time, settling time (2 % band: last instant outside +-2 %),
%   peak time, steady-state value (mean of the last 5 % of samples).
y = y(:); t = t(:);
if nargin < 3, yf = 1; end
m.final = mean(y(end-round(0.05*numel(y)):end));
m.overshoot = max(0, (max(y) - yf)/abs(yf)*100);
m.undershoot = max(0, -min(y)/abs(yf)*100);
i10 = find(y >= 0.1*yf, 1); i90 = find(y >= 0.9*yf, 1);
if isempty(i10) || isempty(i90), m.rise = NaN; else, m.rise = t(i90) - t(i10); end
out = find(abs(y - yf) > 0.02*abs(yf));
if isempty(out), m.ts = 0; elseif out(end) == numel(y), m.ts = NaN; else, m.ts = t(out(end)+1); end
[~, ip] = max(y); m.tpeak = t(ip);
end
