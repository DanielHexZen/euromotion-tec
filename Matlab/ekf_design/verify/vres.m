function r = vres(name, pass, value, crit, note)
%VRES  One row of the verification summary (run_all prints the table).
%   pass: 1 pass, 0 fail, -1 informational / not a pass-fail check
if nargin < 5, note = ''; end
r = struct('name', name, 'pass', pass, 'value', value, 'crit', crit, 'note', note);
end
