function capture_sysid(port, test_id, fname)
% CAPTURE_SYSID  Start an identification test on the car and save the log.
%   capture_sysid("COM5", 2, "data/lon_prbs_1.csv")
%   test_id: 1 lon_step, 2 lon_prbs, 3 lon_chirp, 4 lat_prbs, 5 lat_dlc, 6 lat_slc
%   Car on USB (USART2, 115200 baud). Put the car on the floor first.
% MR2006B Assessment 1 - team EuroMotion

if nargin < 3
    names = {'lon_step','lon_prbs','lon_chirp','lat_prbs','lat_dlc','lat_slc'};
    fname = fullfile('data', sprintf('%s_1.csv', names{test_id}));
end
[d, ~, ~] = fileparts(fname);
if ~isempty(d) && ~exist(d, 'dir'), mkdir(d); end

s = serialport(port, 115200);
configureTerminator(s, "CR/LF");
cleanup = onCleanup(@() delete(s));
flush(s);
pause(0.2);
writeline(s, sprintf('X%d', test_id));

lines = strings(0, 1);
t0 = tic;
while toc(t0) < 90
    if s.NumBytesAvailable > 0
        L = readline(s);
        lines(end+1, 1) = L; %#ok<AGROW>
        if startsWith(L, "# END"), break; end
    else
        pause(0.01);
    end
end
writelines(lines, fname);
fprintf('Saved %d lines to %s\n', numel(lines), fname);
end
