function runs = id_read_log(files)
%ID_READ_LOG  Parse calibration-firmware logs into a struct array of complete runs.
%   Lines that do not match the log grammar (screen noise, prompts) are ignored.
%   A run without "DONE X" is truncated: warning 'id_read_log:truncated', run dropped.
if ischar(files) || isstring(files), files = cellstr(files); end
runs = struct('profile', {}, 'base', {}, 'diff', {}, 'm_per_cnt_l', {}, 'm_per_cnt_r', {}, ...
    'b', {}, 'Ts', {}, 't', {}, 'uL', {}, 'uR', {}, 'dL', {}, 'dR', {}, 'file', {});
for i = 1:numel(files)
    lines = splitlines(fileread(files{i}));
    cur = [];  rows = zeros(0, 5);
    for j = 1:numel(lines)
        s = strtrim(lines{j});                       % also strips the CR of CRLF
        tok = regexp(s, '^# X ([SPNLD]) base_pct=(-?\d+) diff_pct=(-?\d+)', 'tokens', 'once');
        if ~isempty(tok)
            if ~isempty(cur), trunc(files{i}, cur.profile); end
            cur = struct('profile', tok{1}, 'base', str2double(tok{2}), 'diff', str2double(tok{3}), ...
                'par', []);
            rows = zeros(0, 5);
            continue
        end
        if isempty(cur), continue, end
        tok = regexp(s, '^# X par m_per_cnt_l_nm=(\d+) m_per_cnt_r_nm=(\d+) wheelbase_um=(\d+) hz=(\d+)', 'tokens', 'once');
        if ~isempty(tok)
            cur.par = str2double(tok);
            continue
        end
        tok = regexp(s, '^X,(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)$', 'tokens', 'once');
        if ~isempty(tok)
            rows(end+1, :) = str2double(tok); %#ok<AGROW>
        elseif strcmp(s, 'DONE X')
            if isempty(cur.par) || isempty(rows)
                warning('id_read_log:badrun', '%s: run %s has no "# X par" line or no data, skipped.', files{i}, cur.profile);
            else
                runs(end+1) = finish(cur, rows, files{i}); %#ok<AGROW>
            end
            cur = [];
        end
    end
    if ~isempty(cur), trunc(files{i}, cur.profile); end
end
end

function trunc(file, p)
warning('id_read_log:truncated', '%s: run %s has no "DONE X" (truncated), skipped.', file, p);
end

function r = finish(cur, rows, file)
r.profile = cur.profile;  r.base = cur.base;  r.diff = cur.diff;
r.m_per_cnt_l = cur.par(1) * 1e-9;           % nm -> m
r.m_per_cnt_r = cur.par(2) * 1e-9;
r.b  = cur.par(3) * 1e-6;                    % um -> m
r.Ts = 1 / cur.par(4);
r.t  = rows(:, 1) * 1e-3;
r.uL = rows(:, 2) * 1e-3;
r.uR = rows(:, 3) * 1e-3;
r.dL = rows(:, 4);
r.dR = rows(:, 5);
r.file = file;
end
