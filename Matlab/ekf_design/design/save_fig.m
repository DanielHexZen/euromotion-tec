function save_fig(f, name)
%SAVE_FIG  Save a figure as PNG under Matlab/ekf_design/results/ (created if needed).
d = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~exist(d, 'dir'), mkdir(d); end
try
    print(f, fullfile(d, [name '.png']), '-dpng', '-r110');
catch err
    fprintf('  (figure %s not saved: %s)\n', name, err.message);
end
close(f);
end
