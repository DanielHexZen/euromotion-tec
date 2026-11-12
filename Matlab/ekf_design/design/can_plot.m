function ok = can_plot()
%CAN_PLOT  True if figures can be created and printed here (MATLAB normally).
%
%   Headless Octave: text rendering fails ("ft_text_renderer: unable to load
%   appropriate font") when Octave's own font directory is missing, as in the
%   Debian/Ubuntu package without octave fonts. In that case the FreeSans fonts
%   of the system (package fonts-freefont-ttf or -otf) are linked into
%   results/octave_fonts under the names Octave looks for and OCTAVE_FONTS_DIR
%   is pointed there before the first figure is created; the gnuplot toolkit
%   then writes the PNG files (no display needed).
persistent cached
if ~isempty(cached), ok = cached; return; end
ok = false;
if exist('OCTAVE_VERSION', 'builtin') > 0
    octave_fonts();
end
try
    f = figure('Visible', 'off');
    plot(1:2, 1:2);  title('x');
    tmp = [tempname() '.png'];
    print(f, tmp, '-dpng');
    close(f);
    ok = exist(tmp, 'file') == 2;
    if ok, delete(tmp); end
catch
    try, close all; catch, end %#ok<NOCOM>
end
cached = ok;
end

function octave_fonts()
% Octave only: provide FreeSans*.otf if Octave has no font directory of its own.
if ~isempty(getenv('OCTAVE_FONTS_DIR')), return; end
try
    d0 = __octave_config_info__('octfontsdir');
catch
    d0 = '';
end
if ~isempty(d0) && exist(fullfile(d0, 'FreeSans.otf'), 'file'), return; end
src = {'/usr/share/fonts/opentype/freefont', '/usr/share/fonts/truetype/freefont', ...
       '/usr/share/fonts/gnu-free', '/usr/share/fonts/TTF', '/usr/share/fonts/OTF'};
ext = {'.otf', '.ttf'};
var = {'', 'Bold', 'Oblique', 'BoldOblique'};
dst = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'octave_fonts');
for i = 1:numel(src)
    for j = 1:2
        if exist(fullfile(src{i}, ['FreeSans' ext{j}]), 'file')
            if ~exist(dst, 'dir'), mkdir(dst); end
            for k = 1:numel(var)
                s = fullfile(src{i}, ['FreeSans' var{k} ext{j}]);
                t = fullfile(dst, ['FreeSans' var{k} '.otf']);
                if exist(s, 'file') && ~exist(t, 'file')
                    copyfile(s, t);
                end
            end
            setenv('OCTAVE_FONTS_DIR', dst);
            return;
        end
    end
end
end
