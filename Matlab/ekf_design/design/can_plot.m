function ok = can_plot()
%CAN_PLOT  True if figures can be created and printed here (MATLAB normally;
%   headless Octave often cannot render text and returns false).
persistent cached
if ~isempty(cached), ok = cached; return; end
ok = false;
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
