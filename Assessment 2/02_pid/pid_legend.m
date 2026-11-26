function pid_legend(h, labels, loc)
%PID_LEGEND  Colour-coded text legend (workaround for the gnuplot toolkit, whose legend swatches
%   lose their line colours). h: line/patch handles, labels: cell array, loc: 'northeast' etc.
%   Each entry is a short coloured line sample followed by the label.
ax = gca; h = h(:).'; n = numel(h); dy = 0.09;
if any(strcmp(loc, {'northwest', 'southwest', 'west'})), x = 0.04; else, x = 0.40; end
if strncmp(loc, 'north', 5), y0 = 0.93; elseif strncmp(loc, 'south', 5), y0 = 0.06 + (n-1)*dy; else, y0 = 0.5 + (n-1)*dy/2; end
for i = 1:n
    if ~ishandle(h(i)), continue; end
    switch get(h(i), 'type')
        case 'patch', col = 0.6*get(h(i), 'facecolor'); sym = '##';
        otherwise
            col = get(h(i), 'color'); ls = get(h(i), 'linestyle');
            switch ls, case ':', sym = '....'; case '--', sym = '- - -'; case '-.', sym = '-.-.'; otherwise, sym = '----'; end
    end
    text(x, y0 - (i-1)*dy, [sym ' ' labels{i}], 'units', 'normalized', 'color', col, 'fontsize', 6, 'parent', ax);
end
end
