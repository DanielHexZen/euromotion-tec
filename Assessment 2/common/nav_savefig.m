function nav_savefig(f, file, w, h)
%NAV_SAVEFIG  Save figure f as PNG, 200 dpi, w x h inches (IEEE single column: 3.5 in).
set(f, 'paperunits', 'inches', 'papersize', [w h], 'paperposition', [0 0 w h]);
print(f, file, '-dpng', '-r200');
close(f);
end
