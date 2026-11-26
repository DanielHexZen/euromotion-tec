function lkf_savefig(h, file)
%LKF_SAVEFIG  PNG, 200 dpi.
print(h, '-dpng', '-r200', file);
close(h);
end
