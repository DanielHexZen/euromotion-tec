function s = mat2str_c(z)
%MAT2STR_C  Compact text of a (possibly empty / complex) vector.
if isempty(z), s = 'none'; return; end
s = '';
for i = 1:numel(z)
    if abs(imag(z(i))) < 1e-12, s = [s sprintf('%.6f  ', real(z(i)))]; %#ok<AGROW>
    else, s = [s sprintf('%.6f%+.6fi  ', real(z(i)), imag(z(i)))]; end %#ok<AGROW>
end
end
