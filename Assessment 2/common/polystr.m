function s = polystr(c, var)
%POLYSTR  Text of a polynomial c(1) + c(2) var^1 + ... (var given as 'z^-' -> z^-1, z^-2 ...).
s = '';
for i = 1:numel(c)
    p = i - 1;
    if p == 0, term = sprintf('%.6f', abs(c(i)));
    elseif p == 1, term = sprintf('%.6f %s1', abs(c(i)), var);
    else, term = sprintf('%.6f %s%d', abs(c(i)), var, p); end
    if i == 1
        if c(i) < 0, s = ['-' term]; else, s = term; end
    else
        if c(i) < 0, s = [s ' - ' term]; else, s = [s ' + ' term]; end %#ok<AGROW>
    end
end
end
