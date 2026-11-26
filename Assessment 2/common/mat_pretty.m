function s = mat_pretty(M, fmt)
%MAT_PRETTY  Format a numeric matrix as aligned text rows, e.g. for Markdown code blocks.
if nargin < 2, fmt = '%14.10f'; end
M(M == 0) = 0;                       % avoid printing negative zeros
s = '';
for i = 1:size(M,1)
    row = '';
    for j = 1:size(M,2), row = [row sprintf(fmt, M(i,j))]; end %#ok<AGROW>
    s = [s '    [' row ' ]' char(10)]; %#ok<AGROW>
end
end
