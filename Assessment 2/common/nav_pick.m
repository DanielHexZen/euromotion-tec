function s = nav_pick(g, names)
%NAV_PICK  Sub-struct with the listed fields of g.
s = struct();
for i = 1:numel(names)
    s.(names{i}) = g.(names{i});
end
end
