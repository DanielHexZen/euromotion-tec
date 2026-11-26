function blk = nav_sl_block(mdl, name, script, outsizes, params, pos)
%NAV_SL_BLOCK  Add a MATLAB Function block with the given script to the Simulink model mdl.
%   outsizes : cell array of {name, size-string} for the output data (set explicitly);
%   params   : cell array of base-workspace variable names made available as Parameter-scope
%              data (same name inside the function body);  pos : [left top right bottom].
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = script;
for i = 1:size(outsizes,1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', outsizes{i,1});
    d.Props.Array.Size = outsizes{i,2};
    d.DataType = 'double';
end
for i = 1:numel(params)
    d = Stateflow.Data(ch);
    d.Name = params{i};
    d.Scope = 'Parameter';
end
end
