function h = lkf_newfig(w, hgt)
%LKF_NEWFIG  Invisible figure of w x hgt inches (IEEE column = 3.5 in), 8 pt fonts.
h = figure('visible','off','color','w');
set(h,'paperunits','inches','papersize',[w hgt],'paperposition',[0 0 w hgt]);
set(h,'defaultaxesfontsize',8,'defaulttextfontsize',8,'defaultlinelinewidth',1);
end
