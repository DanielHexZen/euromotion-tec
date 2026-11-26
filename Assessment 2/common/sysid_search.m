function [S, theta, info, Phi, Y] = sysid_search(y, u, segI, segV, NA, NB, NK, nk_min, tol)
%SYSID_SEARCH  Structured ARX order / delay search (explicit LS for every candidate).
%   For every (na, nb, nk) the LS problem theta = Phi\Y is solved on the
%   identification segments segI; the model is then judged by
%     - one-step-ahead NRMSE fit on segI,
%     - free-run NRMSE fit on segI (selection criterion),
%     - free-run NRMSE fit on the validation segments segV (reported only, NOT used for selection),
%     - AIC and FPE of the one-step residuals (complexity-penalised).
%   Selection rule (parsimony): among stable candidates with nk >= nk_min whose
%   identification free-run fit is within `tol` percentage points of the best one,
%   take the one with the fewest parameters (na+nb), then the smallest nk.
%   S.tab columns: [na nb nk n_params ident_osa ident_free val_free AIC FPE stable]
tab = []; 
for na = NA
  for nb = NB
    for nk = NK
      rows = sysid_rows(segI, na, nb, nk);
      [th, ~, ~, inf] = arx_ls(y, u, na, nb, nk, rows);
      stable = all(abs(roots(inf.a)) < 1);
      [ym, yf, yo] = arx_eval(inf.a, inf.b, u, y, segI);
      if all(isfinite(yf)) && max(abs(yf)) < 1e3*max(abs(y))
          fi = fit_metrics(ym, yf).fit;
      else
          fi = -Inf;
      end
      mo = fit_metrics(ym, yo).fit;
      [ym2, yf2] = arx_eval(inf.a, inf.b, u, y, segV);
      if all(isfinite(yf2)) && max(abs(yf2)) < 1e3*max(abs(y)), fv = fit_metrics(ym2, yf2).fit; else, fv = -Inf; end
      tab(end+1,:) = [na nb nk na+nb mo fi fv inf.aic inf.fpe stable]; %#ok<AGROW>
    end
  end
end
ok = tab(:,3) >= nk_min & tab(:,10) == 1 & isfinite(tab(:,6));
best = max(tab(ok,6));
cand = find(ok & tab(:,6) >= best - tol);
[~, ord] = sortrows([tab(cand,4), tab(cand,3), -tab(cand,6)]);
isel = cand(ord(1));
na = tab(isel,1); nb = tab(isel,2); nk = tab(isel,3);
[theta, Phi, Y, info] = arx_ls(y, u, na, nb, nk, sysid_rows(segI, na, nb, nk));
[ym, yf] = arx_eval(info.a, info.b, u, y, segI); S.sel.ident_free = fit_metrics(ym, yf);
[ym, yf] = arx_eval(info.a, info.b, u, y, segV); S.sel.val_free = fit_metrics(ym, yf);
S.tab = tab; S.isel = isel; S.best_ident_free = best; S.tol = tol; S.nk_min = nk_min;
end
