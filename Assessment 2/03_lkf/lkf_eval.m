function R = lkf_eval(par, C)
%LKF_EVAL  Run LKF + raw integrations on a case and compute metrics.
%   RMSE_x = sqrt(mean(ex^2 + ey^2)) (2-D position error vs reference).
N = numel(C.t); idx = 1:N;
x0 = [C.x0; zeros(par.nx-4,1)];
if size(par.H,1) == 4, zz = [C.z; C.zvel]; zvv = C.zvv; else, zz = C.z; zvv = C.zv; end
out  = lkf_run(par, C.u, zz, zvv, x0, par.P0, C.psi);
Xraw = lkf_raw(par, C.u_raw, C.x0);      % plain accelerometers, no bias removal
Xrb  = lkf_raw(par, C.u, C.x0);          % bias-corrected accelerometers
m_l = lkf_metrics(out.X, C.ref, out.Pd, idx);
m_r = lkf_metrics(Xraw, C.ref, [], idx);
m_b = lkf_metrics(Xrb, C.ref, [], idx);
fx = find(C.zv);
ix = find(C.zv); cnt = cumsum(C.zv(:)'); zh = C.z(:, ix(max(cnt,1)));   % zero-order hold of the fixes
m_h = lkf_metrics(zh, C.ref, [], idx);
R.rmse_zoh = m_h.rmse; R.red_zoh = 100*(1 - m_l.rmse/m_h.rmse);
m_z = lkf_metrics(C.z(:,fx), C.ref(:,fx), [], 1:numel(fx));   % raw fixes (at fix instants)
m_lf = lkf_metrics(out.X(:,fx), C.ref(:,fx), [], 1:numel(fx));
R.out = out; R.Xraw = Xraw; R.Xrb = Xrb;
R.rmse_lkf = m_l.rmse; R.rmse_raw = m_r.rmse; R.rmse_rawbc = m_b.rmse;
R.rmse_fix = m_z.rmse; R.rmse_lkf_atfix = m_lf.rmse;
R.max_lkf = m_l.max; R.max_raw = m_r.max;
R.err_lkf = m_l.err; R.cover3 = m_l.cover3;
R.red_raw   = 100*(1 - R.rmse_lkf/R.rmse_raw);
R.red_rawbc = 100*(1 - R.rmse_lkf/R.rmse_rawbc);
R.red_fix   = 100*(1 - R.rmse_lkf/R.rmse_fix);
R.red_fix_atfix = 100*(1 - R.rmse_lkf_atfix/R.rmse_fix);
if isfield(C,'vref')
    ev = out.X([2 4],:) - C.vref;
    R.rmse_vel = sqrt(mean(sum(ev.^2,1)));
end
end
