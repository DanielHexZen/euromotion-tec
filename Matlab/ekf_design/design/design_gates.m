function [R, out] = design_gates(run_sims)
%DESIGN_GATES  Choice of the innovation gates ekf_gate1 / ekf_gate2.
%
%   [R, out] = design_gates(run_sims)
%   1. False rejection of a consistent measurement: 1 - F_chi2(gate).
%   2. Probability of rejecting a k-sigma outlier (scalar: |N(k,1)| > sqrt(gate)).
%   3. Expected number of false rejections on the course: edge events per run
%      times the false-rejection probability.
%   4. (run_sims) t_ekf_line with 10 % false edge events (offset 0.1..0.3 m):
%      gate at the spec value versus no gate (gate1 = 1e9).
if nargin < 1, run_sims = true; end
p = ekf_params();
g1 = p.ekf_gate1;  g2 = p.ekf_gate2;
fprintf('\n=== Gates ===\n');
fprintf('false rejection: scalar gate %.2f -> %.3f %%, 2-D gate %.2f -> %.3f %%\n', g1, ...
    100*(1 - chi2_cdf(g1, 1)), g2, 100*(1 - chi2_cdf(g2, 2)));
Phi = @(z) 0.5*erfc(-z/sqrt(2));
ks = [2 3 4 5 6 8 10];
fprintf('scalar outlier of k sigma (sigma = sqrt(S)) rejected with probability:\n');
for k = ks
    pr = Phi(-sqrt(g1) - k) + 1 - Phi(sqrt(g1) - k);
    out.pk(k) = pr;
    fprintf('   k = %2d: %.3f\n', k, pr);
end
fprintf(['note: the gate measures k in units of sqrt(S), not of ekf_sig_edge. After a long edge-free\n' ...
         '  stretch S is dominated by P, so a false event 0.2 m off is accepted when the cross-track\n' ...
         '  sigma exceeds ~0.2/3 = 7 cm; the first edge after such a stretch is the vulnerable one.\n']);
R = vres('gate1 false-rejection probability', -1, sprintf('%.3f %%', 100*(1 - chi2_cdf(g1, 1))), 'design');
R = [R, vres('gate1 rejects a 5-sigma outlier with', -1, sprintf('%.3f', out.pk(5)), 'design')];
if run_sims
    for gate = [g1 1e9]
        pp = p;  pp.ekf_gate1 = gate;
        cfg = sim_config('line40', 9, 'truth', 'spec_line', 'gnd', 2, 'p', pp);
        cfg.tr.gnd_outlier_p = 0.10;
        L = sim_run(cfg);
        M = sim_metrics(L);
        ct = M.e_ct(M.i0:end);  ct = ct(~isnan(ct));
        fprintf('t_ekf_line + 10 %% false edges, gate1 = %g: ct max %.1f mm rms %.1f mm, rejected %d of %d\n', ...
            gate, 1e3*max(abs(ct)), 1e3*sqrt(mean(ct.^2)), M.n_edge_rej, M.n_edge_rej + M.n_edge_acc);
        out.sim(1 + (gate > 100)) = struct('ct_max', max(abs(ct)), 'rej', M.n_edge_rej, 'acc', M.n_edge_acc);
    end
    R = [R, vres('10 % false edges: ct max with / without gate', double(out.sim(1).ct_max < 0.03), ...
        sprintf('%.1f / %.1f mm', 1e3*out.sim(1).ct_max, 1e3*out.sim(2).ct_max), 'with gate < 30 mm')];
end
end
