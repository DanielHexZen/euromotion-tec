function st = sim_gnd_init(tr, c, p)
%SIM_GND_INIT  Ground sensors G1..G4 (VL53L1X downward) and the gnd debounce (§7.13, §7.14).
%   Sensor index 1..4 = G1 left outer, G2 left inner, G3 right inner, G4 right outer.
%#codegen
NS = numel(c.type);
st.rng = sim_rng_init(tr.seed, 3);
st.rs  = [p.gnd_x  p.gnd_x  p.gnd_x  p.gnd_x;
          p.gnd_y_out p.gnd_y_in -p.gnd_y_in -p.gnd_y_out];
% raggedness of the true boundary: 8 sinusoids per segment and side
st.wob_a  = tr.sig_wobble/2;
st.wob_l  = zeros(NS, 2, 8);
st.wob_ph = zeros(NS, 2, 8);
rr = sim_rng_init(tr.seed, 4);
for k = 1:NS
    for sd = 1:2
        for j = 1:8
            [rr, u1] = sim_rand(rr);  [rr, u2] = sim_rand(rr);
            st.wob_l(k, sd, j)  = tr.wob_lmin*(tr.wob_lmax/tr.wob_lmin)^u1;
            st.wob_ph(k, sd, j) = 2*pi*u2;
        end
    end
end
% truth pose history (1 ms) for the mid-window footprint
st.hx = zeros(64, 1);  st.hy = zeros(64, 1);  st.hth = zeros(64, 1);
% ranging and polling
tb = p.vl_tb_ms*1e-3;  imp = p.vl_imp_ms*1e-3;
st.tb = tb;  st.imp = imp;  st.poll = p.vl_poll_ms*1e-3;
st.t_end    = 0.0023 + (0:3)*0.005 + imp;     % first integration window ends
st.poll_off = 0.0011 + (0:3)*0.00125;
st.pend     = zeros(1, 4);
st.t_deliv  = zeros(1, 4);
st.s_cls    = zeros(1, 4);
st.s_tmeas  = zeros(1, 4);
% debounce (gnd_n_deb)
st.pub    = -ones(1, 4);                      % -1: no published class yet
st.cand   = zeros(1, 4);
st.cand_n = zeros(1, 4);
st.cand_t = zeros(1, 4);
% event ring [t_us id from to d_ovr has_ovr]
st.ev   = zeros(16, 6);
st.seq  = 0;
% periodic mode
st.s_trav = 0;
st.next_s = tr.gnd_period_m;
st.alt    = 0;
st.pq     = zeros(4, 7);                      % [active t_deliv t_us id d_ovr 0 0]
st.lastx  = c.start(1) + tr.start_err(1);
st.lasty  = c.start(2) + tr.start_err(2);
st.n_samples = 0;
end
