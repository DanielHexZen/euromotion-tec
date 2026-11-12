function st = sim_lat_init(tr, p)
%SIM_LAT_INIT  Lateral VL53L1X S1 (left) and S2 (right), EXTI-timestamped (§7.13, D-5).
%#codegen
st.rng   = sim_rng_init(tr.seed, 5);
st.tb    = p.vl_tb_ms*1e-3;
st.imp   = p.vl_imp_ms*1e-3;
st.t_end = [0.0047 0.0147] + st.imp;
st.hx = zeros(64, 1);  st.hy = zeros(64, 1);  st.hth = zeros(64, 1);
st.ev  = zeros(16, 5);          % [t_us id r ok s_enc]
st.seq = 0;
end
