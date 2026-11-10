function y = sim_enc_step(yp, p)
%SIM_ENC_STEP  1 kHz encoder snapshot (enc_snapshot_isr, §7.6): quantised counts + time.
%   counts = floor(encoder-frame distance / odo_m_per_cnt) (real resolution,
%   3584 counts per wheel revolution); t_us = TIM2 time of the tick.
%   y = [cntL; cntR; t_us]
%#codegen
y = [floor(yp(5)/p.odo_m_per_cnt); floor(yp(6)/p.odo_m_per_cnt); round(yp(1)*1e6)];
end
