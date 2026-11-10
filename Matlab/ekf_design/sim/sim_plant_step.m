function [st, y] = sim_plant_step(st, u, tr)
%SIM_PLANT_STEP  1 kHz truth plant: closed speed loops as first-order lags.
%
%   [st, y] = sim_plant_step(st, u, tr)
%   u = [refL; refR] wheel speed references in the encoder frame (m/s), held
%   over the next 1 ms (the output of drv, delayed by one tick).
%
%   Model:
%     w' = (ref - w)/tau_w               exact ZOH solution over h = 1 ms
%     ground distance  = k_side*(1 - slip)*encoder distance
%                        k_L = scale*(1 - eps/2), k_R = scale*(1 + eps/2)
%     yaw              = (g_R - g_L)/b_true      (b_true != odo_wheelbase)
%   The output is the state at t_k BEFORE the update, plus the inputs and
%   ground coefficients of [t_k, t_k+1), so that the IMU model can evaluate the
%   exact true yaw rate at any instant inside the tick.
%
%   y = [t; x; y; th; dL; dR; wL; wR; refL; refR; cL; cR; k]   (13 x 1)
%#codegen
h  = 1e-3;
cL = tr.kL;  cR = tr.kR;
for i = 1:tr.n_slip
    if st.t >= tr.slip_t0(i) && st.t < tr.slip_t1(i)
        if tr.slip_side(i) == 1 || tr.slip_side(i) == 3, cL = cL*(1 - tr.slip_frac(i)); end
        if tr.slip_side(i) == 2 || tr.slip_side(i) == 3, cR = cR*(1 - tr.slip_frac(i)); end
    end
end
y = [st.t; st.x; st.y; st.th; st.dL; st.dR; st.wL; st.wR; u(1); u(2); cL; cR; st.k];
st.ylast = y;

a    = exp(-h/tr.tau_w);
incL = u(1)*h + (st.wL - u(1))*tr.tau_w*(1 - a);
incR = u(2)*h + (st.wR - u(2))*tr.tau_w*(1 - a);
st.wL = u(1) + (st.wL - u(1))*a;
st.wR = u(2) + (st.wR - u(2))*a;
gL = cL*incL;  gR = cR*incR;
ds  = 0.5*(gL + gR);
dth = (gR - gL)/tr.b_true;
st.x  = st.x + ds*cos(st.th + 0.5*dth);
st.y  = st.y + ds*sin(st.th + 0.5*dth);
st.th = st.th + dth;               % unwrapped truth heading
st.dL = st.dL + incL;
st.dR = st.dR + incR;
st.t  = st.t + h;
st.k  = st.k + 1;
end
