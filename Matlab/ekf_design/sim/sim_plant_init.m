function st = sim_plant_init(tr, course)
%SIM_PLANT_INIT  Truth state of the differential-drive vehicle at t = 0.
%#codegen
st.t  = 0;
st.k  = 0;
st.x  = course.start(1) + tr.start_err(1);
st.y  = course.start(2) + tr.start_err(2);
st.th = course.start(3) + tr.start_err(3);
st.dL = 0;  st.dR = 0;            % wheel travel in the ENCODER frame (nominal m)
st.wL = 0;  st.wR = 0;            % wheel speed in the encoder frame, m/s
st.ylast = zeros(13, 1);          % last output (held for the IMU samples)
end
