function y = sim_yaw_pack(r)
%SIM_YAW_PACK  Yaw ring as one double vector [t; psi; rate; head; n] (signal between blocks).
%   Lossless also for single (single -> double -> single is exact).
%#codegen
M = numel(r.t);
y = zeros(3*M + 2, 1);
y(1:M)       = r.t;
y(M+1:2*M)   = double(r.psi);
y(2*M+1:3*M) = double(r.rate);
y(3*M+1)     = r.head;
y(3*M+2)     = r.n;
end
