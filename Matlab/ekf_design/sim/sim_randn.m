function [s, z] = sim_randn(s)
%SIM_RANDN  Standard normal deviate (Box-Muller, cosine branch) from the combined LCG.
%#codegen
[s, u1] = sim_rand(s);
[s, u2] = sim_rand(s);
z = sqrt(-2*log(u1))*cos(2*pi*u2);
end
