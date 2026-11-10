function [s, u] = sim_rand(s)
%SIM_RAND  Uniform deviate in (0, 1) from the combined LCG (see sim_rng_init).
%#codegen
s(1) = mod(40014*s(1), 2147483563);
s(2) = mod(40692*s(2), 2147483399);
z = mod(s(1) - s(2), 2147483562);
if z == 0
    z = 2147483562;
end
u = z/2147483563;
end
