function s = sim_rng_init(seed, stream)
%SIM_RNG_INIT  State of the deterministic PRNG used by every noise source.
%
%   s = sim_rng_init(seed, stream) returns the 2x1 state of a combined
%   multiplicative linear congruential generator (P. L'Ecuyer, "Efficient and
%   portable combined random number generators", CACM 31(6), 1988):
%     s1 <- 40014*s1 mod 2147483563,  s2 <- 40692*s2 mod 2147483399
%   period ~2.3e18. All products are below 2^53, so the generator is exact
%   in double and gives bit-identical sequences in MATLAB, Octave, generated
%   code (Simulink MATLAB Function blocks) and C. Each noise source has its own
%   stream so that switching one source on or off leaves the others unchanged.
%#codegen
a  = mod(seed*7919 + stream*104729 + 12345, 2147483562) + 1;
b  = mod(seed*15485863 + stream*32452843 + 54321, 2147483398) + 1;
s  = [a; b];
for k = 1:5                               % warm-up
    [s, ~] = sim_rand(s);
end
end
