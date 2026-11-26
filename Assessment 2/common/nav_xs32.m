function x = nav_xs32(x)
%NAV_XS32  One xorshift32 step on a double holding a 32-bit integer.
M32 = 4294967296;
x = bitxor(x, mod(bitshift(x, 13), M32));
x = bitxor(x, bitshift(x, -17));
x = bitxor(x, mod(bitshift(x, 5), M32));
end
