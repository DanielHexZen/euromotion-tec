function pst = pid_prefilt_init(r0)
%PID_PREFILT_INIT  Initial state of pid_prefilt (output history set to the steady value r0).
%#codegen
if nargin < 1, r0 = 0; end
pst = struct('r1', r0, 'r2', r0);
end
