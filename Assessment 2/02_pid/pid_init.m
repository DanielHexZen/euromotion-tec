function st = pid_init(e0)
%PID_INIT  Initial state of pid_step (integrator, derivative filter, previous error).
%   e0 : error at the first sample (avoids a derivative kick); default 0.
%#codegen
if nargin < 1, e0 = 0; end
st = struct('I', 0, 'ud', 0, 'eprev', e0, 'sat', 0, 'uun', 0);
end
