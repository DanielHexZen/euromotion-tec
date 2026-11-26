function [y, u, uun] = pid_linsim(num, den, g, r, uff, gfun)
%PID_LINSIM  Closed-loop simulation of the difference equations (own implementation).
%   Plant G(z)=num(z)/den(z) (descending powers, strictly proper, den monic) is simulated as
%     y(k) = -den(2:end)*[y(k-1..k-n)] + num_tail*[u(k-1..k-n)]
%   (num padded to the length of den). PID: pid_step (saturation + anti-windup per g).
%   r   : reference sequence (already prefiltered if desired), column
%   uff : feedforward sequence (optional, default zeros)
%   gfun: optional handle gk = gfun(k, y_prev) returning the gain struct used at sample k
%         (speed scheduling / time-varying limits); default constant g.
%   Returns y, saturated u, unsaturated uun (PID + ff before the limit).
N = numel(r); n = numel(den) - 1;
if nargin < 5 || isempty(uff), uff = zeros(N,1); end
if nargin < 6, gfun = []; end
nb = zeros(1, n+1); nb(end-numel(num)+1:end) = num;     % pad numerator
y = zeros(N,1); u = zeros(N,1); uun = zeros(N,1);
st = pid_init(0);
for k = 1:N
    acc = 0;
    for i = 1:n
        if k-i >= 1
            acc = acc - den(i+1)*y(k-i) + nb(i+1)*u(k-i);
        end
    end
    y(k) = acc;            % strictly proper: nb(1) = 0 (no direct feed-through)
    if isempty(gfun), gk = g; else, gk = gfun(k, y(k)); end
    e = r(k) - y(k);
    [u(k), st] = pid_step(e, st, gk, uff(k));
    uun(k) = st.uun;
end
end
