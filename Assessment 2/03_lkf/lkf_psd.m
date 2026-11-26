function [f, Pxx] = lkf_psd(x, fs, nseg)
%LKF_PSD  One-sided Welch PSD (Hann window, 50 % overlap), no toolbox needed.
%   Pxx in (unit^2)/Hz. White noise of std s sampled at fs gives Pxx = 2 s^2/fs.
x = x(:) - mean(x);
if nargin < 3, nseg = 256; end
w = 0.5 - 0.5*cos(2*pi*(0:nseg-1)'/nseg);       % Hann
U = sum(w.^2);
step = nseg/2;
nwin = floor((numel(x)-nseg)/step) + 1;
acc = zeros(nseg/2+1,1);
for i = 1:nwin
    seg = x((i-1)*step + (1:nseg)) .* w;
    X = fft(seg);
    p = abs(X(1:nseg/2+1)).^2 / (fs*U);
    p(2:end-1) = 2*p(2:end-1);
    acc = acc + p;
end
Pxx = acc/nwin;
f = (0:nseg/2)'*fs/nseg;
end
