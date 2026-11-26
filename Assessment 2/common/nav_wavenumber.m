function kf = nav_wavenumber(N, d)
%NAV_WAVENUMBER  Angular wavenumbers [rad/m] in FFT order for N periodic samples of spacing d.
if mod(N,2) == 0
    k = [0:N/2, -N/2+1:-1]';
else
    k = [0:(N-1)/2, -(N-1)/2:-1]';
end
kf = k*(2*pi/(N*d));
end
