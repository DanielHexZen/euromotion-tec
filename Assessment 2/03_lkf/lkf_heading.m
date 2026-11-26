function psi = lkf_heading(wz, psi_fix, fix_valid, psi0, gyro_bias, T, Kc)
%LKF_HEADING  Complementary filter for the heading used to rotate the accelerometers.
%   Predict by integrating the bias-corrected gyro, correct with the (low-rate,
%   noisy) heading fix:   psi = psi_pred + Kc*(psi_fix - psi_pred).
%   Run OUTSIDE the LKF (the LKF input is the already rotated acceleration).
N = numel(wz);
psi = zeros(N,1);
psi(1) = psi0;
for k = 2:N
    p = psi(k-1) + (wz(k-1) - gyro_bias)*T;
    if fix_valid(k)
        p = p + Kc*(psi_fix(k) - p);
    end
    psi(k) = p;
end
end
