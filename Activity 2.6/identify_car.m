%% identify_car.m -- Activity 2.6: plant model, PI tuning, continuous vs. discrete simulation
% MR2006B Industrial Automation, team EuroMotion
% Units: duty in %, speed in mm/s, position in mm, time in s.
% Needs the Control System Toolbox. Figures are saved as PDF next to this file.
clear; clc; close all;
outDir = fileparts(mfilename('fullpath'));
if isempty(outDir), outDir = pwd; end

%% 1. Identified plant (open-loop tests, BTS7960, wheels in the air)
% G(s) = K * exp(-L s) / (tau s + 1), input duty [%], output wheel speed [mm/s]
K    = 7.596;      % static gain [(mm/s)/%], slope of Vss(duty)
tau  = 0.050;      % time constant [s]
L    = 0;          % dead time of the plant [s]
u0   = 7.0;        % dead zone [%]
vMax = 680;        % highest measured speed at 100 % duty [mm/s]

% Sampling and speed measurement in the firmware
Ts    = 0.020;                % control period [s] (TIM6 interrupt, 50 Hz)
alpha = 0.5;                  % IIR speed filter
tauF  = -Ts/log(1 - alpha);   % equivalent filter time constant [s]
kx    = pi*72.6/3584;         % mm per encoder count (wheel 72.6 mm, x4 decoding)
theta = L + Ts + tauF;        % design delay: plant + sampling + filter [s]

s  = tf('s');
G  = K/(tau*s + 1);  G.InputDelay  = L;       % plant
Gd = G;              Gd.InputDelay = theta;   % design model (sampling as delay)

fprintf('=== IDENTIFIED MODEL (input duty %%, output speed mm/s) ===\n');
fprintf('  G(s) = K * exp(-L s) / (tau s + 1)\n');
fprintf('  K   = %.3f (mm/s)/%%   static gain (slope of Vss(duty))\n', K);
fprintf('  tau = %.0f ms   time constant\n', tau*1e3);
fprintf('  L   = %.0f ms   dead time of the plant\n', L*1e3);
fprintf('  design delay theta = L + Ts + filter = %.0f ms\n', theta*1e3);
fprintf('  dead zone u0 = %.1f %%\n', u0);
fprintf('  highest measured speed: %.0f mm/s at 100 %% duty\n\n', vMax);

%% 2. Feed-forward: duty = u0 + Kff * v (inverse of the static map)
Kff = 1/K;   % [% per mm/s]
fprintf('=== FEED-FORWARD ===\n');
fprintf('  DUTY_MIN_PCT    %.1ff\n', u0);
fprintf('  KFF_PCT_PER_MMS %.4ff\n', Kff);
fprintf('  -> terminal (both wheels):  F%.1f %.4f\n', u0, Kff);
fprintf('  -> terminal (wheel L only): FL9.8 0.1275   (own fit per wheel)\n');
fprintf('  -> terminal (wheel R only): FR4.1 0.1361\n\n');

%% 3. PI speed controller, SIMC rules (Skogestad)
% Kp = tau / (K (tau_c + theta)),  Ti = min(tau, 4 (tau_c + theta)),  Ki = Kp / Ti
names = {'tight    (tau_c = theta)  ', ...
         'balanced (tau_c = 2 theta)', ...
         'gentle   (tau_c = 4 theta)'};
tauC = [1 2 4]*theta;
KpS = zeros(1,3);  KiS = zeros(1,3);
tStep = (0:1e-3:1)';

fprintf('=== PI SPEED CONTROLLER (SIMC): Kp [%% per mm/s], Ki [%% per mm/s*s] ===\n');
figure('Name', 'SIMC tuning');  hold on;  grid on;
for i = 1:3
    KpS(i) = tau/(K*(tauC(i) + theta));
    Ti     = min(tau, 4*(tauC(i) + theta));
    KiS(i) = KpS(i)/Ti;
    T_i = feedback(pid(KpS(i), KiS(i))*Gd, 1);
    S   = stepinfo(T_i, 'SettlingTimeThreshold', 0.05);
    fprintf('  %s  Kp = %.3f  Ki = %.3f  Ti = %3.0f ms   overshoot %4.1f %%, settling (5%%) %4.0f ms\n', ...
            names{i}, KpS(i), KiS(i), Ti*1e3, S.Overshoot, S.SettlingTime*1e3);
    plot(tStep, step(T_i, tStep), 'LineWidth', 1.2);
end
xlabel('Time [s]');  ylabel('Speed / setpoint [-]');
title('Speed loop with PI, unit step (design model)');
legend(names, 'Location', 'southeast');
exportgraphics(gcf, fullfile(outDir, 'fig_simc_step.pdf'), 'ContentType', 'vector');

Kp = KpS(2);  Ki = KiS(2);   % balanced -> used on the car
fprintf('  -> balanced is used: Kp = %.3f, Ki = %.3f\n\n', Kp, Ki);

%% 4. Discretization (Ts = 20 ms): continuous vs. discrete speed loop
% Discrete PI as in the firmware (forward Euler):
%   u[k] = Kp e[k] + I[k-1],   I[k] = I[k-1] + Ki Ts e[k]
z  = tf('z', Ts);
Gx = c2d(G/s, Ts, 'zoh');                 % duty -> position, ZOH
Vm = minreal((z - 1)/(Ts*z)*Gx);          % speed from count difference (x[k]-x[k-1])/Ts
F  = alpha*z/(z - (1 - alpha));           % IIR speed filter
Cz = pid(Kp, Ki, 0, 0, Ts, 'IFormula', 'ForwardEuler');

Tc = feedback(pid(Kp, Ki)*Gd, 1);         % continuous (design model)
Tz = feedback(Cz*Vm, F);                  % discrete: ZOH, encoder, filter

Sc = stepinfo(Tc, 'SettlingTimeThreshold', 0.05);
Sz = stepinfo(Tz, 'SettlingTimeThreshold', 0.05);
fprintf('=== CONTINUOUS vs. DISCRETE (Ts = %.0f ms) ===\n', Ts*1e3);
fprintf('  discrete PI: Kp = %.3f, Ki*Ts = %.4f\n', Kp, Ki*Ts);
fprintf('  continuous: overshoot %4.1f %%, settling (5%%) %4.0f ms\n', Sc.Overshoot, Sc.SettlingTime*1e3);
fprintf('  discrete:   overshoot %4.1f %%, settling (5%%) %4.0f ms\n\n', Sz.Overshoot, Sz.SettlingTime*1e3);

[yc, tc] = step(Tc, 1);
[yz, tz] = step(Tz, 1);
figure('Name', 'Continuous vs. discrete');
plot(tc, yc, 'LineWidth', 1.2);  hold on;  grid on;
stairs(tz, yz, 'LineWidth', 1.2);
xlabel('Time [s]');  ylabel('Speed / setpoint [-]');
title(sprintf('Speed loop, continuous vs. discrete (T_s = %.0f ms)', Ts*1e3));
legend('Continuous', 'Discrete', 'Location', 'southeast');
exportgraphics(gcf, fullfile(outDir, 'fig_cont_vs_disc.pdf'), 'ContentType', 'vector');

%% 5. Distance with a trapezoidal speed profile (cascade as in the firmware)
% v_cmd = v_ref + sat(Kpx (x_ref - x)),  duty = FF(v_cmd) + Kff tau a_ref + PI(v_cmd - v)
D = 1000;  vNom = 300;  ta = 1;  td = 1;     % target [mm], speed [mm/s], ramp times [s]
c = struct('K', K, 'tau', tau, 'u0', u0, 'Kff', Kff, 'Kp', Kp, 'Ki', Ki, ...
           'Kpx', 2, 'vCorr', 100, 'Imax', 30, 'Ts', Ts, 'alpha', alpha, 'kx', kx);

[t, ref, xc, vc] = simCar(c, D, vNom, ta, td, false, 1.0);   % continuous controller
[~, ~,   xd, vd] = simCar(c, D, vNom, ta, td, true,  1.0);   % discrete controller

fprintf('=== DISTANCE %.0f mm, %.0f mm/s, ramps %.1f s / %.1f s (T = %.2f s) ===\n', ...
        D, vNom, ta, td, ref.T);
printResult('continuous',             t, ref, xc, vc, D);
printResult('discrete',               t, ref, xd, vd, D);
[~, ~, x8, v8] = simCar(c, D, vNom, ta, td, true, 0.8);
printResult('discrete, gain -20 %',   t, ref, x8, v8, D);
[~, ~, x12, v12] = simCar(c, D, vNom, ta, td, true, 1.2);
printResult('discrete, gain +20 %',   t, ref, x12, v12, D);
fprintf('\n');

figure('Name', 'Trapezoidal profile');
subplot(2,1,1);
plot(t, ref.v, 'k--', t, vc, t, vd, 'LineWidth', 1.2);  grid on;
ylabel('Speed [mm/s]');  legend('Reference', 'Continuous', 'Discrete', 'Location', 'north');
title(sprintf('Distance %.0f mm, %.0f mm/s, ramps %.1f s', D, vNom, ta));
subplot(2,1,2);
plot(t, ref.x, 'k--', t, xc, t, xd, 'LineWidth', 1.2);  grid on;
xlabel('Time [s]');  ylabel('Position [mm]');
legend('Reference', 'Continuous', 'Discrete', 'Location', 'southeast');
exportgraphics(gcf, fullfile(outDir, 'fig_trapezoid.pdf'), 'ContentType', 'vector');

%% 6. Values for the firmware
fprintf('=== VALUES FOR THE FIRMWARE ===\n');
fprintf('  KP_SPEED %.3ff   KI_SPEED %.3ff   (%%/(mm/s), %%/(mm/s*s))\n', Kp, Ki);
fprintf('  -> terminal:  G%.3f %.3f\n', Kp, Ki);
fprintf('  SI parameters: whl_kp_v = %.2f 1/(m/s), whl_ki_v = %.2f 1/m\n', 10*Kp, 10*Ki);
fprintf('  highest speed ~%.0f mm/s -> V_NOM_MAX about %.0f (80 %%, headroom for the controller)\n', ...
        vMax, floor(0.8*vMax/10)*10);
fprintf('  Valid for wheels in the air. Repeat a few short steps on the floor.\n');

%% Local functions
function [t, ref, x, v] = simCar(c, D, vNom, ta, td, discrete, gain)
% Simulates one wheel. Plant: first order with dead zone, gain factor 'gain'.
% discrete = false: controller runs every dt with the true speed (continuous).
% discrete = true:  controller runs every Ts with encoder counts, count
%                   difference and IIR filter, output held (ZOH).
dt = 1e-4;
ramps = (ta + td)/2;
if D >= vNom*ramps
    V = vNom;  T = D/V + ramps;
else
    V = D/ramps;  T = ta + td;       % triangle: nominal speed not reached
end
t = (0:dt:T + 1)';  n = numel(t);
ref.T = T;
ref.v = V*max(0, min(min(t/ta, 1), (T - t)/td));
ref.a = (t < ta)*V/ta - (t >= T - td & t < T)*V/td;
ref.x = cumtrapz(t, ref.v);

x = zeros(n, 1);  v = zeros(n, 1);
u = 0;  I = 0;  vf = 0;  xOld = 0;  nTs = round(c.Ts/dt);
for k = 1:n-1
    if ~discrete || mod(k - 1, nTs) == 0
        if discrete
            xm = floor(x(k)/c.kx)*c.kx;               % encoder counts -> mm
            vf = vf + c.alpha*((xm - xOld)/c.Ts - vf); % count difference + IIR
            xOld = xm;  vm = vf;  h = c.Ts;
        else
            xm = x(k);  vm = v(k);  h = dt;
        end
        vCmd = ref.v(k) + max(-c.vCorr, min(c.vCorr, c.Kpx*(ref.x(k) - xm)));
        if abs(vCmd) <= 2 && ref.v(k) == 0, I = 0; end   % reset at standstill
        e  = vCmd - vm;
        ff = 0;
        if abs(vCmd) > 2
            ff = sign(vCmd)*(c.u0 + c.Kff*abs(vCmd)) + c.Kff*c.tau*ref.a(k);
        end
        u = ff + c.Kp*e + I;
        if abs(u) < 100                                  % anti-windup
            I = max(-c.Imax, min(c.Imax, I + c.Ki*e*h));
        end
        u = max(-100, min(100, u));
    end
    ue = sign(u)*max(abs(u) - c.u0, 0);                  % dead zone
    v(k+1) = v(k) + (gain*c.K*ue - v(k))*(1 - exp(-dt/c.tau));
    x(k+1) = x(k) + (v(k) + v(k+1))/2*dt;
end
end

function printResult(name, t, ref, x, v, D)
i = find(abs(D - x) < 2, 1);
if isempty(i), tArr = NaN; else, tArr = t(i); end
fprintf('  %-22s final error %5.2f mm, max position error %5.2f mm, max speed error %5.1f mm/s, within 2 mm at %.2f s\n', ...
        name, D - x(end), max(abs(ref.x - x)), max(abs(ref.v - v)), tArr);
end
