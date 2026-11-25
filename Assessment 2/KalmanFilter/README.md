# Diskreter Kalman-Filter (EKF) für die autonome Navigation – openloop_test01

Fahrzeug: F1TENTH-Ackermann-Fahrzeug „McQueen56“ (Radstand L = 0,324 m).
Abtastzeit: **Ts = 20 ms** (50 Hz, Rate von VESC und IMU).

## Dateien

| Datei | Zweck |
|---|---|
| `main_kalman.m` | Hauptskript: Daten laden, Parameter identifizieren, Filter auslegen, 4 Konfigurationen vergleichen, Plots, `ekf_params.mat` speichern |
| `ekf_load_data.m` | lädt `openloop_test01_synced.mat`, dezimiert 100 Hz → 50 Hz, ZOH für Befehle, Gültigkeitsflags für Sensoren |
| `ekf_identify.m` | Systemidentifikation aus den Steuerbefehlen und Sensordaten |
| `ekf_step.m` | **ein** Filterschritt (Prädiktion + Korrektur), codegen-fähig – wird von MATLAB *und* Simulink genutzt |
| `ekf_param_vector.m` | packt die Parameter in einen Vektor (Eingang für Simulink) |
| `ekf_run.m`, `ekf_align_odometry.m` | Filterlauf über den Datensatz, Vergleich mit der Rad-Odometrie |
| `build_ekf_simulink.m` | baut `ekf_navigation.slx`, simuliert und vergleicht mit dem MATLAB-Ergebnis |

Reihenfolge: `main_kalman` → `build_ekf_simulink` (beide im Ordner `KalmanFilter` starten; die Daten werden aus `../openloop_test01/` geladen).

## Modell (diskret, kinematisches Einspurmodell mit Aktordynamik)

Zustand x = [p_x, p_y, ψ, v, δ, b_g]ᵀ, Eingang u = [v_cmd, δ_cmd]ᵀ

```
p_x(k+1) = p_x(k) + Ts·v(k)·cos ψ(k)
p_y(k+1) = p_y(k) + Ts·v(k)·sin ψ(k)
ψ(k+1)   = ψ(k)   + Ts·k_δ·v(k)/L·tan δ(k)
v(k+1)   = a_v·v(k) + (1−a_v)·v_cmd(k)        a_v = exp(−Ts/τ_v)
δ(k+1)   = a_δ·δ(k) + (1−a_δ)·δ_cmd(k)        a_δ = exp(−Ts/τ_δ)
b_g(k+1) = b_g(k)                              (Random Walk über Q)
```

Weil Kinematik und Messgleichung nichtlinear sind (cos ψ, tan δ), wird der
Kalman-Filter als **Extended Kalman Filter** ausgeführt: F = ∂f/∂x und
H = ∂h/∂x werden in jedem Schritt am aktuellen Schätzwert linearisiert.

Messungen (sequenzielle Korrektur, nur wenn gültig):

| Messung | h(x) | R |
|---|---|---|
| Radgeschwindigkeit VESC | v | (0,02 m/s)² |
| IMU-Gierrate (× k_g) | k_δ·v/L·tan δ + b_g | σ_g² aus Stillstand |
| Absolutpose (Lokalisierung, 1 Hz) | [p_x, p_y, ψ] | (0,05 m)², (0,03 rad)² |

Filtergleichungen je Schritt k:

```
Prädiktion:  x⁻ = f(x, u),   P⁻ = F P Fᵀ + Q
Korrektur:   K = P⁻Hᵀ(H P⁻ Hᵀ + R)⁻¹,  x = x⁻ + K(z − h(x⁻)),
             P = (I−KH) P⁻ (I−KH)ᵀ + K R Kᵀ      (Joseph-Form)
```

## Identifizierte Parameter (aus openloop_test01)

| Parameter | Wert |
|---|---|
| τ_v Antrieb | 0,278 s |
| τ_δ Lenkservo | 0,099 s |
| k_δ effektive Lenkverstärkung | 1,387 |
| b_g Gyro-Bias | 1,1·10⁻⁴ rad/s |
| σ_g Gyro-Rauschen | 0,0052 rad/s |
| k_g Gyro-Skalenfaktor | 1,012 |

## Ergebnisse (Fehler gegenüber der Partikelfilter-Pose als Referenz)

| Konfiguration | mittl. Positionsfehler | max. | mittl. Kursfehler |
|---|---|---|---|
| Rad-Odometrie des Fahrzeugs (/odom) | 3,29 m | 7,64 m | 94,6° |
| A: nur Modell (open loop, nur Steuerbefehle) | 2,34 m | 5,19 m | 40,9° |
| B: EKF Radgeschwindigkeit + Gyro | 1,00 m | 2,33 m | 10,7° |
| C: EKF + Absolutpose 1 Hz | 0,22 m | 1,07 m | 5,2° |
| D: wie C, 20 s Ausfall der Lokalisierung | 0,27 m | 1,07 m | 5,3° |

Kernaussage: Die Fusion von Radgeschwindigkeit und Gyro (B) halbiert den
Fehler gegenüber dem reinen Modell, driftet aber weiter (Koppelnavigation).
Für die autonome Navigation braucht der Filter eine absolute Stützung –
schon 1 Hz reicht für ≈ 0,2 m mittleren Fehler; bei Ausfall (D) wächst die
Kovarianz sichtbar und der Filter fängt sich nach der ersten neuen Messung.

Hinweis: Die Partikelfilter-Pose ist Referenz *und* (in C/D, unterabgetastet)
Messung. Für einen strengen Test sollte die Auslegung auf einem zweiten
Datensatz validiert werden.
