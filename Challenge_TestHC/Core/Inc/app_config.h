/* app_config.h — build switches and rates (spec §3.4, §5.3). Pure. */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifndef APP_CALIB
#define APP_CALIB 0          /* 1 = calibration firmware (calib.c), 0 = vehicle firmware */
#endif

#ifndef FEAT_IMU
#define FEAT_IMU 1          /* 1 = IMU on SPI1 (phase 2, on since 2026-11-20, D-56); 0 = EKF in encoder-heading mode (§7.17.7) */
#endif

#ifndef FEAT_CAPTURE
#define FEAT_CAPTURE 1      /* 1 = 48 KB capture buffer, CAPTURE / CAP_GET (phase 4, REQ-TLM-01); 0 = ACK ST_NODEV */
#endif

#define TICK_HZ     1000U    /* TIM6, PSC 89 / ARR 999 */
#define CTRL_DIV    20U      /* S_CTRL divider -> 50 Hz */
#define CTRL_DT_S   0.02f    /* = CTRL_DIV / TICK_HZ; whl/drv nominal period */

#endif
