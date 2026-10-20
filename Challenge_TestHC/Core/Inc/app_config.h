/* app_config.h — build switches and rates (spec §3.4, §5.3). Pure. */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifndef APP_CALIB
#define APP_CALIB 0          /* 1 = calibration firmware (calib.c), 0 = vehicle firmware */
#endif

#define TICK_HZ     1000U    /* TIM6, PSC 89 / ARR 999 */
#define CTRL_DIV    20U      /* S_CTRL divider -> 50 Hz */
#define CTRL_DT_S   0.02f    /* = CTRL_DIV / TICK_HZ; whl/drv nominal period */

#endif
