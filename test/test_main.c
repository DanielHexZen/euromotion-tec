#include <stdio.h>

#include "test.h"

int g_checks, g_fails;

void t_crc(void);
void t_fmt(void);
void t_par(void);
void t_prof(void);
void t_whl_law(void);
void t_drv_move(void);
void t_con(void);
void t_sched(void);
void t_att(void);
void t_ekf_static(void);
void t_ekf_slip(void);
void t_ekf_gate(void);
void t_ekf_sf_q(void);
void t_ekf_jacobians(void);
void t_ekf_backshift(void);
void t_ekf_line(void);
void t_ekf_pred(void);
void t_imu_yaw(void);

int main(void)
{
    static const struct { const char *name; void (*fn)(void); } T[] = {
        { "t_crc", t_crc }, { "t_fmt", t_fmt }, { "t_par", t_par }, { "t_prof", t_prof },
        { "t_whl_law", t_whl_law }, { "t_drv_move", t_drv_move }, { "t_con", t_con }, { "t_sched", t_sched },
        { "t_att", t_att },
        { "t_ekf_static", t_ekf_static },
        { "t_ekf_slip", t_ekf_slip },
        { "t_ekf_gate", t_ekf_gate },
        { "t_ekf_sf_q", t_ekf_sf_q },
        { "t_ekf_jacobians", t_ekf_jacobians },
        { "t_ekf_backshift", t_ekf_backshift },
        { "t_ekf_line", t_ekf_line },
        { "t_ekf_pred", t_ekf_pred },
        { "t_imu_yaw", t_imu_yaw },
    };
    for (unsigned i = 0; i < sizeof T / sizeof T[0]; i++) {
        printf("%s\n", T[i].name);
        T[i].fn();
    }
    printf("%d checks, %d failed\n", g_checks, g_fails);
    return g_fails != 0;
}
