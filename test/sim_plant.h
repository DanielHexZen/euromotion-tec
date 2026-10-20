/* sim_plant.h — drivetrain model for the host tests (spec §14.1 t_drv_move).
 * Per wheel: first-order lag (tau) towards v_ss = sign(u) (|u| - Ks) / Kv, Coulomb breakaway
 * at Ksb, encoder quantised to counts. Kv is the nominal value times a mismatch factor. */
#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct { double x, v, ks, ksb, kv, tau; bool blocked; double sign; } sim_wheel_t;
typedef struct { sim_wheel_t w[2]; double mpc; } sim_plant_t;

/* kv_mis_l/r: true Kv = nominal * factor. Needs par_init() first. */
void    sim_init(sim_plant_t *s, double kv_mis_l, double kv_mis_r);
/* Zero-order hold on duty for dt_s, integrated in 1 ms steps. enable=false: coasting. */
void    sim_advance(sim_plant_t *s, const float duty[2], bool enable, double dt_s);
int32_t sim_counts(const sim_plant_t *s, int side);

#endif
