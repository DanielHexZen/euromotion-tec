/* att.h — roll and pitch complementary filter (spec §7.12). Pure. */
#ifndef ATT_H
#define ATT_H

#include "units.h"

typedef struct {
    float phi, theta;          /* rad                                                   */
    float sum_phi, sum_theta;  /* accelerometer angle sums of the 0.5 s start-up window */
    float sum_w[2], bw[2];     /* gyro x, y: sums of that window, then the bias (D-69)  */
    float t_init_s;            /* time spent in the start-up window                     */
    uint16_t n_init;
} att_t;

void att_init(att_t *a);
/* w = body rates p,q,r (rad/s), acc = specific force (m/s2). The first 0.5 s only average the
 * accelerometer angles and the gyro x, y rates; the vehicle stands at start (spec §7.12). The
 * averaged x, y rates are the gyro bias and are subtracted afterwards (D-69). */
void att_update(att_t *a, const float w_radps[3], const float acc_mps2[3], float dt_s);
float att_roll(const att_t *a);
float att_pitch(const att_t *a);
float att_bias_y(const att_t *a);   /* gyro y bias of the start-up window, rad/s (0 before) */

#endif
