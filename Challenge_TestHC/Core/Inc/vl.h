/* vl.h — VL53L1X sensor IDs and result type (spec §7.13). Types only: vl.c needs the ST ULD
 * (§7.13.2) and is not written yet. Pure modules (gnd, fwd, lat) include this header. */
#ifndef VL_H
#define VL_H

#include "units.h"

/* IDs of VL_CFG (§7.13.1). D-63: G1-G4 not fitted, L1 takes the slot of S1. */
enum { VL_G1, VL_G2, VL_G3, VL_G4, VL_F1, VL_F2, VL_L1, VL_S2, VL_N };

#define VL_ST_VALID       0U
#define VL_ST_SIGNAL_FAIL 2U   /* ULD range status mapping [VERIFY] against VL53L1X_GetResult */

typedef struct {
    float    range_m;       /* raw distance                 */
    uint8_t  status;        /* ULD range status (0 = valid) */
    float    signal_kcps;   /* signal rate per SPAD         */
    float    ambient_kcps;
    uint32_t t_us;          /* t_meas, REQ-VL-03            */
    uint16_t seq;
} vl_meas_t;

const vl_meas_t *vl_get(uint8_t id);

#endif
