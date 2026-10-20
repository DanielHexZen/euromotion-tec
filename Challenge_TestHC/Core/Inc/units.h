/* units.h — constants, status codes and scalar helpers (spec §6.3, §6.4). Pure. */
#ifndef UNITS_H
#define UNITS_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define PI_F        3.14159265f
#define TWO_PI_F    6.28318531f
#define DEG2RAD_F   0.0174532925f
#define RAD2DEG_F   57.2957795f
#define G_MPS2      9.80665f

#define BAT_ABSENT_V 3.0f   /* below this the pack is absent (USB bench), REQ-BAT-02 */

typedef enum {
    ST_OK = 0,
    ST_BUSY,        /* operation in progress, call again        */
    ST_TIMEOUT,     /* peripheral did not respond in time        */
    ST_BUS,         /* I2C NACK / bus error / SPI error          */
    ST_RANGE,       /* argument out of the allowed range         */
    ST_STATE,       /* not allowed in the current mode/state     */
    ST_CRC,         /* frame or data integrity error             */
    ST_NODEV,       /* device absent or disabled                 */
    ST_REJECT       /* measurement rejected by a gate            */
} st_t;

/* Wraps to (-pi, pi]. NaN passes through. */
static inline float wrap_pi(float a)
{
    float r = fmodf(a + PI_F, TWO_PI_F);
    if (r <= 0.0f) { r += TWO_PI_F; }
    return r - PI_F;
}

/* NaN passes through so the actuator boundary can catch it (REQ-COD-10). */
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline float signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }
static inline float sqf(float x)   { return x * x; }

#endif
