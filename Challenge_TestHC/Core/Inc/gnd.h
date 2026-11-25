/* gnd.h — ground classification and edge events (spec §7.14, D-70: S2 only). Pure. */
#ifndef GND_H
#define GND_H

#include "vl.h"

typedef enum { GND_CL_UNKNOWN, GND_CL_PATH, GND_CL_GRASS, GND_CL_EDGE_UP, GND_CL_DROP } gnd_class_t;
typedef enum { GND_UNKNOWN, GND_FAR, GND_NOMINAL, GND_TOO_CLOSE } gnd_side_t;
enum { GND_LEFT, GND_RIGHT };

#define GND_SD_MAX 15U    /* gnd_sd_n upper limit */
#define GND_EV_N   8U

typedef struct { uint8_t id; uint32_t t_us; gnd_class_t from, to; } gnd_edge_ev_t;

typedef struct {
    float       h[GND_SD_MAX];   /* last heights (m), ring */
    uint8_t     h_n, h_i;
    gnd_class_t cls;             /* published               */
    gnd_class_t cand;            /* debounce candidate      */
    uint8_t     cand_n;
    uint32_t    cand_t_us;       /* first sample of cand    */
    uint32_t    t_valid_us;      /* last classified sample  */
    bool        seen;
} gnd_sensor_t;

typedef struct {
    gnd_sensor_t  s;             /* S2, the only ground sensor (D-70) */
    gnd_edge_ev_t ev[GND_EV_N];
    uint8_t       ev_head, ev_tail;
} gnd_t;

void gnd_init(gnd_t *g);
/* roll, pitch in rad (att). Results of sensors other than S2 are ignored (D-70). */
void gnd_on_meas(gnd_t *g, uint8_t id, const vl_meas_t *m, float roll, float pitch);
void gnd_step(gnd_t *g, uint32_t now_us);              /* staleness                      */
gnd_class_t gnd_class(const gnd_t *g, uint8_t id);
gnd_side_t gnd_side(const gnd_t *g, uint8_t side);
bool gnd_interlock(const gnd_t *g);                    /* any side TOO_CLOSE             */
bool gnd_pop_edge_event(gnd_t *g, gnd_edge_ev_t *ev);
/* Median of v[0..n-1] (sorts v in place); NAN for n = 0. LATCAL (REQ-VL-10). */
float gnd_median(float *v, uint8_t n);

#endif
