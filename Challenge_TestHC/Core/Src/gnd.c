/* gnd.c — spec §7.14 (REQ-GND-01..03) with the reduced sensor set of D-63: the classifier runs
 * on S2 with h = r·sin(lat_tilt + roll) and h0 = lat_h (REQ-LAT-07). */
#include "gnd.h"

#include <string.h>

#include "par.h"

void gnd_init(gnd_t *g)
{
    memset(g, 0, sizeof *g);
    g->s.cls = GND_CL_UNKNOWN;
    g->s.cand = GND_CL_UNKNOWN;
}

static float spread(const gnd_sensor_t *s, uint8_t n)
{
    float m = 0.0f, v = 0.0f;
    for (uint8_t i = 0U; i < n; i++) { m += s->h[i]; }
    m /= (float)n;
    for (uint8_t i = 0U; i < n; i++) { v += (s->h[i] - m) * (s->h[i] - m); }
    return sqrtf(v / (float)n);
}

static bool is_edge(gnd_class_t c) { return c == GND_CL_GRASS || c == GND_CL_EDGE_UP || c == GND_CL_DROP; }

static void publish(gnd_t *g, gnd_class_t to, uint32_t t_us)
{
    const gnd_class_t from = g->s.cls;
    g->s.cls = to;
    const bool ev = (from == GND_CL_PATH && is_edge(to)) || (is_edge(from) && to == GND_CL_PATH);   /* REQ-GND-03 */
    if (!ev) { return; }
    if ((uint8_t)(g->ev_head - g->ev_tail) >= GND_EV_N) { g->ev_tail++; }   /* drop the oldest */
    g->ev[g->ev_head % GND_EV_N] = (gnd_edge_ev_t){ .id = VL_S2, .t_us = t_us, .from = from, .to = to };
    g->ev_head++;
}

static void debounce(gnd_t *g, gnd_class_t raw, uint32_t t_us, uint8_t n_deb)
{
    gnd_sensor_t *s = &g->s;
    if (raw == s->cls) { s->cand_n = 0U; return; }
    if (raw != s->cand || s->cand_n == 0U) { s->cand = raw; s->cand_n = 0U; s->cand_t_us = t_us; }
    if (++s->cand_n >= n_deb) { publish(g, raw, s->cand_t_us); s->cand_n = 0U; }
}

void gnd_on_meas(gnd_t *g, uint8_t id, const vl_meas_t *m, float roll, float pitch)
{
    (void)pitch;                                    /* the S2 beam axis is the body y axis (§7.13.4) */
    if (id != VL_S2) { return; }
    const par_t *q = par();
    gnd_sensor_t *s = &g->s;
    const float h = m->range_m * sinf(q->lat_tilt + roll);   /* S2: θ_eff = θ + φ */
    const float h0 = q->lat_h;
    const uint8_t n_sd = (uint8_t)q->gnd_sd_n;

    gnd_class_t raw;
    const bool far_fail = m->status == VL_ST_SIGNAL_FAIL && h > h0;   /* nothing returned: ground far below */
    if (m->status != VL_ST_VALID && !far_fail) {
        raw = GND_CL_UNKNOWN;
    } else {
        s->h[s->h_i] = h;
        s->h_i = (uint8_t)((s->h_i + 1U) % n_sd);
        if (s->h_n < n_sd) { s->h_n++; }
        const float sd = spread(s, s->h_n);
        if (h > h0 + q->gnd_d_drop) {
            raw = GND_CL_DROP;
        } else if (sd >= q->gnd_sd_grass || (q->gnd_sig_grass > 0.0f && m->signal_kcps < q->gnd_sig_grass)) {
            raw = GND_CL_GRASS;
        } else if (h < h0 - q->gnd_d_up) {
            raw = GND_CL_EDGE_UP;
        } else {
            raw = GND_CL_PATH;
        }
        s->t_valid_us = m->t_us;
        s->seen = true;
    }
    debounce(g, raw, m->t_us, (uint8_t)q->gnd_n_deb);
}

void gnd_step(gnd_t *g, uint32_t now_us)
{
    gnd_sensor_t *s = &g->s;
    const uint32_t to_us = (uint32_t)(par()->vl_timeout * 1.0e6f);
    if (s->cls != GND_CL_UNKNOWN && (!s->seen || (uint32_t)(now_us - s->t_valid_us) > to_us)) {
        s->cls = GND_CL_UNKNOWN;                    /* stale: no event (REQ-GND-03) */
        s->cand_n = 0U;
    }
}

gnd_class_t gnd_class(const gnd_t *g, uint8_t id) { return (id == VL_S2) ? g->s.cls : GND_CL_UNKNOWN; }

gnd_side_t gnd_side(const gnd_t *g, uint8_t side)
{
    if (side != GND_RIGHT || g->s.cls == GND_CL_UNKNOWN) { return GND_UNKNOWN; }   /* left has no sensor (D-63) */
    return (g->s.cls == GND_CL_PATH) ? GND_FAR : GND_NOMINAL;
}

/* shortcut: one footprint cannot tell NOMINAL from TOO_CLOSE (D-63), so no sensor interlock;
 * replace with the decision of OQ-12 before AUTO runs on the course. */
bool gnd_interlock(const gnd_t *g) { (void)g; return false; }

bool gnd_pop_edge_event(gnd_t *g, gnd_edge_ev_t *ev)
{
    if (g->ev_head == g->ev_tail) { return false; }
    *ev = g->ev[g->ev_tail % GND_EV_N];
    g->ev_tail++;
    return true;
}
