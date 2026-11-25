/* map.c — spec §7.18 (REQ-MAP-01..03). */
#include "map.h"

#include <string.h>

#include "par.h"

#define MAP_CONNECT_M  0.001f
#define MAP_TURN_TOL   (0.1f * DEG2RAD_F)
#define MAP_W_MIN_M    0.25f

extern const course_t course_rect10;

static const char *s_why = "";

const course_t *map_course(void) { return &course_rect10; }
const char *map_validate_why(void) { return s_why; }

static bool geo(const seg_t *s) { return s->type == SEG_LINE || s->type == SEG_ARC; }

float map_seg_len(const seg_t *sg)
{
    if (sg->type == SEG_LINE) { return hypotf(sg->g.line.x1 - sg->g.line.x0, sg->g.line.y1 - sg->g.line.y0); }
    if (sg->type == SEG_ARC)  { return sg->g.arc.r * fabsf(sg->g.arc.phi1 - sg->g.arc.phi0); }
    return 0.0f;
}

/* start/end point and heading of a LINE or ARC */
static void ends(const seg_t *s, float p0[2], float p1[2], float *h0, float *h1)
{
    if (s->type == SEG_LINE) {
        p0[0] = s->g.line.x0; p0[1] = s->g.line.y0;
        p1[0] = s->g.line.x1; p1[1] = s->g.line.y1;
        *h0 = *h1 = atan2f(p1[1] - p0[1], p1[0] - p0[0]);
    } else {
        const float q = (float)s->g.arc.dir * 0.5f * PI_F;
        p0[0] = s->g.arc.cx + s->g.arc.r * cosf(s->g.arc.phi0);
        p0[1] = s->g.arc.cy + s->g.arc.r * sinf(s->g.arc.phi0);
        p1[0] = s->g.arc.cx + s->g.arc.r * cosf(s->g.arc.phi1);
        p1[1] = s->g.arc.cy + s->g.arc.r * sinf(s->g.arc.phi1);
        *h0 = s->g.arc.phi0 + q;
        *h1 = s->g.arc.phi1 + q;
    }
}

st_t map_validate(const course_t *c)
{
#define FAIL(msg) do { s_why = (msg); return ST_RANGE; } while (0)
    s_why = "";
    if (c == NULL || c->seg == NULL || c->n_seg < 2U) { FAIL("no segments"); }
    const float vmax = par()->drv_v_max;
    float pe[2] = { c->start_x, c->start_y };   /* end of the previous geometric segment */
    for (uint16_t i = 0U; i < c->n_seg; i++) {
        const seg_t *s = &c->seg[i];
        if (geo(s)) {
            float p0[2], p1[2], h0, h1;
            ends(s, p0, p1, &h0, &h1);
            if (hypotf(p0[0] - pe[0], p0[1] - pe[1]) > MAP_CONNECT_M) { FAIL("segments do not connect"); }
            pe[0] = p1[0]; pe[1] = p1[1];
            if (!(s->v_max > 0.0f && s->v_max <= vmax)) { FAIL("v_max outside (0, drv_v_max]"); }
            /* the reference side sits at the placement distance of D-15 (0.20 m), below the minimum */
            if ((s->ref_side != 1 && s->w_left < MAP_W_MIN_M) || (s->ref_side != -1 && s->w_right < MAP_W_MIN_M)) {
                FAIL("corridor width below 0.25 m");
            }
            if (s->lm_count > 0U) {
                if ((uint32_t)s->lm_first + s->lm_count > c->n_lm) { FAIL("landmark index outside the table"); }
                const float L = map_seg_len(s);
                for (uint16_t k = s->lm_first; k < s->lm_first + s->lm_count; k++) {
                    if (!(c->lm[k].s_from >= 0.0f && c->lm[k].s_from <= c->lm[k].s_to && c->lm[k].s_to <= L)) {
                        FAIL("landmark window outside its segment");
                    }
                }
            }
        }
        if (s->type == SEG_WAYPOINT || s->type == SEG_TURN) {
            if (i == 0U || (c->seg[i - 1U].flags & SEG_F_STOP_AT_END) == 0U) { FAIL("WAYPOINT/TURN without STOP_AT_END before it"); }
        }
        if (s->type == SEG_TURN) {
            uint16_t k = (uint16_t)(i + 1U);
            while (k < c->n_seg && !geo(&c->seg[k])) { k++; }
            if (k >= c->n_seg || c->seg[k].type != SEG_LINE) { FAIL("TURN not followed by a LINE"); }
            float p0[2], p1[2], h0, h1;
            ends(&c->seg[k], p0, p1, &h0, &h1);
            if (fabsf(wrap_pi(h0 - s->g.turn.theta)) > MAP_TURN_TOL) { FAIL("TURN angle differs from the next heading"); }
        }
    }
    if (c->seg[c->n_seg - 1U].type != SEG_FINISH) { FAIL("last segment is not FINISH"); }
    if (hypotf(pe[0] - c->start_x, pe[1] - c->start_y) > MAP_CONNECT_M || !geo(&c->seg[c->n_seg - 2U])) {
        FAIL("FINISH not preceded by a segment ending at the start");
    }
    return ST_OK;
#undef FAIL
}

void map_seg_frame(const seg_t *sg, float x, float y, float th, map_frame_t *out)
{
    memset(out, 0, sizeof *out);
    float psi;
    if (sg->type == SEG_LINE) {
        const float dx = x - sg->g.line.x0, dy = y - sg->g.line.y0;
        out->L = map_seg_len(sg);
        out->tx = (sg->g.line.x1 - sg->g.line.x0) / out->L;
        out->ty = (sg->g.line.y1 - sg->g.line.y0) / out->L;
        out->nx = -out->ty;
        out->ny = out->tx;
        out->s = dx * out->tx + dy * out->ty;
        out->e_y = dx * out->nx + dy * out->ny;
        psi = atan2f(out->ty, out->tx);
    } else if (sg->type == SEG_ARC) {
        const float dir = (float)sg->g.arc.dir;
        const float dx = x - sg->g.arc.cx, dy = y - sg->g.arc.cy;
        const float phi = atan2f(dy, dx);
        psi = phi + dir * 0.5f * PI_F;
        out->L = map_seg_len(sg);
        out->kappa = dir / sg->g.arc.r;
        out->s = sg->g.arc.r * dir * wrap_pi(phi - sg->g.arc.phi0);
        out->e_y = dir * (sg->g.arc.r - hypotf(dx, dy));
        out->tx = cosf(psi);
        out->ty = sinf(psi);
        out->nx = -out->ty;
        out->ny = out->tx;
    } else {
        return;
    }
    out->e_psi = wrap_pi(th - psi);
}

st_t map_boundary(const seg_t *sg, int8_t side, ekf_line_t *out)
{
    if (sg->type != SEG_LINE || (side != 1 && side != -1)) { return ST_RANGE; }
    if ((side == 1 ? sg->bnd_left : sg->bnd_right) == BND_NONE) { return ST_NODEV; }
    map_frame_t f;
    map_seg_frame(sg, sg->g.line.x0, sg->g.line.y0, 0.0f, &f);
    const float w = (side == 1) ? sg->w_left : sg->w_right;
    const float sgn = (float)side;
    const float px = sg->g.line.x0 + sgn * w * f.nx, py = sg->g.line.y0 + sgn * w * f.ny;
    out->nx = -sgn * f.nx;                      /* into the path */
    out->ny = -sgn * f.ny;
    out->d  = out->nx * px + out->ny * py;
    return ST_OK;
}
