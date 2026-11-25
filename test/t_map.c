#include <string.h>

#include "map.h"
#include "par.h"
#include "test.h"

static void frames(void)
{
    const seg_t line = { SEG_LINE, { .line = { 0.0f, 0.0f, 10.0f, 0.0f } }, 0.3f, 0.6f, 0.6f, BND_NONE, BND_NONE, 0, 0, 0, 0 };
    map_frame_t f;
    map_seg_frame(&line, 3.0f, 0.2f, 0.1f, &f);
    CHECK_NEAR(f.s, 3.0, 1e-6); CHECK_NEAR(f.e_y, 0.2, 1e-6); CHECK_NEAR(f.e_psi, 0.1, 1e-6);
    CHECK_NEAR(f.L, 10.0, 1e-6); CHECK_NEAR(f.kappa, 0.0, 0.0);
    CHECK_NEAR(f.tx, 1.0, 1e-6); CHECK_NEAR(f.ny, 1.0, 1e-6);

    const seg_t up = { SEG_LINE, { .line = { 1.0f, 1.0f, 1.0f, 11.0f } }, 0.3f, 0.6f, 0.6f, BND_NONE, BND_NONE, 0, 0, 0, 0 };
    map_seg_frame(&up, 1.3f, 4.0f, PI_F * 0.5f + 0.05f, &f);       /* right of the path: e_y < 0 */
    CHECK_NEAR(f.s, 3.0, 1e-5); CHECK_NEAR(f.e_y, -0.3, 1e-5); CHECK_NEAR(f.e_psi, 0.05, 1e-5);
    CHECK_NEAR(f.nx, -1.0, 1e-6);
    map_seg_frame(&up, 1.0f, 4.0f, -PI_F + 0.1f, &f);              /* heading wraps: pi/2 -> -pi+0.1 */
    CHECK_NEAR(f.e_psi, wrap_pi(-PI_F + 0.1f - PI_F * 0.5f), 1e-5);

    seg_t arc = { SEG_ARC, { .arc = { 0.0f, 0.0f, 2.0f, -PI_F * 0.5f, 0.0f, 1 } }, 0.3f, 0.6f, 0.6f, BND_NONE, BND_NONE, 0, 0, 0, 0 };
    float ph = -PI_F * 0.25f;                                       /* CCW, 0.1 m inside */
    map_seg_frame(&arc, 1.9f * cosf(ph), 1.9f * sinf(ph), PI_F * 0.25f + 0.02f, &f);
    CHECK_NEAR(f.s, 2.0 * (double)PI_F * 0.25, 1e-5); CHECK_NEAR(f.e_y, 0.1, 1e-5);
    CHECK_NEAR(f.kappa, 0.5, 1e-6); CHECK_NEAR(f.e_psi, 0.02, 1e-5); CHECK_NEAR(f.L, PI_F, 1e-5);
    CHECK_NEAR(f.tx, cosf(PI_F * 0.25f), 1e-5);

    arc.g.arc.phi0 = PI_F * 0.5f; arc.g.arc.phi1 = 0.0f; arc.g.arc.dir = -1;   /* CW from (0, 2), 0.1 m outside */
    ph = PI_F * 0.25f;
    map_seg_frame(&arc, 2.1f * cosf(ph), 2.1f * sinf(ph), -PI_F * 0.25f, &f);
    CHECK_NEAR(f.s, 2.0 * (double)PI_F * 0.25, 1e-5); CHECK_NEAR(f.e_y, 0.1, 1e-5);
    CHECK_NEAR(f.kappa, -0.5, 1e-6); CHECK_NEAR(f.e_psi, 0.0, 1e-5);
}

static void boundaries(void)
{
    const course_t *c = map_course();
    ekf_line_t l;
    /* rect10 leg 2: x = 10, going +y, grass on the right at 0.20 m -> the boundary is the line x = 10.2 */
    CHECK(map_boundary(&c->seg[3], -1, &l) == ST_OK);
    CHECK_NEAR(l.nx, -1.0, 1e-6); CHECK_NEAR(l.ny, 0.0, 1e-6); CHECK_NEAR(l.d, -10.2, 1e-5);
    CHECK(map_boundary(&c->seg[3], 1, &l) == ST_NODEV);
    CHECK(map_boundary(&c->seg[0], -1, &l) == ST_NODEV);
    CHECK(map_boundary(&c->seg[1], -1, &l) == ST_RANGE);
    CHECK(map_boundary(&c->seg[3], 0, &l) == ST_RANGE);

    seg_t s = c->seg[0];                                             /* left kerb at 0.6 m on +x */
    s.bnd_left = BND_KERB_DROP;
    CHECK(map_boundary(&s, 1, &l) == ST_OK);
    CHECK_NEAR(l.nx, 0.0, 1e-6); CHECK_NEAR(l.ny, -1.0, 1e-6); CHECK_NEAR(l.d, -0.6, 1e-6);
    /* a point on the path has n'q - d = +w: inside the corridor, positive towards the path */
    CHECK_NEAR(l.ny * 0.0f - l.d, 0.6, 1e-6);
}

static course_t mut(seg_t *s)
{
    const course_t *c = map_course();
    memcpy(s, c->seg, c->n_seg * sizeof *s);
    course_t m = *c;
    m.seg = s;
    return m;
}

static void bad(const course_t *c, const char *why)
{
    CHECK(map_validate(c) == ST_RANGE);
    CHECK(strstr(map_validate_why(), why) != NULL);
    if (strstr(map_validate_why(), why) == NULL) { printf("    why: %s\n", map_validate_why()); }
}

static void validation(void)
{
    seg_t s[11];
    course_t c = mut(s);
    CHECK(map_validate(map_course()) == ST_OK);                      /* rect10 as compiled */
    CHECK(map_validate(&c) == ST_OK);
    CHECK(strcmp(map_course()->name, "rect10") == 0 && map_course()->n_seg == 11U);

    c = mut(s); s[3].g.line.x0 += 0.002f;      bad(&c, "connect");
    c = mut(s); s[0].flags = 0U;               bad(&c, "STOP_AT_END");
    c = mut(s); s[4].flags = 0U;               bad(&c, "STOP_AT_END");           /* waypoint 2 -> turn */
    c = mut(s); s[2].g.turn.theta += 0.01f;    bad(&c, "TURN angle");
    c = mut(s); c.n_seg = 10U;                 bad(&c, "FINISH");
    c = mut(s); s[9].g.line.y1 = 0.01f;        bad(&c, "start");
    c = mut(s); s[0].v_max = 0.5f;             bad(&c, "v_max");
    c = mut(s); s[0].v_max = 0.0f;             bad(&c, "v_max");
    c = mut(s); s[0].w_left = 0.2f;            bad(&c, "0.25");
    c = mut(s); s[3].w_right = 0.30f;          CHECK(map_validate(&c) == ST_OK);  /* reference side may be anything >= 0 */
    c = mut(s); s[6].ref_side = 0;             bad(&c, "0.25");                  /* ... but only on the reference side */

    static const lm_t lm_ok[1]  = { { LM_POST, -1, 2.0f, 8.0f, 8.0f, 0.5f, 1.0f, 0.0f, 0.05f, 0.03f } };
    static const lm_t lm_bad[1] = { { LM_POST, -1, 2.0f, 12.0f, 8.0f, 0.5f, 1.0f, 0.0f, 0.05f, 0.03f } };
    c = mut(s); s[0].lm_count = 1U; c.lm = lm_ok; c.n_lm = 1U;
    CHECK(map_validate(&c) == ST_OK);
    c.lm = lm_bad;                             bad(&c, "landmark window");
    c.n_lm = 0U;                               bad(&c, "landmark index");
}

void t_map(void)
{
    (void)par_init();
    REQ("REQ-MAP-01: frame quantities (s, L, e_y, e_psi, kappa, t, n) for LINE and ARC, both directions");
    frames();
    REQ("REQ-MAP-01: boundary lines, normal into the path, d = n'q");
    boundaries();
    REQ("REQ-MAP-03: rect10 validates; every rule violation is caught and named");
    validation();
}
