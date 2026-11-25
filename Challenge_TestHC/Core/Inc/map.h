/* map.h — course representation (spec §7.18, REQ-MAP-01..03). Pure. */
#ifndef MAP_H
#define MAP_H

#include "ekf.h"
#include "units.h"

typedef enum { SEG_LINE, SEG_ARC, SEG_TURN, SEG_WAYPOINT, SEG_PAUSE, SEG_FINISH } seg_type_t;
typedef enum { BND_NONE, BND_GRASS, BND_EDGING, BND_KERB_DROP, BND_STEP_DROP, BND_WALL } bnd_t;
enum { SEG_F_STOP_AT_END = 1, SEG_F_NO_EVADE = 2, SEG_F_RAMP = 4, SEG_F_NO_LANDMARKS = 8 };

typedef struct {
    seg_type_t type;
    union {
        struct { float x0, y0, x1, y1; }                          line;  /* m          */
        struct { float cx, cy, r, phi0, phi1; int8_t dir; }       arc;   /* dir +1 CCW */
        struct { float theta; }                                   turn;  /* absolute   */
        struct { uint8_t id; float hold_s; }                      wp;
        struct { float t_s; }                                     pause;
    } g;
    float    v_max;                 /* m/s, <= drv limit                             */
    float    w_left, w_right;       /* path line -> boundary, m (corridor)           */
    bnd_t    bnd_left, bnd_right;
    int8_t   ref_side;              /* +1 left, -1 right, 0 none (edge tracked)      */
    uint8_t  flags;
    uint16_t lm_first, lm_count;    /* landmarks valid on this segment               */
} seg_t;

typedef enum { LM_FACE, LM_POST } lm_type_t;
typedef struct {
    lm_type_t type;
    int8_t    side;                 /* sensor side that sees it: +1 S1, -1 S2        */
    float     s_from, s_to;         /* along-segment window, m                       */
    float     px, py;               /* post centre, or a point on the face           */
    float     tx, ty;               /* face direction, unit (travel direction)       */
    float     r_post;               /* post radius, m                                */
    float     sigma;                /* survey sigma, m                               */
} lm_t;

typedef struct {
    const char *name; const char *gen_hash;
    float start_x, start_y, start_theta;
    const seg_t *seg; uint16_t n_seg;
    const lm_t  *lm;  uint16_t n_lm;
} course_t;

typedef struct { float s, L, e_y, e_psi, kappa, tx, ty, nx, ny; } map_frame_t;

const course_t *map_course(void);
/* ST_OK or ST_RANGE; map_validate_why() names the first violated rule of the last call. */
st_t map_validate(const course_t *c);
const char *map_validate_why(void);
float map_seg_len(const seg_t *sg);                       /* LINE, ARC: length in m, else 0 */
/* LINE and ARC only (others: all zero). e_y positive to the left of the path. ARC s is
 * R*dir*wrap(phi - phi0): fine for arcs that sweep less than pi. */
void map_seg_frame(const seg_t *sg, float x, float y, float th, map_frame_t *out);
/* side +1 left, -1 right. ST_RANGE: not a LINE or bad side; ST_NODEV: no boundary on that side. */
st_t map_boundary(const seg_t *sg, int8_t side, ekf_line_t *out);

#endif
