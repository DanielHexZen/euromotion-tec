/* nav.h — guidance: line/arc following, encoder-frame final stop, turn (spec §7.19, REQ-NAV-01..03).
 * Pure. */
#ifndef NAV_H
#define NAV_H

#include "ekf.h"
#include "map.h"
#include "prof.h"

typedef enum { NAV_NONE, NAV_VEL, NAV_MOVE_FROM, NAV_STOP } nav_cmd_type_t;

typedef struct {
    nav_cmd_type_t type;
    float v, w;          /* VEL: command. MOVE_FROM: v is the cruise speed                    */
    float D, v0;         /* MOVE_FROM: distance to stop at and speed to start from            */
    bool  done;          /* follow: non-stop segment end reached. turn: finished              */
    bool  aligned;       /* follow: |e_y| and |e_psi| inside the final-stop window. turn: in tolerance */
    float s_rem;         /* follow: L - s (m). turn: heading residual (rad). final: encoder-frame remaining (m) */
} nav_cmd_t;

typedef struct { float s, L, e_y, e_psi, v_ref, w_ref, miss_m; } nav_dbg_t;

typedef struct {
    const seg_t *sg;
    float L, v_end, v_prev;
    bool  final_issued, fin_started, turn_init, prof_ok;
    float fin_D, fin_s0;                       /* final stop: distance and encoder start (encoder frame) */
    float th_t, th0, dir, t_s, t_ok_s;         /* turn                                                   */
    prof_t prof;
    nav_dbg_t dbg;
} nav_t;

void nav_init(nav_t *n);
/* est and s_enc are not needed for the start of a LINE/ARC/TURN; kept for the spec interface. A
 * non-stop LINE/ARC keeps the commanded speed for the next segment, everything else restarts from 0. */
void nav_begin_segment(nav_t *n, const seg_t *sg, const ekf_out_t *est, float s_enc);
/* v_limit: fwd/degraded limit (+inf = none). y_off: lateral offset of the path, left positive.
 * Issues MOVE_FROM once when the final stop starts (with STOP_AT_END). NAV_STOP: pose not finite. */
nav_cmd_t nav_follow(nav_t *n, const ekf_out_t *est, float v_limit, float y_off, float dt);
/* Call every tick of a TURN segment. */
nav_cmd_t nav_turn(nav_t *n, const ekf_out_t *est, float w_meas, float dt);
/* Call every tick after MOVE_FROM, first time in the tick it was issued (latches s_enc). Reports the
 * encoder-frame remainder and, in nav_debug()->miss_m, the EKF's L - s (waypoint miss estimate). */
nav_cmd_t nav_final(nav_t *n, const ekf_out_t *est, float s_enc);
const nav_dbg_t *nav_debug(const nav_t *n);

#endif
