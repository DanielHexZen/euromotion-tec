/* course_rect10.c — 10 x 10 m test course of the spec §7.18.5 YAML example (leg 1 without boundary,
 * grass on the right of legs 2..4 at the 0.20 m placement distance, holds 10.5 s).
 * Written by hand in the form tools/course_gen.py will generate (REQ-MAP-04); that tool does not
 * exist yet, so gen_hash says so. Replace this file by the generated one, do not extend it. */
#include <stddef.h>

#include "map.h"

#define V   0.30f
#define W   0.60f
#define WB  0.20f
#define S   SEG_F_STOP_AT_END

static const seg_t SEG[] = {
    { SEG_LINE,     { .line = { 0.0f, 0.0f, 10.0f, 0.0f } },   V,    W, W,  BND_NONE, BND_NONE,   0, S, 0, 0 },
    { SEG_WAYPOINT, { .wp = { 1U, 10.5f } },                   0.0f, 0, 0,  BND_NONE, BND_NONE,   0, S, 0, 0 },
    { SEG_TURN,     { .turn = { 1.57079633f } },               0.0f, 0, 0,  BND_NONE, BND_NONE,   0, 0, 0, 0 },
    { SEG_LINE,     { .line = { 10.0f, 0.0f, 10.0f, 10.0f } }, V,    W, WB, BND_NONE, BND_GRASS, -1, S, 0, 0 },
    { SEG_WAYPOINT, { .wp = { 2U, 10.5f } },                   0.0f, 0, 0,  BND_NONE, BND_NONE,   0, S, 0, 0 },
    { SEG_TURN,     { .turn = { 3.14159265f } },               0.0f, 0, 0,  BND_NONE, BND_NONE,   0, 0, 0, 0 },
    { SEG_LINE,     { .line = { 10.0f, 10.0f, 0.0f, 10.0f } }, V,    W, WB, BND_NONE, BND_GRASS, -1, S, 0, 0 },
    { SEG_WAYPOINT, { .wp = { 3U, 10.5f } },                   0.0f, 0, 0,  BND_NONE, BND_NONE,   0, S, 0, 0 },
    { SEG_TURN,     { .turn = { -1.57079633f } },              0.0f, 0, 0,  BND_NONE, BND_NONE,   0, 0, 0, 0 },
    { SEG_LINE,     { .line = { 0.0f, 10.0f, 0.0f, 0.0f } },   V,    W, WB, BND_NONE, BND_GRASS, -1, S, 0, 0 },
    { SEG_FINISH,   { .turn = { 0.0f } },                      0.0f, 0, 0,  BND_NONE, BND_NONE,   0, 0, 0, 0 },
};

const course_t course_rect10 = {
    "rect10", "handwritten (course_gen.py not written)",
    0.0f, 0.0f, 0.0f,
    SEG, (uint16_t)(sizeof SEG / sizeof SEG[0]),
    NULL, 0U,
};
