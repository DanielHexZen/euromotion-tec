/* sysid.c -- open-loop identification tests for the EuroMotion car
 * MR2006B Assessment 1 (LS / ARX identification), team EuroMotion
 *
 * Every test: 1.5 s warm-up at the base duty (operating point), then the
 * excitation, then the motors stop. The log holds one row per 20 ms:
 *   k, duty_L*100, duty_R*100, encoder delta L, encoder delta R
 * Speed on the PC: v = delta * MM_PER_COUNT / 0.02  [mm/s]
 */
#include "sysid.h"
#include <stdio.h>
#include <math.h>

#define TS_S          0.02f      /* must match TIM6 / CTRL_PERIOD_S          */
#define T_PRE_S       1.5f       /* warm-up at the operating point           */
#define LOG_MAX       1000U      /* 20 s at 50 Hz, 8 bytes per row = 8 kB    */

/* longitudinal tests: same duty on both wheels */
#define LON_BASE      40.0f      /* [%] operating point                      */
#define LON_AMP       10.0f      /* [%] excitation amplitude                 */
/* lateral tests: base duty +/- differential duty */
#define LAT_BASE      35.0f      /* [%]                                      */
#define LAT_AMP        8.0f      /* [%] differential duty (R up, L down)     */

typedef struct { int16_t ul, ur, dl, dr; } Row_t;

static Row_t    log_buf[LOG_MAX];
static uint16_t n_log   = 0;
static uint32_t tick    = 0;
static uint8_t  test    = 0;
static uint8_t  active  = 0;
static uint8_t  dump    = 0;
static uint8_t  lfsr    = 0x7F;  /* 7-bit PRBS, x^7 + x^6 + 1, 127 bits     */
static float    prbs_v  = 1.0f;

/* ---- segment tables: {duration [s], value [-1..1] of the amplitude} ---- */
typedef struct { float dur, val; } Seg_t;

static const Seg_t seg_step[] = {            /* multi-level step, 10 s     */
  {2.0f, +1.0f}, {2.0f, -1.0f}, {2.0f, +0.5f}, {2.0f, -0.5f}, {2.0f, 0.0f} };
static const Seg_t seg_dlc[] = {             /* double lane change, 6.5 s  */
  {0.5f, 0.0f}, {1.0f, +1.0f}, {1.0f, -1.0f}, {1.0f, 0.0f},
  {1.0f, -1.0f}, {1.0f, +1.0f}, {1.0f, 0.0f} };
static const Seg_t seg_slc[] = {             /* single lane change, 4.4 s  */
  {0.5f, 0.0f}, {1.2f, +1.0f}, {1.2f, -1.0f}, {1.5f, 0.0f} };

#define NSEG(a) (sizeof(a) / sizeof((a)[0]))

static float seg_eval(const Seg_t *s, uint32_t n, float t, uint8_t *done)
{
  for (uint32_t i = 0; i < n; i++) {
    if (t < s[i].dur) return s[i].val;
    t -= s[i].dur;
  }
  *done = 1;
  return 0.0f;
}

/* PRBS: new bit every tb ticks, 127 bits = one full period */
static float prbs_eval(uint32_t k, uint32_t tb, uint8_t *done)
{
  if (k >= 127U * tb) { *done = 1; return 0.0f; }
  if (k % tb == 0U) {
    uint8_t bit = ((lfsr >> 6) ^ (lfsr >> 5)) & 1U;
    lfsr = (uint8_t)(((lfsr << 1) | bit) & 0x7FU);
    prbs_v = bit ? 1.0f : -1.0f;
  }
  return prbs_v;
}

uint8_t SysId_Start(uint8_t id)
{
  if (id == 0U || id >= SYSID_N_TESTS) return 0U;
  test = id; tick = 0; n_log = 0; dump = 0;
  lfsr = 0x7F; prbs_v = 1.0f;
  active = 1;
  return 1U;
}

void    SysId_Abort(void)     { active = 0; }
uint8_t SysId_Active(void)    { return active; }
uint8_t SysId_DumpReady(void) { return dump; }

void SysId_Tick(int32_t d_l, int32_t d_r, float *duty_l, float *duty_r)
{
  if (!active) { *duty_l = 0.0f; *duty_r = 0.0f; return; }

  float t   = (float)tick * TS_S;
  float te  = t - T_PRE_S;                       /* time of the excitation */
  uint32_t ke = (tick >= (uint32_t)(T_PRE_S / TS_S + 0.5f))
              ? tick - (uint32_t)(T_PRE_S / TS_S + 0.5f) : 0U;
  uint8_t done = 0;
  float x = 0.0f;                                /* excitation in -1..1    */
  uint8_t lat = (test >= SYSID_LAT_PRBS);

  if (te >= 0.0f) {
    switch (test) {
      case SYSID_LON_STEP:  x = seg_eval(seg_step, NSEG(seg_step), te, &done); break;
      case SYSID_LON_PRBS:  x = prbs_eval(ke, 3U, &done); break;     /* 60 ms bit */
      case SYSID_LON_CHIRP: {
        const float f0 = 0.2f, f1 = 3.0f, T = 10.0f;
        if (te >= T) done = 1;
        else x = sinf(2.0f * 3.14159265f * (f0 * te + 0.5f * (f1 - f0) * te * te / T));
      } break;
      case SYSID_LAT_PRBS:  x = prbs_eval(ke, 4U, &done); break;     /* 80 ms bit */
      case SYSID_LAT_DLC:   x = seg_eval(seg_dlc, NSEG(seg_dlc), te, &done); break;
      case SYSID_LAT_SLC:   x = seg_eval(seg_slc, NSEG(seg_slc), te, &done); break;
      default: done = 1; break;
    }
  }

  if (done || n_log >= LOG_MAX) {
    active = 0; dump = 1;
    *duty_l = 0.0f; *duty_r = 0.0f;
    return;
  }

  if (lat) {
    *duty_l = LAT_BASE - LAT_AMP * x;
    *duty_r = LAT_BASE + LAT_AMP * x;
  } else {
    *duty_l = LON_BASE + LON_AMP * x;
    *duty_r = *duty_l;
  }

  /* the encoder deltas belong to the duty of the PREVIOUS period, the PC
     script handles this with the input delay nk >= 1                       */
  log_buf[n_log].ul = (int16_t)lrintf(*duty_l * 100.0f);
  log_buf[n_log].ur = (int16_t)lrintf(*duty_r * 100.0f);
  log_buf[n_log].dl = (int16_t)d_l;
  log_buf[n_log].dr = (int16_t)d_r;
  n_log++;
  tick++;
}

void SysId_Dump(void (*print)(const char *txt))
{
  static const char *names[SYSID_N_TESTS] =
    { "", "lon_step", "lon_prbs", "lon_chirp", "lat_prbs", "lat_dlc", "lat_slc" };
  char line[64];
  snprintf(line, sizeof line, "# SYSID %s Ts_ms=20 Tpre_ms=1500 N=%u\r\n",
           names[test], (unsigned)n_log);
  print(line);
  print("k,uL_x100,uR_x100,dL,dR\r\n");
  for (uint16_t i = 0; i < n_log; i++) {
    snprintf(line, sizeof line, "%u,%d,%d,%d,%d\r\n", (unsigned)i,
             log_buf[i].ul, log_buf[i].ur, log_buf[i].dl, log_buf[i].dr);
    print(line);
  }
  print("# END\r\n");
  dump = 0;
}
