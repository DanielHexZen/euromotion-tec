/* sysid.h -- open-loop identification tests for the EuroMotion car
 * MR2006B Assessment 1 (LS / ARX identification)
 *
 * Runs inside Control_Tick() (TIM6, 50 Hz). Generates the excitation,
 * logs duty and raw encoder counts to RAM, dumps a CSV over USART2.
 */
#ifndef SYSID_H
#define SYSID_H

#include <stdint.h>

/* Test IDs (serial command X<id>) */
enum {
  SYSID_LON_STEP  = 1,   /* longitudinal: multi-level step          */
  SYSID_LON_PRBS  = 2,   /* longitudinal: PRBS around base duty     */
  SYSID_LON_CHIRP = 3,   /* longitudinal: sine sweep 0.2 -> 3 Hz    */
  SYSID_LAT_PRBS  = 4,   /* lateral: PRBS on differential duty      */
  SYSID_LAT_DLC   = 5,   /* lateral: double lane change             */
  SYSID_LAT_SLC   = 6,   /* lateral: single lane change             */
  SYSID_N_TESTS   = 7
};

uint8_t SysId_Start(uint8_t test_id);           /* 1 = started, 0 = bad id    */
void    SysId_Abort(void);
uint8_t SysId_Active(void);                     /* test is running            */
uint8_t SysId_DumpReady(void);                  /* test done, data not sent   */

/* Call every 20 ms. d_l, d_r: raw encoder deltas of this period (signed,
   forward positive). Returns the duty [%] for both wheels.                  */
void    SysId_Tick(int32_t d_l, int32_t d_r, float *duty_l, float *duty_r);

/* Sends the log as CSV through the given print function (blocking).        */
void    SysId_Dump(void (*print)(const char *txt));

#endif
