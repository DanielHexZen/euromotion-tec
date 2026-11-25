#include <math.h>

#include "test.h"
#include "tlm.h"
#include "units.h"

static tlm_cfg_t cfg(uint32_t baud, uint32_t air_max_ms)
{
    return (tlm_cfg_t){ .baud = baud, .period_ms = 100U, .air_max_ms = air_max_ms, .guard_ms = 5U, .uplink_ms = 30U };
}

/* airtime of a frame with `payload` bytes, in ms, against the number printed in §8.3 (one decimal) */
static void air(uint32_t baud, size_t payload, double ms)
{
    CHECK_NEAR((double)tlm_air_us(baud, PROTO_WIRE(payload)) / 1000.0, ms, 0.06);
}

static void ev_n(tlm_ev_t *e, int32_t k)
{
    const proto_event_t ev = { (uint32_t)k * 10U, 0x0101U, k };
    tlm_ev_push(e, &ev);
}

void t_tlm(void)
{
    tlm_plan_t p;

    REQ("REQ-PRO-04, §8.3: airtime at 9600 Bd (1.0417 ms per byte)");
    air(9600U, PROTO_LEN_TELEM_FAST, 40.6);
    air(9600U, PROTO_LEN_TELEM_SLOW, 43.8);
    air(9600U, PROTO_LEN_ACK, 10.4);
    air(9600U, PROTO_LEN_PARAM_VAL, 14.6);
    air(9600U, PROTO_LEN_EVENT, 17.7);
    air(9600U, PROTO_LEN_HELLO, 34.4);
    air(9600U, PROTO_LEN_DRIVE, 12.5);
    air(9600U, PROTO_LEN_PARAM_SET, 13.5);
    CHECK(tlm_air_us(9600U, 39U) == 40625U && tlm_air_us(9600U, 1U) == 1041U);

    REQ("REQ-PRO-04, §8.3: airtime at 19200 Bd (0.5208 ms per byte, D-35)");
    air(19200U, PROTO_LEN_TELEM_FAST, 20.3);
    air(19200U, PROTO_LEN_TELEM_SLOW, 21.9);
    air(19200U, PROTO_LEN_ACK, 5.2);
    air(19200U, PROTO_LEN_PARAM_VAL, 7.3);
    air(19200U, PROTO_LEN_EVENT, 8.9);
    air(19200U, PROTO_LEN_HELLO, 17.2);
    air(19200U, PROTO_LEN_DRIVE, 6.3);
    air(19200U, PROTO_LEN_PARAM_SET, 6.8);
    CHECK(tlm_air_us(19200U, 39U) == 20312U && tlm_air_us(0U, 39U) == 0U);

    REQ("REQ-PRO-04: uplink frame limit 30 ms = 28 B at 9600 Bd, 57 B at 19200 Bd");
    { const tlm_cfg_t a = cfg(9600U, 55U), b = cfg(19200U, 55U);
      CHECK(tlm_uplink_max_bytes(&a) == 28U && tlm_uplink_max_bytes(&b) == 57U); }

    REQ("REQ-PRO-04, D-34: window check at 9600 Bd, tlm_air_max_ms = 55");
    { const tlm_cfg_t c = cfg(9600U, 55U);
      tlm_plan(&c, TLM_REPLY_ACK, 0U, 0U, &p);                   /* ACK + TELEM_FAST = 51.0 ms */
      CHECK(p.telem == TLM_T_FAST && !p.over && !p.pc_late);
      CHECK_NEAR((double)p.air_us / 1000.0, 51.0, 0.06);
      tlm_plan(&c, TLM_REPLY_ACK, 9U, 0U, &p);                   /* ACK + TELEM_SLOW = 54.2 ms */
      CHECK(p.telem == TLM_T_SLOW && !p.over && !p.pc_late);
      CHECK_NEAR((double)p.air_us / 1000.0, 54.2, 0.06);
      tlm_plan(&c, TLM_REPLY_PARAM_VAL, 0U, 0U, &p);             /* 55.2 ms: over, still sent  */
      CHECK(p.telem == TLM_T_FAST && p.over);
      CHECK_NEAR((double)p.air_us / 1000.0, 55.2, 0.06);
      tlm_plan(&c, TLM_REPLY_PARAM_VAL, 9U, 0U, &p);             /* 58.3 ms, PC window ends 93.3 */
      CHECK(p.telem == TLM_T_SLOW && p.over && !p.pc_late);
      CHECK_NEAR((double)p.air_us / 1000.0, 58.3, 0.06);
      CHECK_NEAR((double)p.pc_end_us / 1000.0, 93.3, 0.06);
      tlm_plan(&c, TLM_REPLY_ACK_HELLO, 0U, 0U, &p);             /* HELLO replaces telemetry: 44.8 ms */
      CHECK(p.telem == TLM_T_NONE && !p.over && !p.pc_late);
      CHECK_NEAR((double)p.air_us / 1000.0, 44.8, 0.06);
      tlm_plan(&c, TLM_REPLY_HELLO, 9U, 0U, &p);
      CHECK(p.telem == TLM_T_NONE);
      /* ACK + HELLO + TELEM_FAST would be 85.4 ms, PC window ends at 120 ms: the reason for the rule */
      CHECK(tlm_air_us(9600U, 10U) + tlm_air_us(9600U, 33U) + tlm_air_us(9600U, 39U) + 35000U > 100000U);
      tlm_plan(&c, TLM_REPLY_NONE, 0U, 0U, &p);
      CHECK(p.telem == TLM_T_FAST && !p.over);
      CHECK_NEAR((double)p.air_us / 1000.0, 40.6, 0.06); }

    REQ("D-34 [DECIDE]: at 9600 Bd and 55 ms no EVENT fits behind the telemetry; 59 ms admits one behind FAST");
    { tlm_cfg_t c = cfg(9600U, 55U);
      tlm_plan(&c, TLM_REPLY_NONE, 0U, 5U, &p);
      CHECK(p.n_events == 0U && p.telem == TLM_T_FAST);
      tlm_plan(&c, TLM_REPLY_ACK, 0U, 5U, &p);
      CHECK(p.n_events == 0U);
      tlm_plan(&c, TLM_REPLY_ACK, 9U, 5U, &p);
      CHECK(p.n_events == 0U && p.telem == TLM_T_SLOW);
      c.air_max_ms = 58U;
      tlm_plan(&c, TLM_REPLY_NONE, 0U, 5U, &p);                  /* 58.3 ms needs 59 */
      CHECK(p.n_events == 0U);
      c.air_max_ms = 59U;
      tlm_plan(&c, TLM_REPLY_NONE, 0U, 5U, &p);
      CHECK(p.n_events == 1U && !p.over && !p.pc_late);
      CHECK_NEAR((double)p.air_us / 1000.0, 58.3, 0.06);
      CHECK_NEAR((double)p.pc_end_us / 1000.0, 93.3, 0.06);
      c.air_max_ms = 60U;
      tlm_plan(&c, TLM_REPLY_NONE, 9U, 5U, &p);                  /* none behind TELEM_SLOW */
      CHECK(p.n_events == 0U); }

    REQ("REQ-PRO-04, D-35: window check at 19200 Bd, all combinations fit, events behind the telemetry");
    { const tlm_cfg_t c = cfg(19200U, 55U);
      tlm_plan(&c, TLM_REPLY_ACK, 0U, 0U, &p);                   /* 25.5 ms */
      CHECK(!p.over && !p.pc_late);
      CHECK_NEAR((double)p.air_us / 1000.0, 25.5, 0.06);
      tlm_plan(&c, TLM_REPLY_ACK, 0U, 9U, &p);                   /* 25.5 + 3 * 8.9 = 52.1 ms */
      CHECK(p.n_events == 3U && !p.over);
      CHECK_NEAR((double)p.air_us / 1000.0, 52.1, 0.06);
      tlm_plan(&c, TLM_REPLY_ACK, 0U, 2U, &p);                   /* capped by what is pending */
      CHECK(p.n_events == 2U);
      tlm_plan(&c, TLM_REPLY_PARAM_VAL, 9U, 0U, &p);             /* 29.2 ms */
      CHECK(p.telem == TLM_T_SLOW && !p.over);
      CHECK_NEAR((double)p.air_us / 1000.0, 29.2, 0.06);
      tlm_plan(&c, TLM_REPLY_PARAM_VAL, 9U, 9U, &p);             /* 2 events behind PARAM_VAL + SLOW */
      CHECK(p.n_events == 2U && !p.over);
      tlm_plan(&c, TLM_REPLY_ACK_HELLO, 0U, 0U, &p);
      CHECK(p.telem == TLM_T_NONE && !p.over && !p.pc_late);
      tlm_plan(&c, TLM_REPLY_NONE, 0U, 9U, &p);                  /* FAST alone: 20.3 + 3 * 8.9 = 47.0, a 4th would be 55.9 */
      CHECK(p.n_events == 3U && p.air_us <= 55000U);
      tlm_plan(&c, TLM_REPLY_NONE, 0U, 16U, &p);
      CHECK(p.air_us <= 55000U && p.air_us + tlm_air_us(19200U, 17U) > 55000U); }

    REQ("D-34: telemetry is never skipped or delayed for an event; TELEM_SLOW every 10th period");
    { const tlm_cfg_t c = cfg(9600U, 90U);
      unsigned slow = 0U;
      for (uint32_t k = 0U; k < 100U; k++) {
          tlm_plan(&c, TLM_REPLY_NONE, k, 16U, &p);
          CHECK(p.telem == ((k % 10U == 9U) ? TLM_T_SLOW : TLM_T_FAST));
          if (p.telem == TLM_T_SLOW) { slow++; }
      }
      CHECK(slow == 10U); }

    REQ("D-33: CAP_DATA never on the radio");
    CHECK(tlm_cap_data_allowed(TLM_PORT_USB) && !tlm_cap_data_allowed(TLM_PORT_RADIO));

    REQ("REQ-TLM-03, D-34: event queues, depth 16 per port, new event dropped when full, ev_drop saturates");
    { tlm_ev_t e;
      proto_event_t ev;
      tlm_ev_init(&e);
      CHECK(tlm_ev_drop(&e) == 0U && !tlm_ev_peek(&e, TLM_PORT_USB, &ev) && tlm_ev_count(&e, TLM_PORT_RADIO) == 0U);
      for (int32_t k = 0; k < 20; k++) { ev_n(&e, k); }
      CHECK(tlm_ev_count(&e, TLM_PORT_USB) == 16U && tlm_ev_count(&e, TLM_PORT_RADIO) == 16U);
      CHECK(tlm_ev_drop(&e) == 8U);                                /* 4 events x 2 full queues */
      CHECK(tlm_ev_peek(&e, TLM_PORT_RADIO, &ev) && ev.arg == 0 && ev.t_ms == 0U);   /* oldest kept */
      for (int32_t k = 0; k < 16; k++) {                           /* USB drained at once, FIFO, 0..15 */
          CHECK(tlm_ev_peek(&e, TLM_PORT_USB, &ev) && ev.arg == k);
          tlm_ev_pop(&e, TLM_PORT_USB);
      }
      CHECK(!tlm_ev_peek(&e, TLM_PORT_USB, &ev) && tlm_ev_count(&e, TLM_PORT_RADIO) == 16U);
      tlm_ev_pop(&e, TLM_PORT_USB);                                /* pop on empty is harmless */
      tlm_ev_pop(&e, TLM_PORT_RADIO);
      ev_n(&e, 100);                                               /* wrap around: radio has room for one */
      CHECK(tlm_ev_count(&e, TLM_PORT_RADIO) == 16U && tlm_ev_count(&e, TLM_PORT_USB) == 1U);
      for (int32_t k = 1; k < 16; k++) { tlm_ev_pop(&e, TLM_PORT_RADIO); }
      CHECK(tlm_ev_peek(&e, TLM_PORT_RADIO, &ev) && ev.arg == 100);
      for (int32_t k = 0; k < 400; k++) { ev_n(&e, k); }
      CHECK(tlm_ev_drop(&e) == 255U);                              /* saturating */
      CHECK(!tlm_ev_peek(&e, 2U, &ev) && tlm_ev_count(&e, 2U) == 0U); }

    REQ("REQ-TLM-03, §8.4: TELEM_FAST assembly, scaling and saturation");
    { tlm_fast_in_t in = { .t_ms = 1234U, .mode = 3U, .state = 2U, .flags = TLM_F_STILL | TLM_F_CAPTURE,
                           .x_m = 1.2345f, .y_m = -2.0f, .th_rad = PI_F / 2.0f, .v_mps = 0.45f, .w_radps = -0.5f,
                           .duty_l = -0.25f, .duty_r = 0.3f, .seg = 4U, .wp = 9U, .e_y_m = -0.0123f,
                           .gnd_l = 2U, .gnd_r = 5U, .fwd_m = 0.874f };
      uint8_t b[PROTO_PAYLOAD_MAX];
      proto_telem_fast_t m, r;
      CHECK(tlm_fast_payload(&in, b) == 32U && proto_unpack_telem_fast(b, 32U, &r));
      tlm_fast_build(&in, &m);
      CHECK(r.t_ms == 1234U && r.mode == 3U && r.state == 2U && r.flags == 0x0804U && r.x == 1235 && r.y == -2000 &&
            r.th == 9000 && r.v == 450 && r.w == -500 && r.duty_l == -250 && r.duty_r == 300 && r.seg == 4U &&
            r.wp == 9U && r.e_y == -12 && r.gnd == 0x2AU && r.fwd == 87U);
      CHECK(m.x == r.x && m.fwd == r.fwd);
      in.th_rad = 3.0f * PI_F;                                     /* wraps to pi: +18000 */
      tlm_fast_build(&in, &m);
      CHECK(m.th == 18000);
      in.th_rad = -PI_F - 0.001f;
      tlm_fast_build(&in, &m);
      CHECK(m.th > 17900 && m.th <= 18000);
      in.fwd_m = INFINITY; tlm_fast_build(&in, &m); CHECK(m.fwd == 255U);
      in.fwd_m = -1.0f;    tlm_fast_build(&in, &m); CHECK(m.fwd == 255U);
      in.fwd_m = NAN;      tlm_fast_build(&in, &m); CHECK(m.fwd == 255U);
      in.fwd_m = 2.54f;    tlm_fast_build(&in, &m); CHECK(m.fwd == 254U);
      in.fwd_m = 2.55f;    tlm_fast_build(&in, &m); CHECK(m.fwd == 255U);
      in.fwd_m = 0.0f;     tlm_fast_build(&in, &m); CHECK(m.fwd == 0U);
      in.duty_l = 100.0f; in.duty_r = -100.0f; in.v_mps = NAN; in.x_m = 1.0e9f; in.gnd_l = 0xFFU;
      tlm_fast_build(&in, &m);
      CHECK(m.duty_l == 32767 && m.duty_r == -32768 && m.v == 0 && m.x == 2000000000 && (m.gnd & 7U) == 7U); }

    REQ("REQ-TLM-03, §8.4: TELEM_SLOW assembly, ev_drop at offset 34");
    { const tlm_slow_in_t in = { .t_ms = 99999U, .bat_v = 12.1f, .sig_x_m = 0.011f, .sig_y_m = 0.012f,
                                 .sig_th_rad = 0.0087f, .bg_radps = 0.001f, .scale = 0.003f, .imu_t_degc = 36.65f,
                                 .fault_first = 0x0102U, .n_faults = 1U, .cpu = 37U, .ovr_ctrl = 3U, .rx_err = 4U,
                                 .ekf_acc = 500U, .ekf_rej = 6U, .stack_free = 1800U, .vl_alive = 5U, .sens = 3U,
                                 .ev_drop = 12U };
      uint8_t b[PROTO_PAYLOAD_MAX];
      proto_telem_slow_t r;
      CHECK(tlm_slow_payload(&in, b) == 35U && b[34] == 12U && proto_unpack_telem_slow(b, 35U, &r));
      CHECK(r.t_ms == 99999U && r.bat == 12100U && r.sig_x == 11U && r.sig_y == 12U && r.sig_th == 50U &&
            r.bg == 57 && r.scale == 3000 && r.imu_t == 3665 && r.fault_first == 0x0102U && r.n_faults == 1U &&
            r.cpu == 37U && r.ovr_ctrl == 3U && r.rx_err == 4U && r.ekf_acc == 500U && r.ekf_rej == 6U &&
            r.stack_free == 1800U && r.vl_alive == 5U && r.sens == 3U && r.ev_drop == 12U);
      tlm_slow_in_t s = in;
      s.bat_v = -1.0f; s.scale = 1.0f; s.sig_x_m = 1000.0f; s.imu_t_degc = NAN;
      proto_telem_slow_t m;
      tlm_slow_build(&s, &m);
      CHECK(m.bat == 0U && m.scale == 32767 && m.sig_x == 65535U && m.imu_t == 0); }
}
