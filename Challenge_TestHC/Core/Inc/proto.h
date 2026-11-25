/* proto.h — binary protocol codec (spec §7.24, §8.2, §8.4). Pure: no HAL, no main.h.
 * Frame on the wire: COBS( vid | type | seq | payload[0..96] | crc_lo | crc_hi ) 0x00.
 * Every message is serialised byte by byte, little-endian, no struct memcpy (REQ-PRO-01). */
#ifndef PROTO_H
#define PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTO_PAYLOAD_MAX 96U
#define PROTO_RAW_MAX     101U   /* vid type seq payload crc                         */
#define PROTO_ENC_MAX     103U   /* spec §8.2: <= 103 after COBS (102 is reached)    */
#define PROTO_FRAME_MAX   104U   /* encoded frame plus the 0x00 delimiter            */
#define PROTO_VID_BCAST   0xFFU  /* only STOP is honoured with this vid              */
#define PROTO_WIRE(n)     ((n) + 7U)   /* header 3 + CRC 2 + COBS 1 + delimiter 1    */

typedef enum {
    PROTO_T_HELLO = 0x01, PROTO_T_TELEM_FAST = 0x02, PROTO_T_TELEM_SLOW = 0x03, PROTO_T_EVENT = 0x04,
    PROTO_T_ACK = 0x05, PROTO_T_PARAM_VAL = 0x06, PROTO_T_CAP_DATA = 0x07, PROTO_T_TEXT = 0x08,
    PROTO_T_PING = 0x20, PROTO_T_STOP = 0x21, PROTO_T_MODE = 0x22, PROTO_T_DRIVE = 0x23,
    PROTO_T_PARAM_GET = 0x24, PROTO_T_PARAM_SET = 0x25, PROTO_T_CAPTURE = 0x26, PROTO_T_CAP_GET = 0x27,
    PROTO_T_CLEAR = 0x28, PROTO_T_MODE_TEXT = 0x2F
} proto_type_t;

/* Payload lengths of the fixed-size messages (§8.4). */
enum {
    PROTO_LEN_HELLO = 26, PROTO_LEN_TELEM_FAST = 32, PROTO_LEN_TELEM_SLOW = 35, PROTO_LEN_EVENT = 10,
    PROTO_LEN_ACK = 3, PROTO_LEN_PARAM_VAL = 7, PROTO_LEN_MODE = 1, PROTO_LEN_DRIVE = 5,
    PROTO_LEN_PARAM_GET = 2, PROTO_LEN_PARAM_SET = 6, PROTO_LEN_CAPTURE = 4, PROTO_LEN_CAP_GET = 4,
    PROTO_LEN_CAP_HDR = 4
};

/* ---- CRC and COBS ---------------------------------------------------------------- */
/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor. */
uint16_t crc16_ccitt(const uint8_t *d, size_t n);
/* COBS without the delimiter. Returns the encoded length (>= 1), 0 if out_max is too small.
 * A full 254-byte block at the end of the input gets no trailing 0x01. */
size_t cobs_encode(const uint8_t *in, size_t n, uint8_t *out, size_t out_max);
/* Accepts the minimal form and the one with a trailing 0x01. false on a zero byte, a block
 * running past the end or a full out buffer. */
bool   cobs_decode(const uint8_t *in, size_t n, uint8_t *out, size_t out_max, size_t *out_n);

/* ---- frames ---------------------------------------------------------------------- */
typedef struct { uint8_t vid, type, seq, len; uint8_t payload[PROTO_PAYLOAD_MAX]; } proto_frame_t;

/* Encodes with an explicit vid. Returns the length including the 0x00 delimiter,
 * 0 if n > 96 or out_max is too small (PROTO_FRAME_MAX bytes are always enough). */
size_t proto_encode_vid(uint8_t vid, uint8_t type, uint8_t seq, const uint8_t *payload, size_t n,
                        uint8_t *out, size_t out_max);
/* Same with the vehicle ID from par() (com_vid; par_init() must have run). */
size_t proto_encode(uint8_t type, uint8_t seq, const uint8_t *payload, size_t n,
                    uint8_t *out, size_t out_max);

typedef struct {
    uint32_t ok, oversize, cobs, crc, vid, runt, overrun;   /* REQ-PRO-03 drop counters */
} proto_stats_t;

typedef struct {
    uint8_t       buf[PROTO_ENC_MAX];
    uint8_t       n;
    bool          overflow, ready, text_req;
    uint8_t       scan;          /* characters of "#TEXT" matched so far     */
    uint8_t       vid;           /* frames with another vid are dropped      */
    proto_frame_t frame;
    proto_stats_t st;
} proto_rx_t;

void proto_rx_init(proto_rx_t *rx, uint8_t vid);
/* Call after com_vid changes (PARAM_SET), so the receive check follows proto_encode(). */
void proto_rx_set_vid(proto_rx_t *rx, uint8_t vid);
void proto_rx_byte(proto_rx_t *rx, uint8_t b);
/* Copies the pending frame (CRC and vid checked) and clears it. A frame that arrives while
 * one is pending replaces it and counts as overrun. */
bool proto_rx_frame(proto_rx_t *rx, proto_frame_t *f);
/* REQ-PRO-02: the raw stream held "#TEXT" followed by CR or LF. Returns and clears the flag. */
bool proto_rx_text_request(proto_rx_t *rx);

/* ---- messages (§8.4) ------------------------------------------------------------- */
typedef struct {
    uint8_t proto_ver, fw_major, fw_minor, reset_cause; uint32_t git_hash; uint8_t dirty, features;
    char course[8]; uint32_t last_fault_code, last_fault_pc;
} proto_hello_t;
typedef struct {
    uint32_t t_ms; uint8_t mode, state; uint16_t flags; int32_t x, y; int16_t th, v, w, duty_l, duty_r;
    uint8_t seg, wp; int16_t e_y; uint8_t gnd, fwd;
} proto_telem_fast_t;
typedef struct {
    uint32_t t_ms; uint16_t bat, sig_x, sig_y, sig_th; int16_t bg, scale, imu_t; uint16_t fault_first;
    uint8_t n_faults, cpu; uint16_t ovr_ctrl, rx_err, ekf_acc, ekf_rej, stack_free;
    uint8_t vl_alive, sens, ev_drop;
} proto_telem_slow_t;
typedef struct { uint32_t t_ms; uint16_t code; int32_t arg; } proto_event_t;
typedef struct { uint8_t type, seq, result; } proto_ack_t;
typedef struct { uint16_t id; float value; uint8_t result; } proto_param_val_t;
typedef struct { uint16_t index; uint8_t set, n; const uint8_t *rec; } proto_cap_data_t;  /* rec: n records */
typedef struct { int16_t v, w; uint8_t flags; } proto_drive_t;
typedef struct { uint16_t id; float value; } proto_param_set_t;
typedef struct { uint8_t set, decim; uint16_t duration; } proto_capture_t;   /* duration in 0.1 s */
typedef struct { uint16_t from, count; } proto_cap_get_t;

/* pack: writes PROTO_LEN_* bytes (out must hold them), returns the payload length.
 * unpack: false unless n is exactly the message length. */
size_t proto_pack_hello(const proto_hello_t *m, uint8_t *out);
bool   proto_unpack_hello(const uint8_t *p, size_t n, proto_hello_t *m);
size_t proto_pack_telem_fast(const proto_telem_fast_t *m, uint8_t *out);
bool   proto_unpack_telem_fast(const uint8_t *p, size_t n, proto_telem_fast_t *m);
size_t proto_pack_telem_slow(const proto_telem_slow_t *m, uint8_t *out);
bool   proto_unpack_telem_slow(const uint8_t *p, size_t n, proto_telem_slow_t *m);
size_t proto_pack_event(const proto_event_t *m, uint8_t *out);
bool   proto_unpack_event(const uint8_t *p, size_t n, proto_event_t *m);
size_t proto_pack_ack(const proto_ack_t *m, uint8_t *out);
bool   proto_unpack_ack(const uint8_t *p, size_t n, proto_ack_t *m);
size_t proto_pack_param_val(const proto_param_val_t *m, uint8_t *out);
bool   proto_unpack_param_val(const uint8_t *p, size_t n, proto_param_val_t *m);
size_t proto_pack_mode(uint8_t mode, uint8_t *out);
bool   proto_unpack_mode(const uint8_t *p, size_t n, uint8_t *mode);
size_t proto_pack_drive(const proto_drive_t *m, uint8_t *out);
bool   proto_unpack_drive(const uint8_t *p, size_t n, proto_drive_t *m);
size_t proto_pack_param_get(uint16_t id, uint8_t *out);
bool   proto_unpack_param_get(const uint8_t *p, size_t n, uint16_t *id);
size_t proto_pack_param_set(const proto_param_set_t *m, uint8_t *out);
bool   proto_unpack_param_set(const uint8_t *p, size_t n, proto_param_set_t *m);
size_t proto_pack_capture(const proto_capture_t *m, uint8_t *out);
bool   proto_unpack_capture(const uint8_t *p, size_t n, proto_capture_t *m);
size_t proto_pack_cap_get(const proto_cap_get_t *m, uint8_t *out);
bool   proto_unpack_cap_get(const uint8_t *p, size_t n, proto_cap_get_t *m);

/* CAP_DATA (REQ-TLM-02, D-33): record size per set (24, 24, 24, 16; 0 = unknown set) and
 * n_max = floor((96 - 4) / size). pack copies n records of the set's size from m->rec and
 * returns the payload length (4 + n*size); 0 if the set is unknown, n > n_max or out_max is
 * too small (n = 0 is the end frame). unpack points m->rec into p. */
size_t proto_cap_rec_size(uint8_t set);
size_t proto_cap_n_max(uint8_t set);
size_t proto_pack_cap_data(const proto_cap_data_t *m, uint8_t *out, size_t out_max);
bool   proto_unpack_cap_data(const uint8_t *p, size_t n, proto_cap_data_t *m);

/* TEXT: ASCII payload, not terminated, 1..96 bytes. Returns the length; 0 if too long or empty
 * (an empty text is not allowed). */
size_t proto_pack_text(const char *s, size_t n, uint8_t *out);

#endif
