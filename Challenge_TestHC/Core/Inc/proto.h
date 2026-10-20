/* proto.h — binary protocol codec (spec §7.24). Pure. Phase 0 carries only the CRC that
 * t_crc already covers; COBS framing and the message packers arrive with phase 4. */
#ifndef PROTO_H
#define PROTO_H

#include <stddef.h>
#include <stdint.h>

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor. */
uint16_t crc16_ccitt(const uint8_t *d, size_t n);

#endif
