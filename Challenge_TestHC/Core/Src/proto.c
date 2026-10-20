/* proto.c — spec §7.24 (CRC only until phase 4). */
#include "proto.h"

uint16_t crc16_ccitt(const uint8_t *d, size_t n)
{
    uint32_t crc = 0xFFFFU;
    for (size_t i = 0U; i < n; i++) {
        crc ^= (uint32_t)d[i] << 8;
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = ((crc & 0x8000U) != 0U) ? ((crc << 1) ^ 0x1021U) : (crc << 1);
        }
    }
    return (uint16_t)(crc & 0xFFFFU);
}
