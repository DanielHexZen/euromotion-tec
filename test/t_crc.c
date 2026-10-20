#include "proto.h"
#include "test.h"

void t_crc(void)
{
    REQ("proto: CRC-16/CCITT-FALSE");
    const uint8_t s[] = "123456789";
    CHECK(crc16_ccitt(s, 9U) == 0x29B1U);
    CHECK(crc16_ccitt(s, 0U) == 0xFFFFU);
}
