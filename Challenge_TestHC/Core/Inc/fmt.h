/* fmt.h — float formatting without %f (REQ-COD-03, spec §6.5). Pure. */
#ifndef FMT_H
#define FMT_H

#include <stddef.h>
#include <stdint.h>

/* 1.2345f, 3 -> "1.235"; -0.0004f, 3 -> "-0.000". decimals <= 6. Returns length. */
int fmt_fixed(char *buf, size_t n, float value, uint8_t decimals);

/* Six significant digits, "6.36380e-05". Enough for a #define dump, not bit-exact. */
int fmt_sci(char *buf, size_t n, float value);

#endif
