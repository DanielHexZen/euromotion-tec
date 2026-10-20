/* fmt.c — spec §6.5. */
#include "fmt.h"

#include <math.h>
#include <stdio.h>

static const uint32_t P10[] = { 1U, 10U, 100U, 1000U, 10000U, 100000U, 1000000U };

int fmt_fixed(char *buf, size_t n, float value, uint8_t decimals)
{
    if (decimals > 6U) { decimals = 6U; }
    const float scaled = fabsf(value) * (float)P10[decimals];
    if (!(scaled < 2.0e9f)) { return snprintf(buf, n, "ovf"); }   /* also NaN */
    const uint32_t q = (uint32_t)(scaled + 0.5f);
    const char *sign = (value < 0.0f) ? "-" : "";
    if (decimals == 0U) { return snprintf(buf, n, "%s%lu", sign, (unsigned long)q); }
    return snprintf(buf, n, "%s%lu.%0*lu", sign, (unsigned long)(q / P10[decimals]),
                    (int)decimals, (unsigned long)(q % P10[decimals]));
}

int fmt_sci(char *buf, size_t n, float value)
{
    if (value == 0.0f) { return snprintf(buf, n, "0.0"); }
    if (!isfinite(value)) { return snprintf(buf, n, "nan"); }
    const float a = fabsf(value);
    int e = (int)floorf(log10f(a));
    uint32_t q = (uint32_t)(a / powf(10.0f, (float)e) * 1.0e5f + 0.5f);   /* 100000..999999 */
    if (q >= 1000000U) { q = 100000U; e++; }         /* rounded up to the next decade */
    else if (q < 100000U) { q = 999999U; e--; }      /* log10f landed one decade high */
    return snprintf(buf, n, "%s%lu.%05lue%+03d", (value < 0.0f) ? "-" : "",
                    (unsigned long)(q / 100000U), (unsigned long)(q % 100000U), e);
}
