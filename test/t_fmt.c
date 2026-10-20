#include <stdlib.h>
#include <string.h>

#include "fmt.h"
#include "test.h"

void t_fmt(void)
{
    char b[32];
    REQ("REQ-COD-03 / spec 6.5: fmt_fixed");
    (void)fmt_fixed(b, sizeof b, 1.2345f, 3);  CHECK(strcmp(b, "1.235") == 0);
    (void)fmt_fixed(b, sizeof b, -0.0004f, 3); CHECK(strcmp(b, "-0.000") == 0);
    (void)fmt_fixed(b, sizeof b, 12.0f, 0);    CHECK(strcmp(b, "12") == 0);
    (void)fmt_fixed(b, sizeof b, NAN, 2);      CHECK(strcmp(b, "ovf") == 0);

    REQ("fmt_sci round trip, 6 significant digits");
    static const float v[] = { 6.3638e-5f, 0.45f, 8.98f, -1.275f, 1.0f, 100.0f, 9.99999e3f, 1.0e-10f };
    for (unsigned i = 0; i < sizeof v / sizeof v[0]; i++) {
        (void)fmt_sci(b, sizeof b, v[i]);
        CHECK_NEAR(strtod(b, NULL) / (double)v[i], 1.0, 2e-6);
    }
}
