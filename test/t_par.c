#include <string.h>

#include "par.h"
#include "test.h"

void t_par(void)
{
    REQ("REQ-PAR-01/02: every default inside its range, IDs and names unique and ascending");
    CHECK(par_init() == ST_OK);
    for (size_t i = 0; i < par_count(); i++) {
        const par_desc_t *a = par_desc_at(i);
        CHECK(a->def >= a->min && a->def <= a->max);
        if (i > 0) { CHECK(par_desc_at(i - 1)->id < a->id); }
        for (size_t j = i + 1; j < par_count(); j++) { CHECK(strcmp(a->name, par_desc_at(j)->name) != 0); }
        float v;
        CHECK(par_get(a->id, &v) == ST_OK && v == a->def);
    }

    REQ("REQ-PAR-03: out of range / wrong mode change nothing");
    float v;
    const uint32_t g0 = par_generation();
    CHECK(par_set(113, 11.0f) == ST_RANGE);
    CHECK(par_set(113, NAN) == ST_RANGE);
    CHECK(par_set(9999, 1.0f) == ST_RANGE);
    (void)par_get(113, &v); CHECK(v == 2.0f);
    par_set_stationary(false);
    CHECK(par_set(201, 1.01f) == ST_STATE);          /* PAR_S while moving */
    CHECK(par_set(113, 3.0f) == ST_OK);              /* PAR_L is live      */
    par_set_stationary(true);
    CHECK(par_set(201, 1.01f) == ST_OK);
    (void)par_get(201, &v); CHECK(v == 1.01f);
    CHECK(par()->odo_scale_l == 1.01f);

    REQ("REQ-PAR-04: generation counts successful sets");
    CHECK(par_generation() == g0 + 2U);

    REQ("lookup by name and the #define dump");
    CHECK(par_find("whl_kp_x") != NULL && par_find("whl_kp_x")->id == 113);
    CHECK(par_find("nope") == NULL && par_find("whl_kp") == NULL);
    char line[64];
    (void)par_init();
    (void)par_format_define(0, line, sizeof line);
    CHECK(strcmp(line, "#define PAR_mot_d_max 1.00000e+00f") == 0);
}
