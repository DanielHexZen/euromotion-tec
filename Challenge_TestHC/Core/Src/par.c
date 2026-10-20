/* par.c — spec §7.4 (REQ-PAR-01..05). */
#include "par.h"

#include <stdio.h>

#include "fmt.h"

const par_t PAR_DEFAULT = {
#define X(id, name, def, unit, lo, hi, flags) .name = def,
    PAR_TABLE(X)
#undef X
};

static const par_desc_t DESC[] = {
#define X(id, name, def, unit, lo, hi, flags) { id, #name, unit, def, lo, hi, flags },
    PAR_TABLE(X)
#undef X
};
#define N_PAR (sizeof DESC / sizeof DESC[0])

_Static_assert(sizeof(par_t) == N_PAR * sizeof(float), "par_t must be N floats, no padding");

static par_t    g_par;
static uint32_t g_gen;
static bool     g_stationary;

/* Field i of par_t, in table order. The struct is N floats (asserted above). */
static float *field(size_t i) { return &((float *)&g_par)[i]; }

static const par_desc_t *lookup(uint16_t id, size_t *idx)
{
    for (size_t i = 0; i < N_PAR; i++) {
        if (DESC[i].id == id) { *idx = i; return &DESC[i]; }
    }
    return NULL;
}

st_t par_init(void)
{
    g_par = PAR_DEFAULT;
    g_gen = 0U;
    g_stationary = true;
    for (size_t i = 0; i < N_PAR; i++) {
        const float v = *field(i);
        if (!(v >= DESC[i].min && v <= DESC[i].max)) { return ST_RANGE; }
    }
    return ST_OK;
}

const par_t *par(void) { return &g_par; }
uint32_t par_generation(void) { return g_gen; }
void par_set_stationary(bool stationary) { g_stationary = stationary; }

st_t par_set(uint16_t id, float value)
{
    size_t i;
    const par_desc_t *d = lookup(id, &i);
    if (d == NULL || !(value >= d->min && value <= d->max)) { return ST_RANGE; }
    if ((d->flags & PAR_S) != 0U && !g_stationary) { return ST_STATE; }
    *field(i) = value;
    g_gen++;
    return ST_OK;
}

st_t par_get(uint16_t id, float *value)
{
    size_t i;
    if (lookup(id, &i) == NULL) { return ST_RANGE; }
    *value = *field(i);
    return ST_OK;
}

const par_desc_t *par_desc(uint16_t id)
{
    size_t i;
    return lookup(id, &i);
}

const par_desc_t *par_find(const char *name)
{
    for (size_t i = 0; i < N_PAR; i++) {
        const char *a = DESC[i].name;
        const char *b = name;
        while (*a != '\0' && *a == *b) { a++; b++; }
        if (*a == '\0' && *b == '\0') { return &DESC[i]; }
    }
    return NULL;
}

size_t par_count(void) { return N_PAR; }
const par_desc_t *par_desc_at(size_t i) { return (i < N_PAR) ? &DESC[i] : NULL; }

int par_format_define(size_t i, char *buf, size_t n)
{
    if (i >= N_PAR) { return 0; }
    char num[20];
    (void)fmt_sci(num, sizeof num, *field(i));
    return snprintf(buf, n, "#define PAR_%s %sf", DESC[i].name, num);
}
