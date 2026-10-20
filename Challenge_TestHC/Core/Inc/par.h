/* par.h — parameters (spec §7.4). Pure. */
#ifndef PAR_H
#define PAR_H

#include <stddef.h>
#include <stdint.h>

#include "par_table.h"
#include "units.h"

typedef struct {
#define X(id, name, def, unit, lo, hi, flags) float name;
    PAR_TABLE(X)
#undef X
} par_t;

typedef struct {
    uint16_t    id;
    const char *name;
    const char *unit;
    float       def, min, max;
    uint8_t     flags;
} par_desc_t;

extern const par_t PAR_DEFAULT;

/* Init only. Copies the defaults and validates them; ST_RANGE -> caller raises F_PARAM. */
st_t              par_init(void);
const par_t      *par(void);                          /* any context; read-only        */
uint32_t          par_generation(void);               /* +1 on every successful set    */
/* Main loop. PAR_S parameters are accepted only while stationary (see par_set_stationary). */
st_t              par_set(uint16_t id, float value);  /* ST_RANGE / ST_STATE: no change */
st_t              par_get(uint16_t id, float *value);
void              par_set_stationary(bool stationary);
const par_desc_t *par_desc(uint16_t id);              /* NULL if unknown               */
const par_desc_t *par_find(const char *name);         /* NULL if unknown               */
size_t            par_count(void);
const par_desc_t *par_desc_at(size_t i);              /* ascending ID order            */
/* "#define PAR_<name> <value>" for entry i (the L dump, one line at a time so the
 * console can emit it in chunks, REQ-CON-01). Returns the line length. */
int               par_format_define(size_t i, char *buf, size_t n);

#endif
