#include <stdio.h>

#include "test.h"

int g_checks, g_fails;

void t_crc(void);
void t_fmt(void);
void t_par(void);
void t_prof(void);
void t_whl_law(void);
void t_drv_move(void);
void t_con(void);

int main(void)
{
    static const struct { const char *name; void (*fn)(void); } T[] = {
        { "t_crc", t_crc }, { "t_fmt", t_fmt }, { "t_par", t_par }, { "t_prof", t_prof },
        { "t_whl_law", t_whl_law }, { "t_drv_move", t_drv_move }, { "t_con", t_con },
    };
    for (unsigned i = 0; i < sizeof T / sizeof T[0]; i++) {
        printf("%s\n", T[i].name);
        T[i].fn();
    }
    printf("%d checks, %d failed\n", g_checks, g_fails);
    return g_fails != 0;
}
