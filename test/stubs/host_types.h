/* Host stand-in for the CubeMX main.h (the Makefile defines __MAIN_H to blank the real one):
 * just enough types for con.c and its headers. */
#ifndef HOST_TYPES_H
#define HOST_TYPES_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
#endif
