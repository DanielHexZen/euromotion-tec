/* test.h — 30-line assertion harness (REQ-TST-01). Every test names its REQ ID in REQ(). */
#ifndef TEST_H
#define TEST_H

#include <math.h>
#include <stdio.h>

extern int g_checks, g_fails;

#define REQ(id) printf("  %s\n", id)
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fails++; \
    printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { g_checks++; const double a_ = (double)(a), b_ = (double)(b); \
    if (!(fabs(a_ - b_) <= (double)(tol))) { g_fails++; \
    printf("    FAIL %s:%d  %s=%.9g vs %s=%.9g (tol %g)\n", __FILE__, __LINE__, #a, a_, #b, b_, \
           (double)(tol)); } } while (0)

#endif
