/*
 * test.h - minimal unit test helpers.
 *
 *   CHECK(cond)              fail if cond is false
 *   CHECK_NEAR(a, b, tol)    fail if |a - b| > tol
 *   run_tests(cases, n)      run each case, print PASS/FAIL, return exit code
 */
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int test_failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                               \
            test_failures++;                                                                       \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                      \
    do {                                                                                           \
        double a_ = (a), b_ = (b);                                                                 \
        if (!(fabs(a_ - b_) <= (tol))) {                                                           \
            printf("  FAIL %s:%d: %s = %g, expected %g +/- %g\n", __FILE__, __LINE__, #a, a_, b_,  \
                (double)(tol));                                                                    \
            test_failures++;                                                                       \
        }                                                                                          \
    } while (0)

typedef struct {
    const char *name;
    void (*fn)(void);
} test_case;
#define T(fn)                                                                                      \
    {                                                                                              \
        #fn, fn                                                                                    \
    }

static inline int run_tests(const test_case *cases, int n)
{
    for (int i = 0; i < n; i++) {
        int before = test_failures;
        cases[i].fn();
        printf("%s %s\n", test_failures == before ? "PASS" : "FAIL", cases[i].name);
    }
    return test_failures ? 1 : 0;
}
