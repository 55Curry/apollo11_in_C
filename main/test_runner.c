/**
 * @file test_runner.c
 * @brief Test runner for Apollo-11 on ESP32-C3
 */

#include <stdio.h>
#include <math.h>
#include "apollo_math.h"
#include "apollo_vecmath.h"
#include "apollo_orbit.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Math tests */
int test_math_suite(void)
{
    int failures = 0;
    printf("\n[TEST] Math Suite\n");

    double test_vals[] = {0.0, M_PI/6.0, M_PI/4.0, M_PI/2.0, M_PI, 2.0*M_PI};
    double expected_sin[] = {0.0, 0.5, 0.707106781, 1.0, 0.0, 0.0};

    for (int i = 0; i < 6; i++) {
        double actual = apollo_spsin(test_vals[i]);
        double exp = expected_sin[i];
        if (fabs(actual - exp) > 1e-6) {
            printf("  FAIL: apollo_spsin(%.4f) = %.6f, expected %.6f\n", test_vals[i], actual, exp);
            failures++;
        }
    }

    if (failures == 0) printf("  PASSED\n");
    return failures;
}

/* Vecmath tests */
int test_vecmath_suite(void)
{
    int failures = 0;
    printf("\n[TEST] Vecmath Suite\n");

    apollo_v3_t v1 = {1, 2, 3};
    apollo_v3_t v2 = {4, 5, 6};
    apollo_v3_t v_sum = apollo_v3_add(&v1, &v2);

    if (v_sum.x != 5 || v_sum.y != 7 || v_sum.z != 9) {
        printf("  FAIL: apollo_v3_add\n");
        failures++;
    }

    double dot = apollo_v3_dot(&v1, &v2);
    if (fabs(dot - 32.0) > 1e-10) {
        printf("  FAIL: apollo_v3_dot = %.2f, expected 32.0\n", dot);
        failures++;
    }

    apollo_v3_t vx = {1, 0, 0};
    apollo_v3_t vy = {0, 1, 0};
    apollo_v3_t vz = apollo_v3_cross(&vx, &vy);
    if (fabs(vz.z - 1.0) > 1e-10) {
        printf("  FAIL: apollo_v3_cross z = %.2f, expected 1.0\n", vz.z);
        failures++;
    }

    if (failures == 0) printf("  PASSED\n");
    return failures;
}

/* Orbit tests */
int test_orbit_suite(void)
{
    int failures = 0;
    printf("\n[TEST] Orbit Suite\n");

    double mu = APOLLO_MU_EARTH;
    double Re = APOLLO_R_EARTH;

    apollo_state_t initial;
    initial.r.x = Re + 400000.0;
    initial.r.y = 0;
    initial.r.z = 0;
    initial.v.x = 0;
    initial.v.y = sqrt(mu / initial.r.x);
    initial.v.z = 0;

    apollo_elements_t elem = apollo_state_to_elements(&initial, mu);

    if (elem.e > 1e-6) {
        printf("  FAIL: eccentricity = %.8f, expected ~0\n", elem.e);
        failures++;
    }

    double M = M_PI / 4.0;
    double e = 0.1;
    double E = apollo_solve_kepler(M, e, 1e-12, 50);
    double M_check = E - e * sin(E);
    if (fabs(M_check - M) > 1e-10) {
        printf("  FAIL: Kepler solver M_check = %.10f, expected %.10f\n", M_check, M);
        failures++;
    }

    if (failures == 0) printf("  PASSED\n");
    return failures;
}
