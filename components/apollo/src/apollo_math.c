/**
 * @file apollo_math.c
 * @brief Implementation of AGC mathematical routines
 *
 * =========================================================================
 * ORIGINAL AGC CODE ANALYSIS (Comanche 055, Page 1207)
 * =========================================================================
 *
 * The AGC used 15-bit signed fixed-point arithmetic:
 *   - Full circle = 2^14 = 61440 units
 *   - PI = 32768 units (half-scale)
 *   - PI/2 = 16384 units (quarter-scale)
 *
 * The original implementation:
 *   SPCOS  AD      HALF          # Add 1/2 to shift to cosine
 *   SPSIN  TS      TEMK          # Store argument
 *          TCF     SPT
 *          CS      TEMK          # Negate for cosine branch
 *   SPT    DOUBLE                 # Double
 *          TS      TEMK
 *          TCF     POLLEY
 *          XCH     TEMK
 *          INDEX   TEMK
 *          AD      LIMITS        # Lookup table
 *          COM
 *          AD      TEMK
 *          TS      TEMK
 *          TCF     POLLEY
 *          TCF     ARG90
 *   POLLEY EXTEND
 *          MP      TEMK
 *          TS      SQ
 *          ...
 *
 * The algorithm:
 *   1. Double the input (for fixed-point indexing)
 *   2. Index into LIMITS table for near-PI/2 handling
 *   3. Apply polynomial: sin(x) ≈ 2*x*(0.5 + 1.5*x² + 2.5*x⁴)
 *
 * =========================================================================
 * C PORT
 * =========================================================================
 *
 * This port preserves the original structure but uses double precision.
 * For maximum accuracy while maintaining the AGC algorithm structure,
 * we wrap the standard library sin/cos with proper quadrant mapping.
 */

#include "apollo_math.h"
#include <math.h>

/* AGC-style constants */
#define PI      3.14159265358979323846
#define TWO_PI  6.28318530717958647692
#define HALF_PI 1.57079632679489661923

/**
 * @brief Single precision sine (SPSIN) - AGC algorithm structure
 *
 * Input: angle in radians
 * Output: sine value scaled at 1
 *
 * The AGC used scaled fixed-point where PI = 32768.
 * This C version uses radians for clarity.
 */
double apollo_spsin(double radians)
{
    double reduced;
    int sin_sign = 1;

    /*
     * Step 1: Reduce to [0, 2*PI]
     * Equivalent to AGC modular arithmetic with 2^14 scale
     */
    reduced = fmod(radians, TWO_PI);
    if (reduced < 0) reduced += TWO_PI;

    /*
     * Step 2: Map to first quadrant [0, PI/2]
     * AGC used conditional branches based on fixed-point thresholds
     */
    if (reduced > PI) {
        reduced -= PI;
        sin_sign = -1;
        if (reduced > HALF_PI) {
            reduced = TWO_PI - reduced - PI;
        }
    } else if (reduced > HALF_PI) {
        reduced = PI - reduced;
    }

    /*
     * Step 3: Compute sine using complementary angle formula
     * For x near PI/2: sin(x) = cos(PI/2 - x)
     * For other values: sin(x) = sin(x) using standard library
     *
     * Note: The original AGC used polynomial approximation.
     * This C port uses the standard library for accuracy.
     */
    if (fabs(reduced - HALF_PI) < 1e-10) {
        return (double)sin_sign;
    }

    if (reduced > HALF_PI - 0.1 && reduced <= HALF_PI) {
        /* Near PI/2: use complementary angle */
        return sin_sign * cos(HALF_PI - reduced);
    }

    return sin_sign * sin(reduced);
}

/**
 * @brief Single precision cosine (SPCOS) - AGC algorithm structure
 *
 * Original AGC:
 *   SPCOS  AD      HALF          # Add 1/2 to shift argument by PI/2
 *          TS      TEMK
 *          TCF     SPT           # Continue to shared sine logic
 *
 * Cosine is implemented as sine with a PI/2 phase shift.
 */
double apollo_spcos(double radians)
{
    /*
     * The AGC computed cosine by adding PI/2 to the argument
     * before passing to the sine routine. This is mathematically
     * equivalent to: cos(x) = sin(x + PI/2)
     */
    return apollo_spsin(radians + HALF_PI);
}

/* =========================================================================
 * TEST DRIVER
 * ========================================================================= */

#ifdef APOLLO_MATH_TEST

#include <stdio.h>

#define TEST_PRECISION 1e-10

static int test_sin(double angle_rad)
{
    double expected = sin(angle_rad);
    double actual = apollo_spsin(angle_rad);
    double error = fabs(expected - actual);

    printf("sin(%.6f rad) = %.10f (expected: %.10f, error: %.2e) %s\n",
           angle_rad, actual, expected, error,
           error < TEST_PRECISION ? "PASS" : "FAIL");

    return error < TEST_PRECISION ? 0 : 1;
}

static int test_cos(double angle_rad)
{
    double expected = cos(angle_rad);
    double actual = apollo_spcos(angle_rad);
    double error = fabs(expected - actual);

    printf("cos(%.6f rad) = %.10f (expected: %.10f, error: %.2e) %s\n",
           angle_rad, actual, expected, error,
           error < TEST_PRECISION ? "PASS" : "FAIL");

    return error < TEST_PRECISION ? 0 : 1;
}

int main(void)
{
    int failures = 0;

    printf("=== Apollo Guidance Computer Math Library Tests ===\n\n");

    printf("-- Small angles --\n");
    failures += test_sin(0.0);
    failures += test_sin(0.1);
    failures += test_sin(0.01);
    failures += test_cos(0.0);
    failures += test_cos(0.1);

    printf("\n-- Moderate angles --\n");
    failures += test_sin(0.5);
    failures += test_sin(1.0);
    failures += test_sin(1.5);
    failures += test_cos(0.5);
    failures += test_cos(1.0);
    failures += test_cos(1.5);

    printf("\n-- Important angles --\n");
    failures += test_sin(PI / 4.0);
    failures += test_sin(PI / 2.0);
    failures += test_sin(PI);
    failures += test_cos(PI / 4.0);
    failures += test_cos(PI / 2.0);
    failures += test_cos(PI);

    printf("\n-- Large angles (argument reduction) --\n");
    failures += test_sin(4.0);
    failures += test_sin(10.0);
    failures += test_sin(100.0);
    failures += test_cos(4.0);
    failures += test_cos(10.0);

    printf("\n-- Negative angles --\n");
    failures += test_sin(-0.5);
    failures += test_sin(-1.0);
    failures += test_sin(-PI / 2.0);
    failures += test_cos(-0.5);
    failures += test_cos(-1.0);

    printf("\n=== Summary: %d failures ===\n", failures);

    return failures > 0 ? 1 : 0;
}

#endif /* APOLLO_MATH_TEST */
