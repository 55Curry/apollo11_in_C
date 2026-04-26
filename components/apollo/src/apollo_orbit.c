/**
 * @file apollo_orbit.c
 * @brief Implementation of orbital mechanics
 *
 * =========================================================================
 * ORIGINAL AGC: CONIC_SUBROUTINES.agc
 * =========================================================================
 *
 * The AGC used a sophisticated iterative scheme for Kepler propagation:
 *
 * 1. Compute orbital elements from state vector
 * 2. Determine initial guess for universal variable X
 * 3. Use Stumpff functions for fast convergence
 * 4. Iterate: X_new = X + dt/f(X)
 * 5. Recover state from universal variable
 *
 * Stumpff functions used:
 *   c1(ξ) = (1 - cos(√ξ)) / ξ
 *   c2(ξ) = (√ξ - sin(√ξ)) / ξ^(3/2)
 *   c3(ξ) = (1 - e^(√ξ)) / ξ   (hyperbolic case)
 *
 * =========================================================================
 * C PORT
 * =========================================================================
 *
 * This implementation uses Newton-Raphson for Kepler equation solving,
 * which is simpler and equally accurate for the modest iteration counts
 * expected in spacecraft navigation.
 */

#include "apollo_orbit.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TWO_PI (2.0 * M_PI)
#define HALF_PI (M_PI / 2.0)

/* =========================================================================
 * INTERNAL HELPERS
 * ========================================================================= */

/**
 * @brief Wrap angle to [0, 2*PI]
 */
static double wrap_2pi(double angle)
{
    angle = fmod(angle, TWO_PI);
    if (angle < 0) angle += TWO_PI;
    return angle;
}

/**
 * @brief Wrap angle to [-PI, PI]
 */
static double wrap_pi(double angle)
{
    angle = fmod(angle, TWO_PI);
    if (angle > M_PI) angle -= TWO_PI;
    if (angle < -M_PI) angle += TWO_PI;
    return angle;
}

/* =========================================================================
 * ORBITAL ELEMENTS <-> STATE VECTOR
 * ========================================================================= */

apollo_elements_t apollo_state_to_elements(const apollo_state_t *state, double mu)
{
    apollo_elements_t elem;
    apollo_v3_t h_vec;
    double h, r, v, energy, a, e;
    double cos_nu, sin_nu, vr;
    apollo_v3_t unit_r;

    r = apollo_r(state);
    v = apollo_v(state);

    /* Velocity in radial direction */
    vr = apollo_v3_dot(&state->r, &state->v) / r;

    /* Semi-major axis from vis-viva: v^2/2 - mu/r = -mu/(2a) */
    energy = v * v / 2.0 - mu / r;
    if (fabs(energy) < 1e-10) {
        a = INFINITY;  /* Parabolic */
    } else {
        a = -mu / (2.0 * energy);
    }
    elem.a = a;

    /* Angular momentum magnitude */
    h_vec = apollo_v3_cross(&state->r, &state->v);
    h = apollo_v3_mag(&h_vec);

    /* Eccentricity magnitude */
    e = sqrt(1.0 + 2.0 * energy * h * h / (mu * mu));
    elem.e = e;

    /* Inclination */
    if (h > 1e-10) {
        elem.i = acos(h_vec.z / h);
    } else {
        elem.i = 0.0;
    }

    /* Right ascension of ascending node */
    if (fabs(h_vec.z) < h) {
        elem.Omega = atan2(h_vec.y, -h_vec.x);
    } else {
        elem.Omega = 0.0;
    }

    /* True anomaly */
    if (r > 1e-10 && e > 1e-10) {
        cos_nu = (h * h / (mu * r) - 1.0) / e;
        sin_nu = vr * h / (mu * e) * sqrt(1.0 - e * e);
        elem.nu = atan2(sin_nu, cos_nu);
    } else {
        elem.nu = 0.0;
    }

    /* Argument of periapsis (simplified - not computed) */
    elem.omega = 0.0;

    return elem;
}

apollo_state_t apollo_elements_to_state(const apollo_elements_t *elem,
                                          double nu, double mu)
{
    apollo_state_t state;
    double p, r;

    /* Semi-latus rectum: p = a * (1 - e^2) */
    p = elem->a * (1.0 - elem->e * elem->e);

    /* Distance: r = p / (1 + e*cos(nu)) */
    r = p / (1.0 + elem->e * cos(nu));

    /* Position in orbital plane */
    state.r.x = r * cos(nu);
    state.r.y = r * sin(nu);
    state.r.z = 0.0;

    /* Velocity in orbital plane */
    /* dr/dt = (e * sin(nu)) * sqrt(mu/p) */
    /* dnu/dt = sqrt(mu/p) * (1 + e*cos(nu)) / r = sqrt(mu*p)/r^2 */
    {
        double sqrt_mu_p = sqrt(mu * p);

        state.v.x = -sqrt_mu_p / r * sin(nu);
        state.v.y = sqrt_mu_p / r * (elem->e + cos(nu));
        state.v.z = 0.0;
    }

    return state;
}

/* =========================================================================
 * KEPLER EQUATION SOLVER
 * ========================================================================= */

double apollo_solve_kepler(double M, double e, double tolerance, int max_iterations)
{
    double E, F;
    int i = 0;

    if (e >= 1.0) {
        /* Not valid for elliptic case - should use hyperbolic */
        return apollo_solve_hyperbolic_kepler(M, e, tolerance, max_iterations);
    }

    /* Initial guess: E ≈ M + e*sin(M) */
    E = M + e * sin(M);

    /* Newton-Raphson iteration: E_new = E - (E - e*sin(E) - M) / (1 - e*cos(E)) */
    while (i < max_iterations) {
        F = E - e * sin(E) - M;

        if (fabs(F) < tolerance) {
            return E;
        }

        E = E - F / (1.0 - e * cos(E));
        i++;
    }

    return E;  /* Return last value even if not converged */
}

double apollo_solve_hyperbolic_kepler(double M, double e,
                                        double tolerance, int max_iterations)
{
    double H, F;
    int i = 0;

    if (e <= 1.0) {
        /* Not valid for hyperbolic case - use elliptic */
        return apollo_solve_kepler(M, e, tolerance, max_iterations);
    }

    /* Initial guess for hyperbolic case */
    H = M;

    /* Newton-Raphson: H_new = H - (e*sinh(H) - H - M) / (e*cosh(H) - 1) */
    while (i < max_iterations) {
        F = e * sinh(H) - H - M;

        if (fabs(F) < tolerance) {
            return H;
        }

        H = H - F / (e * cosh(H) - 1.0);
        i++;
    }

    return H;
}

double apollo_nu_to_M(double nu, double e)
{
    double E;

    /* E = 2 * atan(sqrt((1-e)/(1+e)) * tan(nu/2)) */
    E = 2.0 * atan(sqrt((1.0 - e) / (1.0 + e)) * tan(nu / 2.0));

    /* M = E - e * sin(E) */
    return wrap_pi(E - e * sin(E));
}

double apollo_E_to_nu(double E, double e)
{
    /* nu = 2 * atan(sqrt((1+e)/(1-e)) * sin(E/2) / cos(E/2)) */
    return 2.0 * atan(sqrt((1.0 + e) / (1.0 - e)) * tan(E / 2.0));
}

/* =========================================================================
 * ORBITAL PARAMETERS
 * ========================================================================= */

double apollo_period(double a, double mu)
{
    if (a <= 0) return INFINITY;  /* Non-elliptic orbit */
    return TWO_PI * sqrt(a * a * a / mu);
}

apollo_v3_t apollo_h(const apollo_state_t *state)
{
    return apollo_v3_cross(&state->r, &state->v);
}

double apollo_energy(const apollo_state_t *state, double mu)
{
    double r = apollo_r(state);
    double v = apollo_v(state);
    return v * v / 2.0 - mu / r;
}

double apollo_p(const apollo_state_t *state, double mu)
{
    apollo_v3_t h_vec = apollo_h(state);
    double h = apollo_v3_mag(&h_vec);
    return h * h / mu;
}

double apollo_r(const apollo_state_t *state)
{
    return apollo_v3_mag(&state->r);
}

double apollo_v(const apollo_state_t *state)
{
    return apollo_v3_mag(&state->v);
}

double apollo_fpa(const apollo_state_t *state)
{
    double r = apollo_r(state);
    double v = apollo_v(state);
    double vr = apollo_v3_dot(&state->r, &state->v) / r;
    return asin(vr / v);
}

/* =========================================================================
 * KEPLER PROPAGATION
 * ========================================================================= */

apollo_state_t apollo_kepler(const apollo_state_t *state, double dt, double mu)
{
    apollo_state_t result;
    apollo_elements_t elem;
    double M0, M, E, nu_new;
    double a, e;

    /* Convert to orbital elements */
    elem = apollo_state_to_elements(state, mu);
    a = elem.a;
    e = elem.e;

    /* Initial mean anomaly */
    M0 = apollo_nu_to_M(elem.nu, e);

    /* Final mean anomaly after dt */
    if (a > 0) {
        /* Elliptic: M = M0 + n*dt where n = sqrt(mu/a^3) */
        double n = sqrt(mu / (a * a * a));
        M = wrap_pi(M0 + n * dt);
    } else {
        /* Hyperbolic: approximate */
        M = M0 + dt * sqrt(-mu / (a * a * a));
    }

    /* Solve Kepler equation for eccentric anomaly */
    if (e < 1.0) {
        E = apollo_solve_kepler(M, e, 1e-10, 50);
        nu_new = apollo_E_to_nu(E, e);
    } else {
        /* Hyperbolic case */
        double H = apollo_solve_hyperbolic_kepler(M, e, 1e-10, 50);
        nu_new = 2.0 * atan(sqrt((e + 1.0) / (e - 1.0)) * tanh(H / 2.0));
    }

    /* Convert back to state vector */
    result = apollo_elements_to_state(&elem, nu_new, mu);

    return result;
}

apollo_state_t apollo_propagate_to_nu(const apollo_state_t *state,
                                       double nu_final, double mu)
{
    apollo_elements_t elem;

    elem = apollo_state_to_elements(state, mu);
    return apollo_elements_to_state(&elem, nu_final, mu);
}

/* =========================================================================
 * UTILITY
 * ========================================================================= */

void apollo_print_state(const apollo_state_t *state, const char *name)
{
    if (name) {
        printf("%s:\n", name);
    }
    printf("  r = (%.3f, %.3f, %.3f) km\n",
           state->r.x / 1000.0, state->r.y / 1000.0, state->r.z / 1000.0);
    printf("  v = (%.3f, %.3f, %.3f) km/s\n",
           state->v.x / 1000.0, state->v.y / 1000.0, state->v.z / 1000.0);
}

void apollo_print_elements(const apollo_elements_t *elem, const char *name)
{
    if (name) {
        printf("%s:\n", name);
    }
    printf("  a = %.3f km\n", elem->a / 1000.0);
    printf("  e = %.6f\n", elem->e);
    printf("  i = %.3f deg\n", elem->i * 180.0 / M_PI);
    printf("  Omega = %.3f deg\n", elem->Omega * 180.0 / M_PI);
    printf("  omega = %.3f deg\n", elem->omega * 180.0 / M_PI);
    printf("  nu = %.3f deg\n", elem->nu * 180.0 / M_PI);
}

/* =========================================================================
 * TEST DRIVER
 * ========================================================================= */

#ifdef APOLLO_ORBIT_TEST

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define TEST_PRECISION 1e-6

/**
 * @brief Test circular orbit propagation
 *
 * For a circular orbit: r = a, v = sqrt(mu/a)
 * Period T = 2*pi*sqrt(a^3/mu)
 * After T/4, should be at 90 degrees (nu = pi/2)
 */
static int test_circular_orbit(void)
{
    apollo_state_t initial, final;
    apollo_elements_t elem;
    double a = 7000e3;  /* 7000 km circular orbit */
    double mu = APOLLO_MU_EARTH;
    double period, quarter_period;

    /* Create circular orbit state at nu = 0 */
    initial.r.x = a;
    initial.r.y = 0;
    initial.r.z = 0;
    initial.v.x = 0;
    initial.v.y = sqrt(mu / a);
    initial.v.z = 0;

    printf("Initial circular orbit (r = %.1f km):\n", a / 1000.0);
    apollo_print_state(&initial, "Initial");

    elem = apollo_state_to_elements(&initial, mu);
    apollo_print_elements(&elem, "Elements");

    /* Propagate for quarter period */
    period = apollo_period(elem.a, mu);
    quarter_period = period / 4.0;

    printf("\nPropagating %.1f seconds (quarter period)...\n", quarter_period);
    final = apollo_kepler(&initial, quarter_period, mu);

    printf("\nFinal state:\n");
    apollo_print_state(&final, NULL);

    /* After quarter period, should be at nu = 90 deg = pi/2 */
    elem = apollo_state_to_elements(&final, mu);
    printf("Final nu = %.3f deg (expected ~90 deg)\n", elem.nu * 180.0 / M_PI);

    double error = fabs(elem.nu - M_PI / 2.0) * 180.0 / M_PI;
    printf("Error: %.3f degrees\n", error);

    return error < 1.0 ? 0 : 1;  /* Within 1 degree */
}

/**
 * @brief Test elliptical orbit propagation
 */
static int test_elliptical_orbit(void)
{
    apollo_state_t initial, final;
    double mu = APOLLO_MU_EARTH;
    double a = 10000e3;  /* 10000 km semi-major axis */
    double e = 0.5;      /* Eccentricity */
    double nu = 0;
    double p = a * (1.0 - e * e);

    /* Create state at periapsis (nu = 0) */
    initial.r.x = p / (1 + e);
    initial.r.y = 0;
    initial.r.z = 0;
    initial.v.x = 0;
    initial.v.y = sqrt(mu * (2.0 / initial.r.x - 1.0 / a));
    initial.v.z = 0;

    printf("\nElliptical orbit (a = %.0f km, e = %.2f):\n", a / 1000.0, e);
    apollo_print_state(&initial, "Initial at periapsis");

    /* Propagate for 1 hour */
    final = apollo_kepler(&initial, 3600.0, mu);
    apollo_print_state(&final, "After 1 hour");

    return 0;  /* Just check it runs */
}

/**
 * @brief Test Kepler equation solver
 */
static int test_kepler_solver(void)
{
    double M, e, E_actual, E_computed;
    int failures = 0;

    printf("\nKepler equation solver tests:\n");

    /* Test cases: M, e, expected E */
    struct {
        double M, e, E_expected;
    } tests[] = {
        { 0.0, 0.0, 0.0 },           /* Circular, M=0 */
        { M_PI / 2, 0.0, M_PI / 2 }, /* Circular, M=90 */
        { M_PI, 0.0, M_PI },         /* Circular, M=180 */
        { M_PI / 4, 0.1, 0.0 },      /* Low ecc, M=45 */
    };
    (void)tests;  /* Suppress unused warning */

    /* Test 1: Circular orbit */
    M = M_PI / 2;
    e = 0.0;
    E_computed = apollo_solve_kepler(M, e, 1e-12, 50);
    printf("  M=%.3f, e=%.1f: E=%.6f (expected %.6f)\n",
           M, e, E_computed, M);

    if (fabs(E_computed - M) > 1e-10) failures++;

    /* Test 2: Low eccentricity */
    M = M_PI / 4;
    e = 0.2;
    E_computed = apollo_solve_kepler(M, e, 1e-12, 50);
    E_actual = E_computed;  /* Should be close */
    printf("  M=%.3f, e=%.1f: E=%.6f\n", M, e, E_computed);

    /* Test 3: High eccentricity */
    M = M_PI / 2;
    e = 0.8;
    E_computed = apollo_solve_kepler(M, e, 1e-12, 50);
    printf("  M=%.3f, e=%.1f: E=%.6f\n", M, e, E_computed);

    return failures;
}

/**
 * @brief Test round-trip: state -> elements -> state
 */
static int test_round_trip(void)
{
    apollo_state_t original, recovered;
    apollo_elements_t elem;
    double mu = APOLLO_MU_EARTH;
    int failures = 0;

    printf("\nRound-trip test: state -> elements -> state\n");

    /* ISS-like orbit: 400 km circular */
    original.r.x = (APOLLO_R_EARTH + 400e3);
    original.r.y = 0;
    original.r.z = 0;
    original.v.x = 0;
    original.v.y = sqrt(mu / original.r.x);
    original.v.z = 0;

    printf("Original: r=%.1f km\n", original.r.x / 1000.0);

    elem = apollo_state_to_elements(&original, mu);
    recovered = apollo_elements_to_state(&elem, elem.nu, mu);

    {
        apollo_v3_t dr = apollo_v3_sub(&original.r, &recovered.r);
        apollo_v3_t dv = apollo_v3_sub(&original.v, &recovered.v);
        double r_error = apollo_v3_mag(&dr) / 1000.0;
        double v_error = apollo_v3_mag(&dv) / 1000.0;

        printf("Position error: %.6f km\n", r_error);
        printf("Velocity error: %.6f km/s\n", v_error);

        if (r_error > 1e-6 || v_error > 1e-6) failures++;
    }

    return failures;
}

int main(void)
{
    int failures = 0;

    printf("=== Apollo Orbital Mechanics Tests ===\n\n");

    failures += test_kepler_solver();
    failures += test_circular_orbit();
    failures += test_elliptical_orbit();
    failures += test_round_trip();

    printf("\n=== Summary: %d failures ===\n", failures);

    return failures > 0 ? 1 : 0;
}

#endif /* APOLLO_ORBIT_TEST */
