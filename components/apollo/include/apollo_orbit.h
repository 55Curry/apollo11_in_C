/**
 * @file apollo_orbit.h
 * @brief Orbital mechanics for Apollo Guidance Computer
 *
 * =========================================================================
 * ORIGINAL AGC: CONIC_SUBROUTINES.agc (Pages 1262-1308)
 * =========================================================================
 *
 * This module implements conic trajectory calculations:
 *
 *   KEPLER  - Propagate state vector along conic trajectory
 *   LAMBERT - Solve boundary value problem for intercept
 *   DELTIME - Time calculation for Kepler equation
 *
 * The AGC used iterative methods with Stumpff functions for efficiency.
 *
 * =========================================================================
 * C PORT
 * =========================================================================
 *
 * This implementation uses modern numerical methods while preserving
 * the original algorithm structure:
 *
 *   - Newton-Raphson iteration for Kepler equation
 *   - Stumpff functions for series acceleration
 *   - Support for all conic sections (elliptic, parabolic, hyperbolic)
 *
 * Reference: Vallado, "Fundamentals of Astrodynamics and Applications"
 */

#ifndef APOLLO_ORBIT_H
#define APOLLO_ORBIT_H

#include "apollo_vecmath.h"

/* =========================================================================
 * GRAVITATIONAL CONSTANTS
 * ========================================================================= */

/**
 * @name Gravitational parameters (m^3/s^2)
 * @{
 */

/** @brief Earth gravitational parameter */
#define APOLLO_MU_EARTH  3.986004418e14

/** @brief Moon gravitational parameter */
#define APOLLO_MU_MOON   4.9048695e12

/** @brief Earth radius (meters) */
#define APOLLO_R_EARTH   6.378137e6

/** @brief Moon radius (meters) */
#define APOLLO_R_MOON    1.7371e6

/** @} */

/* =========================================================================
 * STATE VECTOR
 * ========================================================================= */

/**
 * @brief Orbital state vector (position and velocity)
 */
typedef struct {
    apollo_v3_t r;  /**< Position vector (meters) */
    apollo_v3_t v;  /**< Velocity vector (meters/second) */
} apollo_state_t;

/**
 * @brief Orbital elements
 */
typedef struct {
    double a;   /**< Semi-major axis (meters) */
    double e;   /**< Eccentricity (dimensionless) */
    double i;   /**< Inclination (radians) */
    double Omega; /**< Right ascension of ascending node (radians) */
    double omega; /**< Argument of periapsis (radians) */
    double nu;    /**< True anomaly (radians) */
} apollo_elements_t;

/**
 * @brief Central body selection
 */
typedef enum {
    APOLLO_BODY_EARTH,
    APOLLO_BODY_MOON
} apollo_body_t;

/* =========================================================================
 * KEPLER PROPAGATION
 * ========================================================================= */

/**
 * @brief Propagate state vector by time using Kepler equation
 *
 * Given an initial state vector and a time, this computes the new
 * state vector after that time along the same conic trajectory.
 *
 * The trajectory can be:
 *   - Elliptic (e < 1)
 *   - Parabolic (e = 1)
 *   - Hyperbolic (e > 1)
 *
 * @param state Initial state vector
 * @param dt Time to propagate (seconds)
 * @param mu Gravitational parameter (m^3/s^2)
 * @return Final state vector
 *
 * Example:
 * @code
 * apollo_state_t initial = { .r = {7e6, 0, 0}, .v = {0, 7.5e3, 0} };
 * apollo_state_t final = apollo_kepler(initial, 3600.0, APOLLO_MU_EARTH);
 * @endcode
 */
apollo_state_t apollo_kepler(const apollo_state_t *state, double dt, double mu);

/**
 * @brief Propagate state vector to a specific true anomaly
 *
 * @param state Initial state vector
 * @param nu_final Final true anomaly (radians)
 * @param mu Gravitational parameter
 * @return Final state vector
 */
apollo_state_t apollo_propagate_to_nu(const apollo_state_t *state,
                                       double nu_final, double mu);

/* =========================================================================
 * ORBITAL ELEMENTS
 * ========================================================================= */

/**
 * @brief Convert state vector to orbital elements
 *
 * @param state State vector
 * @param mu Gravitational parameter
 * @return Orbital elements
 */
apollo_elements_t apollo_state_to_elements(const apollo_state_t *state, double mu);

/**
 * @brief Convert orbital elements to state vector
 *
 * @param elements Orbital elements
 * @param nu True anomaly (radians)
 * @param mu Gravitational parameter
 * @return State vector
 */
apollo_state_t apollo_elements_to_state(const apollo_elements_t *elements,
                                          double nu, double mu);

/* =========================================================================
 * KEPLER EQUATION SOLVER
 * ========================================================================= */

/**
 * @brief Solve Kepler's equation: M = E - e*sin(E)
 *
 * Given mean anomaly M and eccentricity e, solve for eccentric anomaly E.
 * Uses Newton-Raphson iteration.
 *
 * @param M Mean anomaly (radians)
 * @param e Eccentricity
 * @param tolerance Convergence tolerance
 * @param max_iterations Maximum iterations
 * @return Eccentric anomaly (radians)
 */
double apollo_solve_kepler(double M, double e, double tolerance, int max_iterations);

/**
 * @brief Solve hyperbolic Kepler's equation: M = e*sinh(H) - H
 *
 * @param M Mean anomaly (radians)
 * @param e Eccentricity (> 1)
 * @param tolerance Convergence tolerance
 * @param max_iterations Maximum iterations
 * @return Hyperbolic eccentric anomaly
 */
double apollo_solve_hyperbolic_kepler(double M, double e,
                                        double tolerance, int max_iterations);

/**
 * @brief Compute mean anomaly from true anomaly
 *
 * @param nu True anomaly (radians)
 * @param e Eccentricity
 * @return Mean anomaly (radians)
 */
double apollo_nu_to_M(double nu, double e);

/**
 * @brief Compute true anomaly from eccentric anomaly
 *
 * @param E Eccentric anomaly (radians)
 * @param e Eccentricity
 * @return True anomaly (radians)
 */
double apollo_E_to_nu(double E, double e);

/* =========================================================================
 * ORBITAL PARAMETERS
 * ========================================================================= */

/**
 * @brief Compute orbital period
 *
 * @param a Semi-major axis (meters)
 * @param mu Gravitational parameter
 * @return Period (seconds)
 */
double apollo_period(double a, double mu);

/**
 * @brief Compute specific angular momentum
 *
 * @param state State vector
 * @return Angular momentum vector
 */
apollo_v3_t apollo_h(const apollo_state_t *state);

/**
 * @brief Compute specific orbital energy
 *
 * @param state State vector
 * @param mu Gravitational parameter
 * @return Specific energy (J/kg = m^2/s^2)
 */
double apollo_energy(const apollo_state_t *state, double mu);

/**
 * @brief Compute semi-latus rectum
 *
 * @param state State vector
 * @param mu Gravitational parameter
 * @return Semi-latus rectum (meters)
 */
double apollo_p(const apollo_state_t *state, double mu);

/* =========================================================================
 * UTILITY
 * ========================================================================= */

/**
 * @brief Compute distance (magnitude of position vector)
 */
double apollo_r(const apollo_state_t *state);

/**
 * @brief Compute speed (magnitude of velocity vector)
 */
double apollo_v(const apollo_state_t *state);

/**
 * @brief Compute flight path angle
 *
 * @param state State vector
 * @return Flight path angle (radians)
 */
double apollo_fpa(const apollo_state_t *state);

/**
 * @brief Print state vector
 */
void apollo_print_state(const apollo_state_t *state, const char *name);

/**
 * @brief Print orbital elements
 */
void apollo_print_elements(const apollo_elements_t *elem, const char *name);

#endif /* APOLLO_ORBIT_H */
