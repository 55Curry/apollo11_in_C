/**
 * @file apollo_nav.c
 * @brief Navigation functions implementation
 *
 * From: P20-P25.agc, GROUND_TRACKING_DETERMINATION_PROGRAM.agc
 */

#include "apollo_nav.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================================================================
 * STATE VECTOR OPERATIONS
 * ========================================================================= */

/**
 * @brief Compute LOS (Line of Sight) to landmark
 */
apollo_v3_t apollo_los_to_landmark(const apollo_landmark_t *lmk,
                                     const apollo_orbstate_t *sv)
{
    apollo_v3_t los;
    los.x = lmk->r.x - sv->r.x;
    los.y = lmk->r.y - sv->r.y;
    los.z = lmk->r.z - sv->r.z;
    return apollo_v3_unit(&los);
}

/**
 * @brief Compute landmark observation angles
 */
void apollo_compute_lmk_angles(const apollo_landmark_t *lmk,
                                const apollo_orbstate_t *sv,
                                double *az, double *el)
{
    apollo_v3_t los = apollo_los_to_landmark(lmk, sv);
    apollo_v3_t vel_unit = apollo_v3_unit(&sv->v);

    /* Azimuth: angle in horizontal plane */
    *az = atan2(los.y, los.x);

    /* Elevation: angle above horizontal */
    double sin_el = los.z;
    *el = asin(sin_el);
}

/**
 * @brief Update state vector with landmark observation
 */
void apollo_lmk_update(apollo_orbstate_t *sv,
                        const apollo_landmark_obs_t *obs)
{
    /*
     * Simplified Kalman filter update.
     * Original AGC used a more complex weighted least squares.
     */
    double measured_az = obs->observed_az;
    double measured_el = obs->observed_el;
    double computed_az, computed_el;

    apollo_compute_lmk_angles(&obs->landmark, sv, &computed_az, &computed_el);

    /* Azimuth error */
    double az_err = measured_az - computed_az;

    /* Elevation error */
    double el_err = measured_el - computed_el;

    /* Apply correction proportional to error (simplified) */
    double k_az = 0.1;
    double k_el = 0.1;

    /* Update velocity based on angle errors */
    sv->v.x += k_az * az_err * 1000.0;
    sv->v.y += k_az * az_err * 1000.0;
    sv->v.z += k_el * el_err * 1000.0;
}

/* =========================================================================
 * STATE VECTOR PROPAGATION
 * ========================================================================= */

/**
 * @brief Initialize state vector from ground tracking
 */
void apollo_init_sv_from_ground(apollo_orbstate_t *sv,
                                 double lat, double lng, double alt,
                                 double az, double el, double range)
{
    double Re = 6378136.0;  /* Earth equatorial radius */
    double Rp = Re + alt;

    /* Position in ECEF frame */
    sv->r.x = Rp * cos(lat) * cos(lng);
    sv->r.y = Rp * cos(lat) * sin(lng);
    sv->r.z = Rp * sin(lat);

    /* Velocity from tracking (simplified) */
    sv->v.x = 0;
    sv->v.y = 0;
    sv->v.z = 0;

    sv->t = 0;
}

/**
 * @brief Propagate state vector
 */
void apollo_propagate_sv(apollo_orbstate_t *sv, double dt, double mu)
{
    /*
     * Use two-body propagation (Kepler)
     * For more accurate results, use numerical integration
     */
    apollo_state_t state = {
        .r = sv->r,
        .v = sv->v
    };

    apollo_state_t propagated = apollo_kepler(&state, dt, mu);

    sv->r = propagated.r;
    sv->v = propagated.v;
    sv->t += dt * 100.0;  /* Convert to centiseconds */
}

/**
 * @brief Integrate state vector
 */
void apollo_integrate_sv(apollo_orbstate_t *sv, double dt, double mu)
{
    /*
     * Simplified numerical integration (Euler-Cauchy)
     * For better accuracy, use Runge-Kutta or Verlet
     */
    apollo_v3_t r = apollo_v3_unit(&sv->r);
    double r_mag = apollo_v3_mag(&sv->r);
    double a = mu / (r_mag * r_mag);  /* Gravitational acceleration */

    /* Acceleration = -mu/r^3 * r */
    apollo_v3_t acc = apollo_v3_scale(&r, -a);

    /* Update velocity */
    sv->v.x += acc.x * dt;
    sv->v.y += acc.y * dt;
    sv->v.z += acc.z * dt;

    /* Update position */
    sv->r.x += sv->v.x * dt;
    sv->r.y += sv->v.y * dt;
    sv->r.z += sv->v.z * dt;

    sv->t += dt * 100.0;
}

/* =========================================================================
 * RENDEZVOUS NAVIGATION
 * ========================================================================= */

/**
 * @brief Compute rendezvous navigation
 */
void apollo_rendezvous_nav(apollo_rndnav_t *rnd)
{
    /* Relative position */
    rnd->r_rels.x = rnd->sv_tgt.r.x - rnd->sv_rv.r.x;
    rnd->r_rels.y = rnd->sv_tgt.r.y - rnd->sv_rv.r.y;
    rnd->r_rels.z = rnd->sv_tgt.r.z - rnd->sv_rv.r.z;

    /* Relative velocity */
    rnd->v_rels.x = rnd->sv_tgt.v.x - rnd->sv_rv.v.x;
    rnd->v_rels.y = rnd->sv_tgt.v.y - rnd->sv_rv.v.y;
    rnd->v_rels.z = rnd->sv_tgt.v.z - rnd->sv_rv.v.z;

    /* Relative time (assumes same epoch) */
    rnd->t_rel = rnd->sv_tgt.t - rnd->sv_rv.t;
}

/**
 * @brief Compute TPI (Transfer Phase Initiation) time
 */
double apollo_compute_tpi(const apollo_rndnav_t *rnd, double h_ref)
{
    /*
     * Simplified TPI computation.
     * Original AGC used iterative solution.
     */
    double r_mag = apollo_v3_mag(&rnd->r_rels);
    double v_mag = apollo_v3_mag(&rnd->v_rels);

    /* Approximate time to height */
    double dt = sqrt(h_ref / 9.81);  /* Simplified ballistic */

    return dt;
}

/* =========================================================================
 * GROUND TRACKING
 * ========================================================================= */

/**
 * @brief Compute latitude and longitude from state vector
 */
void apollo_sv_to_latlong(const apollo_orbstate_t *sv,
                           double *lat, double *lng, double *alt)
{
    double x = sv->r.x;
    double y = sv->r.y;
    double z = sv->r.z;
    double r = sqrt(x * x + y * y + z * z);

    /* Earth ellipsoid approximation */
    double Re = 6378136.0;
    double f = 1.0 / 298.257;

    *lat = atan2(z, sqrt(x * x + y * y));
    *lng = atan2(y, x);
    *alt = r - Re;

    /* More accurate: account for ellipsoid */
    double sin_lat = sin(*lat);
    double cos_lat = cos(*lat);
    double N = Re / sqrt(1.0 - f * (2.0 - f) * sin_lat * sin_lat);
    *alt = r / cos_lat - N;
}

/**
 * @brief Compute ground track
 */
void apollo_ground_track(const apollo_orbstate_t *sv,
                         double *lat, double *lng,
                         double *azimuth, double *elevation)
{
    /* Get lat/long */
    apollo_sv_to_latlong(sv, lat, lng, azimuth);

    /* Elevation to local horizontal */
    double r = apollo_v3_mag(&sv->r);
    double Re = 6371000.0;
    *elevation = asin((Re * Re) / (r * r) - 1.0);

    /* Placeholder for azimuth calculation */
    *azimuth = 0.0;
}

#ifdef APOLLO_NAV_TEST

#include <stdio.h>
#include <assert.h>

#define TEST_PRECISION 1e-3

static int test_sv_to_latlong(void)
{
    printf("Testing state vector to lat/long...\n");

    apollo_orbstate_t sv;
    double lat, lng, alt;

    /* ISS-like orbit */
    sv.r.x = 6.7e6;
    sv.r.y = 0;
    sv.r.z = 0;
    sv.v.x = 0;
    sv.v.y = 7.66e3;
    sv.v.z = 0;
    sv.t = 0;

    apollo_sv_to_latlong(&sv, &lat, &lng, &alt);

    printf("  Position: (%.1f, %.1f, %.1f) km\n",
           sv.r.x/1000, sv.r.y/1000, sv.r.z/1000);
    printf("  Lat: %.2f deg, Lng: %.2f deg, Alt: %.1f km\n",
           lat * 180 / M_PI, lng * 180 / M_PI, alt / 1000);

    return 0;
}

static int test_los_to_landmark(void)
{
    printf("Testing LOS to landmark...\n");

    apollo_landmark_t lmk = {
        .r = {1000, 1000, 1000},
        .lat = 0,
        .lng = 0,
        .h_ellipsoid = 0
    };

    apollo_orbstate_t sv = {
        .r = {0, 0, 0},
        .v = {0, 0, 0},
        .t = 0
    };

    apollo_v3_t los = apollo_los_to_landmark(&lmk, &sv);
    double az, el;
    apollo_compute_lmk_angles(&lmk, &sv, &az, &el);

    printf("  LOS: (%.3f, %.3f, %.3f)\n", los.x, los.y, los.z);
    printf("  Az: %.2f deg, El: %.2f deg\n",
           az * 180 / M_PI, el * 180 / M_PI);

    return 0;
}

int main(void)
{
    int failures = 0;

    printf("=== Apollo Navigation Module Tests ===\n\n");

    failures += test_sv_to_latlong();
    failures += test_los_to_landmark();

    printf("\n=== Summary: %d failures ===\n", failures);

    return failures;
}

#endif /* APOLLO_NAV_TEST */
