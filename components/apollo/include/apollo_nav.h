/**
 * @file apollo_nav.h
 * @brief Navigation functions
 *
 * From: P20-P25.agc, GROUND_TRACKING_DETERMINATION_PROGRAM.agc
 */

#ifndef APOLLO_NAV_H
#define APOLLO_NAV_H

#include "apollo_types.h"
#include "apollo_vecmath.h"
#include "apollo_orbit.h"
#include "apollo_attitude.h"

/* =========================================================================
 * ORBITAL STATE VECTOR
 * ========================================================================= */

/**
 * @brief Vehicle state vector in space
 */
typedef struct {
    apollo_v3_t r;       /**< Position (meters) */
    apollo_v3_t v;       /**< Velocity (meters/second) */
    double t;            /**< Time (centiseconds) */
} apollo_orbstate_t;

/**
 * @brief State vector stored for downlink
 */
typedef struct {
    apollo_orbstate_t state;    /**< Orbital state */
    int body;                    /**< 0 = CM, 1 = LM */
} apollo_state_vector_t;

/* =========================================================================
 * LANDMARKS
 * ========================================================================= */

/**
 * @brief Landmark for optical navigation
 */
typedef struct {
    apollo_v3_t r;       /**< Position vector of landmark */
    double lat;           /**< Latitude (radians) */
    double lng;           /**< Longitude (radians) */
    double h_ellipsoid;  /**< Height above ellipsoid (meters) */
} apollo_landmark_t;

/**
 * @brief P21 landmark tracking data
 */
typedef struct {
    apollo_landmark_t landmark;
    double az;            /**< Measured azimuth (radians) */
    double el;            /**< Measured elevation (radians) */
    double observed_az;  /**< Observed azimuth */
    double observed_el;  /**< Observed elevation */
    int quality;          /**< Data quality flag */
} apollo_landmark_obs_t;

/* =========================================================================
 * NAVIGATION MEASUREMENTS
 * ========================================================================= */

/**
 * @brief Star sighting measurement
 */
typedef struct {
    apollo_v3_t star_vector;    /**< Unit vector to star */
    apollo_cdu_angles_t cdu_angles;  /**< CDU angles at sighting */
    double time_tag;             /**< Time of sighting (centisec) */
} apollo_star_sighting_t;

/**
 * @brief Horizon sighting for pitch/roll
 */
typedef struct {
    double pitch_angle;   /**< Measured pitch */
    double roll_angle;    /**< Measured roll */
    double time_tag;     /**< Time of sighting */
} apollo_horizon_sighting_t;

/* =========================================================================
 * OPTICAL TRACKER (OT)
 * ========================================================================= */

/**
 * @brief Sextant measurements
 */
typedef struct {
    double shaft;    /**< Shaft angle (radians) */
    double trunnion; /**< Trunnion angle (radians) */
    double time;     /**< Time tag */
} apollo_sextant_t;

/**
 * @brief Landmark tracking results
 */
typedef struct {
    apollo_v3_t position;    /**< Computed position */
    apollo_v3_t velocity;    /**< Computed velocity */
    double time;             /**< Time of solution */
    int quality;             /**< Solution quality 0-9 */
} apollo_lmk_track_result_t;

/**
 * @brief Compute LOS (Line of Sight) to landmark
 */
apollo_v3_t apollo_los_to_landmark(const apollo_landmark_t *lmk,
                                     const apollo_orbstate_t *sv);

/**
 * @brief Compute landmark observation angles
 */
void apollo_compute_lmk_angles(const apollo_landmark_t *lmk,
                                const apollo_orbstate_t *sv,
                                double *az, double *el);

/**
 * @brief Update state vector with landmark observation
 */
void apollo_lmk_update(apollo_orbstate_t *sv,
                        const apollo_landmark_obs_t *obs);

/* =========================================================================
 * STATE VECTOR PROPAGATION
 * ========================================================================= */

/**
 * @brief Initialize state vector from ground tracking
 */
void apollo_init_sv_from_ground(apollo_orbstate_t *sv,
                                 double lat, double lng, double alt,
                                 double az, double el, double range);

/**
 * @brief Propagate state vector
 */
void apollo_propagate_sv(apollo_orbstate_t *sv, double dt, double mu);

/**
 * @brief Integrate state vector
 */
void apollo_integrate_sv(apollo_orbstate_t *sv, double dt, double mu);

/* =========================================================================
 * RENDEZVOUS NAVIGATION (P20-P25)
 * ========================================================================= */

/**
 * @brief Relative motion state
 */
typedef struct {
    apollo_orbstate_t sv_tgt;     /**< Target state vector */
    apollo_orbstate_t sv_rv;      /**< Rendezvous vehicle state */
    apollo_v3_t r_rels;           /**< Relative position */
    apollo_v3_t v_rels;           /**< Relative velocity */
    double t_rel;                 /**< Relative time */
} apollo_rndnav_t;

/**
 * @brief Compute rendezvous navigation
 */
void apollo_rendezvous_nav(apollo_rndnav_t *rnd);

/**
 * @brief Compute TPI (Transfer Phase Initiation) time
 */
double apollo_compute_tpi(const apollo_rndnav_t *rnd, double h_ref);

/* =========================================================================
 * GROUND TRACKING
 * ========================================================================= */

/**
 * @brief Compute latitude and longitude from state vector
 */
void apollo_sv_to_latlong(const apollo_orbstate_t *sv,
                           double *lat, double *lng, double *alt);

/**
 * @brief Compute ground track
 */
void apollo_ground_track(const apollo_orbstate_t *sv,
                         double *lat, double *lng,
                         double *azimuth, double *elevation);

#endif /* APOLLO_NAV_H */
