/**
 * @file apollo_guidance.h
 * @brief Guidance algorithms
 *
 * From: P30-P37.agc, P40-P47.agc, P32-P33_P72-P73.agc
 */

#ifndef APOLLO_GUIDANCE_H
#define APOLLO_GUIDANCE_H

#include "apollo_types.h"
#include "apollo_vecmath.h"
#include "apollo_orbit.h"
#include "apollo_nav.h"

/* =========================================================================
 * BURN TYPES
 * ========================================================================= */

/**
 * @brief Burn type identifiers
 */
typedef enum {
    APOLLO_BURN_TLI = 1,     /**< Trans-lunar injection */
    APOLLO_BURN_LOI = 2,      /**< Lunar orbit insertion */
    APOLLO_BURN_PDI = 3,      /**< Powered descent initiation */
    APOLLO_BURN_TPI = 4,     /**< Transfer phase initiation */
    APOLLO_BURN_TPM = 5,     /**< Transfer phase mid-course */
    APOLLO_BURN_CSI = 6,     /**< Conjunction sequence ignition */
    APOLLO_BURN_NCC = 7,     /**< Nancy character check / NCC */
    APOLLO_BURN_SIVB = 8,    /**< S-IVB burn */
    APOLLO_BURN_CM_ENTRY = 9  /**< CM entry */
} apollo_burn_type_t;

/* =========================================================================
 * BURN TARGETING
 * ========================================================================= */

/**
 * @brief Burn targeting parameters
 */
typedef struct {
    apollo_burn_type_t type;      /**< Type of burn */
    apollo_orbstate_t target_sv;  /**< Target state vector */
    double desired_dv;           /**< Desired delta-V (m/s) */
    double actual_dv;             /**< Actual delta-V achieved */
    double burn_time;             /**< Time of burn (centisec) */
    double pgh;                  /**< Height of periapsis after burn */
    apollo_v3_t n_guid;          /**< Guidance unit vector */
} apollo_burn_t;

/* =========================================================================
 * LAMBERT TARGETING (P30)
 * ========================================================================= */

/**
 * @brief Lambert targeting parameters
 */
typedef struct {
    apollo_orbstate_t sv_initial;   /**< Initial state vector */
    apollo_orbstate_t sv_target;    /**< Target state vector */
    apollo_v3_t r1;                 /**< Position at t0 */
    apollo_v3_t r2;                 /**< Position at t1 */
    double t_trans;                  /**< Transfer time (sec) */
    double cog;                      /**< C of gravity (flight path angle) */
    int n_rev;                       /**< Number of revolutions */
    int direction;                   /**< 0 = prograde, 1 = retrograde */
} apollo_lambert_t;

/**
 * @brief Solve Lambert problem
 *
 * Given r1, r2, and transfer time, find required velocity at r1.
 */
void apollo_lambert_solve(const apollo_lambert_t *lambert, apollo_orbstate_t *out);

/**
 * @brief Compute Lambert targeting for rendezvous
 */
void apollo_lambert_rnd(const apollo_lambert_t *lambert, apollo_orbstate_t *sv);

/* =========================================================================
 * CSI (Conjunction Sequence Initiation)
 * ========================================================================= */

/**
 * @brief CSI targeting parameters
 */
typedef struct {
    apollo_orbstate_t sv0;         /**< Current state */
    apollo_orbstate_t sv1;         /**< Target 1 state */
    apollo_orbstate_t sv2;         /**< Target 2 state */
    double n_des;                   /**< Desired relative motion */
    double dv_csi;                 /**< CSI delta-V */
    double t_csi;                   /**< Time of CSI */
    double t_cdi;                   /**< Time of CDI */
} apollo_csi_t;

/**
 * @brief Solve CSI problem
 */
void apollo_csi_solve(apollo_csi_t *csi);

/* =========================================================================
 * powered flight (P40-P47)
 * ========================================================================= */

/**
 * @brief Powered flight parameters
 */
typedef struct {
    apollo_burn_type_t type;       /**< Type of powered flight */
    apollo_orbstate_t sv_init;     /**< Initial state vector */
    apollo_orbstate_t sv_half;     /**< Half-computation state */
    apollo_orbstate_t sv_next;     /**< Next-computation state */
    double dv_desired;              /**< Desired delta-V */
    double dv_achieved;            /**< Achieved delta-V */
    double ign_time;                /**< Ignition time */
    double cut_time;               /**< Cutoff time */
    apollo_v3_t v_ignition;        /**< Velocity at ignition */
    apollo_v3_t v_cutoff;         /**< Velocity at cutoff */
    apollo_v3_t dv_vector;         /**< Delta-V vector */
    double mass_initial;            /**< Initial mass (kg) */
    double mass_final;             /**< Final mass (kg) */
} apollo_powered_flight_t;

/**
 * @brief Compute powered flight trajectory
 */
void apollo_powered_flight(apollo_powered_flight_t *pf, double mu);

/**
 * @brief Compute half-computation state
 */
void apollo_half_computation(apollo_powered_flight_t *pf, double mu);

/**
 * @brief Compute SIVB retrograde burn
 */
void apollo_sivb_retrograde(apollo_powered_flight_t *pf);

/**
 * @brief Compute LOI burn
 */
void apollo_loi_burn(const apollo_orbstate_t *sv_before,
                      apollo_orbstate_t *sv_after, double mu);

/* =========================================================================
 * DESCENT GUIDANCE (P68-P75)
 * ========================================================================= */

/**
 * @brief Descent guidance parameters
 */
typedef struct {
    double h_des;           /**< Desired height */
    double h_actual;       /**< Actual height */
    double v_vertical;     /**< Vertical velocity */
    double v_horizontal;  /**< Horizontal velocity */
    double range;          /**< Downrange */
    double tgt_range;      /**< Target range */
    double slope_angle;    /**< Descent path angle */
    double thrust_cmd;      /**< Throttle command */
    double mass_current;    /**< Current vehicle mass */
} apollo_descent_guidance_t;

/**
 * @brief Update descent guidance
 */
void apollo_descent_update(apollo_descent_guidance_t *dg, double dt);

/**
 * @brief Compute PDG (Powered Descent Guidance)
 */
void apollo_pdg(const apollo_descent_guidance_t *dg, apollo_v3_t *thrust_cmd);

/* =========================================================================
 * ENTRY GUIDANCE (P67)
 * ========================================================================= */

/**
 * @brief Entry guidance parameters
 */
typedef struct {
    double range;           /**< Range to target */
    double gamma;          /**< Inertial flight path angle */
    double v_inertial;     /**< Inertial velocity */
    double v_loss;         /**< Velocity loss */
    double dr_pred;        /**< Predicted altitude */
    double v_terminal;     /**< Terminal phase entry velocity */
} apollo_entry_guidance_t;

/**
 * @brief Compute entry guidance constants
 */
void apollo_entry_init(const apollo_orbstate_t *sv_entry, double lat, double lng);

/**
 * @brief Update entry guidance
 */
void apollo_entry_update(apollo_entry_guidance_t *eg, double dt);

#endif /* APOLLO_GUIDANCE_H */
