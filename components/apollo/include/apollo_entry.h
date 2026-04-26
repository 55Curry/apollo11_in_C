/**
 * @file apollo_entry.h
 * @brief Reentry control
 *
 * From: REENTRY_CONTROL.agc, CM_ENTRY_DIGITAL_AUTOPILOT.agc
 */

#ifndef APOLLO_ENTRY_H
#define APOLLO_ENTRY_H

#include "apollo_types.h"
#include "apollo_vecmath.h"

/* =========================================================================
 * ENTRY PHASES
 * ========================================================================= */

/**
 * @brief Entry phase identifiers
 */
typedef enum {
    APOLLO_ENTRY_PREENTRY = 0,    /**< Pre-entry */
    APOLLO_ENTRY_INTERFACE = 1,   /**< Interface */
    APOLLO_ENTRY_LINEUP = 2,      /**< Lineup */
    APOLLO_ENTRY_SUPERSONIC = 3,  /**< Supersonic */
    APOLLO_ENTRY_BALLISTIC = 4,   /**< Ballistic */
    APOLLO_ENTRY_STEEP = 5,       /**< Steep dive */
    APOLLO_ENTRY_SHALLOW = 6,     /**< Shallow */
    APOLLO_ENTRY_ROLL = 7,        /**< Roll program */
    APOLLO_ENTRY_PARACHUTE = 8    /**< Parachute deploy */
} apollo_entry_phase_t;

/* =========================================================================
 * ENTRY STATE
 * ========================================================================= */

/**
 * @brief Entry vehicle state
 */
typedef struct {
    apollo_v3_t r;           /**< Position (meters) */
    apollo_v3_t v;           /**< Velocity (meters/sec) */
    double mass;             /**< Mass (kg) */
    double q;                /**< Dynamic pressure (Pa) */
    double v_e;              /**< Entry velocity (m/s) */
    double gamma_e;          /**< Entry flight path angle */
    double lat;              /**< Latitude (rad) */
    double lng;              /**< Longitude (rad) */
    double h;                /**< Altitude (meters) */
    double dr;               /**< Drag (m/s^2) */
    double lift;             /**< Lift (m/s^2) */
} apollo_entry_state_t;

/**
 * @brief Entry target parameters
 */
typedef struct {
    double lat_tgt;          /**< Target latitude */
    double lng_tgt;          /**< Target longitude */
    double range_tgt;        /**< Target range */
    double aimrange;         /**< Aimpoint range */
} apollo_entry_target_t;

/* =========================================================================
 * ENTRY CONSTANTS
 * ========================================================================= */

/**
 * @name Entry corridor limits
 * @{
 */

#define APOLLO_ENTRY_Q_MAX      10000.0    /**< Max q (Pa) */
#define APOLLO_ENTRY_H_MIN     24384.0     /**< Min altitude for drag (m) */
#define APOLLO_ENTRY_H_STRAT   70100.0     /**< Stratopause altitude (m) */
#define APOLLO_ENTRY_V_MIN     1524.0      /**< Minimum velocity (m/s) */
#define APOLLO_ENTRY_V_MAX     11000.0     /**< Maximum velocity (m/s) */
#define APOLLO_ENTRY_G_MAX     8.0         /**< Maximum g */
#define APOLLO_ENTRY_DRAG_MAX  3.0         /**< Maximum drag (g) */
#define APOLLO_ENTRY_LIFT_MAX  3.5         /**< Maximum lift (g) */

/** @} */

/* =========================================================================
 * CM ENTRY (P67)
 * ========================================================================= */

/**
 * @brief CM entry parameters
 */
typedef struct {
    apollo_entry_phase_t phase;      /**< Current phase */
    apollo_entry_state_t state;      /**< Current state */
    apollo_entry_target_t target;    /**< Target */
    double v_approx;                  /**< Velocity approximation */
    double dr_approx;                /**< Altitude approximation */
    double dv_approx;                /**< Velocity difference */
    double cos_lat;                  /**< Cosine of latitude */
    double dv_pred;                  /**< Predicted velocity */
    double range_approx;             /**< Range approximation */
    double target_aim;               /**< Target aimpoint */
} apollo_cm_entry_t;

/**
 * @brief Initialize CM entry
 */
void apollo_cm_entry_init(apollo_cm_entry_t *entry,
                           const apollo_v3_t *r_entry,
                           const apollo_v3_t *v_entry,
                           double lat, double lng);

/**
 * @brief Update CM entry guidance
 */
void apollo_cm_entry_update(apollo_cm_entry_t *entry, double dt);

/**
 * @brief Compute entry commands
 */
void apollo_entry_commands(const apollo_cm_entry_t *entry,
                           double *roll_cmd,
                           double *pitch_cmd,
                           double *yaw_cmd);

/**
 * @brief Check for phase transition
 */
apollo_entry_phase_t apollo_check_phase_transition(const apollo_cm_entry_t *entry);

/* =========================================================================
 * BALLISTIC ENTRY
 * ========================================================================= */

/**
 * @brief Compute ballistic entry
 */
void apollo_ballistic_entry(apollo_entry_state_t *state, double dt);

/* =========================================================================
 * LIFTING ENTRY
 * ========================================================================= */

/**
 * @brief Compute lifting entry
 */
void apollo_lifting_entry(apollo_cm_entry_t *entry, double dt);

/**
 * @brief Compute bank angle command
 */
double apollo_bank_command(const apollo_cm_entry_t *entry);

/* =========================================================================
 * DRAG/HEATING
 * ========================================================================= */

/**
 * @brief Compute stagnation point heat rate
 *
 * @param v Velocity (m/s)
 * @param rho Atmospheric density (kg/m^3)
 * @return Heat rate (W/cm^2)
 */
double apollo_heat_rate(double v, double rho);

/**
 * @brief Compute total heat load
 *
 * @param v_entry Entry velocity (m/s)
 * @param h_entry Entry altitude (m)
 * @param mass Vehicle mass (kg)
 * @param A_ref Reference area (m^2)
 * @return Total heat load (J/cm^2)
 */
double apollo_heat_load(double v_entry, double h_entry,
                         double mass, double A_ref);

/* =========================================================================
 * Drogue/MAIN CHUTES
 * ========================================================================= */

/**
 * @brief Drogue deploy parameters
 */
typedef struct {
    double v_deploy;          /**< Deployment velocity (m/s) */
    double h_deploy;         /**< Deployment altitude (m) */
    double drag;             /**< Drag coefficient */
    double area;            /**< Area (m^2) */
} apollo_drogue_t;

/**
 * @brief Main chute deploy parameters
 */
typedef struct {
    double v_deploy;          /**< Deployment velocity (m/s) */
    double h_deploy;         /**< Deployment altitude (m) */
    double drag;             /**< Drag coefficient */
    double area;            /**< Area (m^2) */
} apollo_main_t;

/**
 * @brief Check for drogue deployment
 */
bool apollo_check_drogue(const apollo_entry_state_t *state);

/**
 * @brief Check for main deployment
 */
bool apollo_check_main(const apollo_entry_state_t *state);

/* =========================================================================
 * LANDING
 * ========================================================================= */

/**
 * @brief Landing parameters
 */
typedef struct {
    double v_vertical;       /**< Vertical velocity (m/s) */
    double v_horizontal;    /**< Horizontal velocity (m/s) */
    double h_surf;           /**< Surface altitude (m) */
    double rate_vertical;   /**< Vertical rate at impact */
} apollo_landing_t;

/**
 * @brief Compute landing parameters
 */
void apollo_landing_update(apollo_landing_t *land, const apollo_entry_state_t *state);

#endif /* APOLLO_ENTRY_H */
