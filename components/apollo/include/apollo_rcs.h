/**
 * @file apollo_rcs.h
 * @brief Reaction Control System
 *
 * From: RCS-CSM_DIGITAL_AUTOPILOT.agc, JET_SELECTION_LOGIC.agc
 */

#ifndef APOLLO_RCS_H
#define APOLLO_RCS_H

#include "apollo_types.h"
#include "apollo_vecmath.h"
#include "apollo_attitude.h"

/* =========================================================================
 * RCS JET CONFIGURATION
 * ========================================================================= */

/**
 * @brief RCS jet identifiers
 */
typedef enum {
    /* Primary RCS jets - 16 total */
    APOLLO_RCS_PITCH_UP_1 = 0,
    APOLLO_RCS_PITCH_UP_2 = 1,
    APOLLO_RCS_PITCH_DN_1 = 2,
    APOLLO_RCS_PITCH_DN_2 = 3,
    APOLLO_RCS_YAW_RIGHT_1 = 4,
    APOLLO_RCS_YAW_RIGHT_2 = 5,
    APOLLO_RCS_YAW_LEFT_1 = 6,
    APOLLO_RCS_YAW_LEFT_2 = 7,
    APOLLO_RCS_ROLL_RIGHT_1 = 8,
    APOLLO_RCS_ROLL_RIGHT_2 = 9,
    APOLLO_RCS_ROLL_LEFT_1 = 10,
    APOLLO_RCS_ROLL_LEFT_2 = 11,
    APOLLO_RCS_TRANS_X_PLUS = 12,
    APOLLO_RCS_TRANS_X_MINUS = 13,
    APOLLO_RCS_TRANS_Y_PLUS = 14,
    APOLLO_RCS_TRANS_Y_MINUS = 15,
    APOLLO_RCS_TRANS_Z_PLUS = 16,
    APOLLO_RCS_TRANS_Z_MINUS = 17
} apollo_rcs_jet_id_t;

/**
 * @brief Jet on/off state
 */
typedef struct {
    bool on;           /**< Jet fired state */
    double duration;   /**< On duration (s) */
    double torque;     /**< Torque produced (Nm) */
} apollo_rcs_jet_state_t;

/**
 * @brief RCS jet location and properties
 */
typedef struct {
    apollo_v3_t position;   /**< Jet position in vehicle frame */
    apollo_v3_t axis;      /**< Thrust axis direction */
    double thrust;          /**< Thrust magnitude (N) */
    apollo_v3_t torque_arm; /**< Torque arm from CG */
} apollo_rcs_jet_config_t;

/* =========================================================================
 * RCS CONTROL
 * ========================================================================= */

/**
 * @brief RCS control mode
 */
typedef enum {
    APOLLO_RCS_MODE_OFF = 0,       /**< RCS off */
    APOLLO_RCS_MODE_PITCH = 1,     /**< Pitch control */
    APOLLO_RCS_MODE_YAW = 2,       /**< Yaw control */
    APOLLO_RCS_MODE_ROLL = 3,      /**< Roll control */
    APOLLO_RCS_MODE_3AXIS = 4,     /**< 3-axis control */
    APOLLO_RCS_MODE_TRANS = 5      /**< Translation control */
} apollo_rcs_mode_t;

/**
 * @brief RCS system state
 */
typedef struct {
    apollo_rcs_mode_t mode;             /**< Current mode */
    apollo_rcs_jet_state_t jets[18];    /**< Jet states */
    apollo_body_rates_t rate_cmd;       /**< Commanded rates */
    apollo_body_rates_t rate_actual;    /**< Actual rates */
    apollo_v3_t torque_cmd;              /**< Commanded torque */
    double impulse_used;                 /**< Total impulse used */
    double fuel_remaining;               /**< Fuel remaining (kg) */
    bool jets_configured;               /**< Jets configured flag */
} apollo_rcs_state_t;

/* =========================================================================
 * RCS DAP
 * ========================================================================= */

/**
 * @brief RCS DAP gains
 */
typedef struct {
    double kp_rate;       /**< Rate proportional gain */
    double ki_rate;       /**< Rate integral gain */
    double kd_rate;       /**< Rate derivative gain */
    double kp_att;        /**< Attitude proportional gain */
    double deadband;       /**< Deadband (rad/s) */
    double rate_limit;    /**< Rate limit (rad/s) */
} apollo_rcs_dap_gains_t;

/**
 * @brief RCS jet selection
 */
typedef struct {
    apollo_rcs_jet_id_t jets[4];  /**< Selected jets */
    int num_jets;                  /**< Number of jets */
    double total_torque;           /**< Total torque */
} apollo_rcs_jet_selection_t;

/**
 * @brief Initialize RCS
 */
void apollo_rcs_init(apollo_rcs_state_t *state);

/**
 * @brief Configure RCS jets
 */
void apollo_rcs_configure(const apollo_rcs_dap_gains_t *gains,
                           apollo_rcs_state_t *state);

/**
 * @brief Update RCS
 */
void apollo_rcs_update(apollo_rcs_state_t *state, double dt);

/**
 * @brief Compute torque command
 */
apollo_v3_t apollo_rcs_torque_cmd(const apollo_body_rates_t *cmd,
                                   const apollo_body_rates_t *actual,
                                   const apollo_rcs_dap_gains_t *gains);

/* =========================================================================
 * JET SELECTION
 * ========================================================================= */

/**
 * @brief Select jets for desired torque
 */
void apollo_select_jets(const apollo_v3_t *desired_torque,
                        const apollo_rcs_state_t *state,
                        apollo_rcs_jet_selection_t *selection);

/**
 * @brief Minimize jet usage for torque
 */
void apollo_minimize_jets(const apollo_v3_t *desired_torque,
                          const apollo_rcs_state_t *state,
                          apollo_rcs_jet_selection_t *selection);

/**
 * @brief Check forjet conflict
 */
bool apollo_check_jet_conflict(const apollo_rcs_jet_selection_t *selection);

/* =========================================================================
 * IMPULSE ALLOCATION
 * ========================================================================= */

/**
 * @brief Compute impulse needed
 */
double apollo_compute_impulse(double desired_rate, double current_rate,
                                double inertia, double max_torque);

/**
 * @brief Allocate impulse among jets
 */
void apollo_allocate_impulse(double total_impulse,
                              apollo_rcs_jet_selection_t *selection,
                              apollo_rcs_state_t *state);

/* =========================================================================
 * FUEL MANAGEMENT
 * ========================================================================= */

/**
 * @brief Update fuel usage
 */
void apollo_update_fuel(apollo_rcs_state_t *state, double dt);

/**
 * @brief Check fuel depletion
 */
bool apollo_rcs_fuel_low(const apollo_rcs_state_t *state, double threshold);

#endif /* APOLLO_RCS_H */
