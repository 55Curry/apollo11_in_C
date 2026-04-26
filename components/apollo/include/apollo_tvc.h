/**
 * @file apollo_tvc.h
 * @brief Thrust Vector Control
 *
 * From: TVCDAPS.agc, TVCINITIALIZE.agc, TVCEXECUTIVE.agc
 */

#ifndef APOLLO_TVC_H
#define APOLLO_TVC_H

#include "apollo_types.h"
#include "apollo_vecmath.h"

/* =========================================================================
 * TVC CONFIGURATION
 * ========================================================================= */

/**
 * @brief TVC actuator state
 */
typedef struct {
    double position;      /**< Current position (rad) */
    double rate;          /**< Rate (rad/s) */
    double cmd_position;  /**< Commanded position (rad) */
    double cmd_rate;     /**< Commanded rate (rad/s) */
} apollo_actuator_t;

/**
 * @brief TVC engine configuration
 */
typedef struct {
    double thrust_min;     /**< Minimum thrust (N) */
    double thrust_max;    /**< Maximum thrust (N) */
    double isp;           /**< Specific impulse (s) */
    double burn_time;     /**< Total burn time (s) */
    double mass_flow;     /**< Mass flow rate (kg/s) */
} apollo_engine_config_t;

/**
 * @brief TVC control system state
 */
typedef struct {
    apollo_actuator_t pitch;    /**< Pitch actuator */
    apollo_actuator_t yaw;      /**< Yaw actuator */
    apollo_actuator_t roll;     /**< Roll actuator (if present) */
    double pitch_cmd;           /**< Pitch command */
    double yaw_cmd;            /**< Yaw command */
    double roll_cmd;           /**< Roll command */
    bool enabled;               /**< TVC enabled flag */
} apollo_tvc_state_t;

/* =========================================================================
 * TVC INITIALIZATION
 * ========================================================================= */

/**
 * @brief Initialize TVC system
 *
 * @param config Engine configuration
 * @param state TVC state to initialize
 */
void apollo_tvc_init(const apollo_engine_config_t *config,
                      apollo_tvc_state_t *state);

/**
 * @brief Reset TVC to initial state
 */
void apollo_tvc_reset(apollo_tvc_state_t *state);

/* =========================================================================
 * TVC CONTROL
 * ========================================================================= */

/**
 * @brief Update TVC system
 *
 * @param state TVC state
 * @param dt Time step (s)
 */
void apollo_tvc_update(apollo_tvc_state_t *state, double dt);

/**
 * @brief Set TVC commanded angles
 *
 * @param state TVC state
 * @param pitch Pitch command (rad)
 * @param yaw Yaw command (rad)
 */
void apollo_tvc_set_cmd(apollo_tvc_state_t *state, double pitch, double yaw);

/**
 * @brief Enable TVC
 */
void apollo_tvc_enable(apollo_tvc_state_t *state);

/**
 * @brief Disable TVC
 */
void apollo_tvc_disable(apollo_tvc_state_t *state);

/* =========================================================================
 * TVC DAP (Digital Autopilot)
 * ========================================================================= */

/**
 * @brief TVC DAP parameters
 */
typedef struct {
    double kp;           /**< Proportional gain */
    double ki;           /**< Integral gain */
    double kd;           /**< Derivative gain */
    double limit;        /**< Output limit */
} apollo_tvc_dap_gains_t;

/**
 * @brief Compute TVC DAP output
 *
 * @param cmd Commanded angle
 * @param actual Actual angle
 * @param rate Actual rate
 * @param gains DAP gains
 * @return Control output
 */
double apollo_tvc_dap(double cmd, double actual, double rate,
                      const apollo_tvc_dap_gains_t *gains);

/* =========================================================================
 * TVC SERVICER
 * ========================================================================= */

/**
 * @brief TVC servicer state (P47)
 */
typedef struct {
    double dv_desired;       /**< Desired delta-V */
    double dv_achieved;     /**< Achieved delta-V */
    double v_vehicle;        /**< Vehicle velocity */
    double mass_initial;     /**< Initial mass */
    double mass_current;     /**< Current mass */
    double mass_flow_rate;   /**< Mass flow rate */
    double thrust_cmd;       /**< Thrust command */
    double isp;              /**< Specific impulse */
    double eng_on;           /**< Engine on flag */
} apollo_tvc_servicer_t;

/**
 * @brief Update TVC servicer
 */
void apollo_tvc_servicer_update(apollo_tvc_servicer_t *svc, double dt);

/* =========================================================================
 * TVC ERRORS
 * ========================================================================= */

/**
 * @brief TVC error codes
 */
typedef enum {
    APOLLO_TVC_OK = 0,
    APOLLO_TVC_ACTUATOR_FAULT = 1,
    APOLLO_TVC_LOOP_FAILURE = 2,
    APOLLO_TVC_ENGINE_FAULT = 3
} apollo_tvc_error_t;

/**
 * @brief Check TVC for errors
 */
apollo_tvc_error_t apollo_tvc_check(const apollo_tvc_state_t *state);

#endif /* APOLLO_TVC_H */
