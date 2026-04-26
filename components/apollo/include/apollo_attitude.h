/**
 * @file apollo_attitude.h
 * @brief Attitude determination and control
 *
 * From: ANGLFIND.agc, KALCMANU_STEERING.agc, GIMBAL_LOCK_AVOIDANCE.agc
 */

#ifndef APOLLO_ATTITUDE_H
#define APOLLO_ATTITUDE_H

#include "apollo_types.h"
#include "apollo_vecmath.h"

/* =========================================================================
 * ATTITUDE ANGLES
 * ========================================================================= */

/**
 * @brief CDU (Coupling Data Unit) angles
 *
 * The CDU angles represent the gimbal angles of the IMU.
 */
typedef struct {
    double x;   /**< Outer gimbal angle (CDUX) */
    double y;   /**< Inner gimbal angle (CDUY) */
    double z;   /**< Middle gimbal angle (CDUZ) */
} apollo_cdu_angles_t;

/**
 * @brief Command angles for desired attitude
 */
typedef struct {
    double x;   /**< Commanded CDUX */
    double y;   /**< Commanded CDUY */
    double z;   /**< Commanded CDUZ */
} apollo_commanded_angles_t;

/* =========================================================================
 * DIRECTION COSINE MATRICES
 * ========================================================================= */

/**
 * @name DCM (Direction Cosine Matrix) Types
 *
 * Transformation matrices between coordinate systems:
 *   MIS  - Stable member to Initialized
 *   BCI  - Body to Current (IMU)
 *   S/C  - Spacecraft body axes
 *   NB   - Navigation base
 * @{
 */

/** @brief DCM type */
typedef apollo_mat3_t apollo_dcm_t;

/** @brief Transformation: Initial S/M to Current S/M */
apollo_dcm_t apollo_compute_mis(const apollo_cdu_angles_t *cdu,
                                  const apollo_dcm_t *mis_initial);

/** @brief Transformation: Current S/M to Stable Member */
apollo_dcm_t apollo_compute_tmis(const apollo_dcm_t *mis);

/** @brief Transformation: Body to IMU */
apollo_dcm_t apollo_compute_bci(const apollo_dcm_t *nb2);

 /** @brief Transformation: IMU to Body */
apollo_dcm_t apollo_compute_cbi(const apollo_dcm_t *bc);

/** @} */

/* =========================================================================
 * ATTITUDE CONTROL
 * ========================================================================= */

/**
 * @brief Maneuver rate identifiers
 */
typedef enum {
    APOLLO_RATE_05_DEG,  /**< 0.05 deg/sec */
    APOLLO_RATE_2_DEG,   /**< 2 deg/sec */
    APOLLO_RATE_5_DEG,   /**< 5 deg/sec */
    APOLLO_RATE_10_DEG   /**< 10 deg/sec */
} apollo_maneuver_rate_t;

/**
 * @brief Maneuver rate values in radians per second
 */
#define APOLLO_RATE_05_DEG_VAL (0.05 * APOLLO_D2R / 60.0)
#define APOLLO_RATE_2_DEG_VAL  (2.0 * APOLLO_D2R / 60.0)
#define APOLLO_RATE_5_DEG_VAL  (5.0 * APOLLO_D2R / 60.0)
#define APOLLO_RATE_10_DEG_VAL (10.0 * APOLLO_D2R / 60.0)

/**
 * @brief Perform automatic maneuver to desired angles
 *
 * @param target Desired CDU angles
 * @param rate Maneuver rate
 * @param body_rate Output: body rates [rad/s]
 */
void apollo_maneuver_to_angles(const apollo_commanded_angles_t *target,
                                 apollo_maneuver_rate_t rate,
                                 apollo_v3_t *body_rate);

/**
 * @brief Check for gimbal lock condition
 *
 * @param cdu Current CDU angles
 * @return true if in gimbal lock
 */
bool apollo_check_gimbal_lock(const apollo_cdu_angles_t *cdu);

/**
 * @brief Avoid gimbal lock by selecting optimal maneuver axis
 *
 * @param desired_direction Target direction vector
 * @param current_att Current attitude matrix
 * @param optimal_axis Output: optimal rotation axis
 */
void apollo_avoid_gimbal_lock(const apollo_v3_t *desired_direction,
                               const apollo_dcm_t *current_att,
                               apollo_v3_t *optimal_axis);

/* =========================================================================
 * QUATERNION OPERATIONS
 * ========================================================================= */

/**
 * @brief Quaternion for attitude representation
 */
typedef struct {
    double q0;  /**< Scalar part */
    double qx;  /**< X component */
    double qy;  /**< Y component */
    double qz;  /**< Z component */
} apollo_quat_t;

/**
 * @brief Convert quaternion to DCM
 */
apollo_dcm_t apollo_quat_to_dcm(const apollo_quat_t *q);

/**
 * @brief Convert DCM to quaternion
 */
apollo_quat_t apollo_dcm_to_quat(const apollo_dcm_t *m);

/**
 * @brief Quaternion multiplication
 */
apollo_quat_t apollo_quat_mul(const apollo_quat_t *a, const apollo_quat_t *b);

/**
 * @brief Quaternion to Euler angles
 */
void apollo_quat_to_euler(const apollo_quat_t *q, double *yaw,
                          double *pitch, double *roll);

/* =========================================================================
 * ATTITUDE HOLD MODE
 * ========================================================================= */

/**
 * @brief Enter attitude hold mode
 */
void apollo_attitude_hold(const apollo_cdu_angles_t *desired_angles);

/**
 * @brief Exit attitude hold mode
 */
void apollo_attitude_hold_exit(void);

/* =========================================================================
 * RATES
 * ========================================================================= */

/**
 * @brief Angular rates (rad/s)
 */
typedef struct {
    double p;   /**< Roll rate */
    double q;   /**< Pitch rate */
    double r;   /**< Yaw rate */
} apollo_body_rates_t;

/**
 * @brief Compute body rates from CDI angles
 */
apollo_body_rates_t apollo_compute_body_rates(const apollo_cdu_angles_t *cdu_angles,
                                               const apollo_cdu_angles_t *cdu_rates);

#endif /* APOLLO_ATTITUDE_H */
