/**
 * @file apollo_attitude.c
 * @brief Attitude determination and control implementation
 *
 * From: ANGLFIND.agc, KALCMANU_STEERING.agc, GIMBAL_LOCK_AVOIDANCE.agc
 */

#include "apollo_attitude.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TWO_PI (2.0 * M_PI)
#define HALF_PI (M_PI / 2.0)

/* =========================================================================
 * CDU (Coupling Data Unit) to DCM Conversion
 * ========================================================================= */

/**
 * @brief Convert CDU angles to DCM (CDUTODCM)
 *
 * The IMU gimbal system uses three gimbals: outer (X), inner (Y), middle (Z)
 * The DCM transforms from stable member axes to spacecraft body axes.
 *
 * Original AGC: Used trigonometric lookups and matrix multiplication
 */
apollo_dcm_t apollo_cdutodcm(const apollo_cdu_angles_t *cdu)
{
    apollo_dcm_t m;
    double cx, sx, cy, sy, cz, sz;

    /* CDU angles to direction cosines */
    cx = cos(cdu->x);
    sx = sin(cdu->x);
    cy = cos(cdu->y);
    sy = sin(cdu->y);
    cz = cos(cdu->z);
    sz = sin(cdu->z);

    /*
     * DCM for X-Y-Z gimbal rotation sequence
     * (outer-inner-middle = Z-Y-X in reverse)
     *
     * R = Rz(cz) * Ry(cy) * Rx(cx)
     */
    m.m[0] = cy * cz;
    m.m[1] = sx * sy * cz + cx * sz;
    m.m[2] = -cx * sy * cz + sx * sz;

    m.m[3] = -cy * sz;
    m.m[4] = -sx * sy * sz + cx * cz;
    m.m[5] = cx * sy * sz + sx * cz;

    m.m[6] = sy;
    m.m[7] = -sx * cy;
    m.m[8] = cx * cy;

    return m;
}

/**
 * @brief Convert DCM to CDU angles (DCMTOCDU)
 */
void apollo_dcmtocdu(const apollo_dcm_t *m, apollo_cdu_angles_t *cdu)
{
    /* Extract gimbal angles from DCM */
    double sin_x = -m->m[7];
    double cos_x = m->m[8];

    cdu->x = atan2(sin_x, cos_x);

    cdu->y = asin(m->m[6]);

    double sin_z = -m->m[3];
    double cos_z = m->m[0];
    cdu->z = atan2(sin_z, cos_z);
}

/* =========================================================================
 * DCM Operations
 * ========================================================================= */

/**
 * @brief Compute transformation from initial to current S/M (MIS)
 */
apollo_dcm_t apollo_compute_mis(const apollo_cdu_angles_t *cdu,
                                  const apollo_dcm_t *mis_initial)
{
    apollo_dcm_t current;

    /* Current DCM from CDUs */
    current = apollo_cdutodcm(cdu);

    /* MIS = initial * current^T (to get relative orientation) */
    apollo_dcm_t current_t = apollo_mat3_transpose(&current);
    return apollo_mat3_mul(mis_initial, &current_t);
}

/**
 * @brief Transpose of MIS for coordinate transformation
 */
apollo_dcm_t apollo_compute_tmis(const apollo_dcm_t *mis)
{
    return apollo_mat3_transpose(mis);
}

/* =========================================================================
 * Quaternion Operations
 * ========================================================================= */

/**
 * @brief Convert quaternion to DCM
 */
apollo_dcm_t apollo_quat_to_dcm(const apollo_quat_t *q)
{
    apollo_dcm_t m;
    double q0q0 = q->q0 * q->q0;
    double qxqx = q->qx * q->qx;
    double qyqy = q->qy * q->qy;
    double qzqz = q->qz * q->qz;

    double q0qx = q->q0 * q->qx;
    double q0qy = q->q0 * q->qy;
    double q0qz = q->q0 * q->qz;
    double qxqy = q->qx * q->qy;
    double qxqz = q->qx * q->qz;
    double qyqz = q->qy * q->qz;

    /* DCM from quaternion (Bodock's formula) */
    m.m[0] = q0q0 + qxqx - qyqy - qzqz;
    m.m[1] = 2.0 * (q0qz + qxqy);
    m.m[2] = 2.0 * (qxqz - q0qy);

    m.m[3] = 2.0 * (q0qz - qxqy);
    m.m[4] = q0q0 - qxqx + qyqy - qzqz;
    m.m[5] = 2.0 * (q0qx + qyqz);

    m.m[6] = 2.0 * (q0qy + qxqz);
    m.m[7] = 2.0 * (qyqz - q0qx);
    m.m[8] = q0q0 - qxqx - qyqy + qzqz;

    return m;
}

/**
 * @brief Convert DCM to quaternion
 */
apollo_quat_t apollo_dcm_to_quat(const apollo_dcm_t *m)
{
    apollo_quat_t q;
    double tr = m->m[0] + m->m[4] + m->m[8];

    if (tr > 0) {
        double s = sqrt(tr + 1.0) * 2.0;  /* s = 4 * q0 */
        q.q0 = 0.25 * s;
        q.qx = (m->m[7] - m->m[5]) / s;
        q.qy = (m->m[2] - m->m[6]) / s;
        q.qz = (m->m[3] - m->m[1]) / s;
    } else if ((m->m[0] > m->m[4]) && (m->m[0] > m->m[8])) {
        double s = sqrt(1.0 + m->m[0] - m->m[4] - m->m[8]) * 2.0;
        q.q0 = (m->m[7] - m->m[5]) / s;
        q.qx = 0.25 * s;
        q.qy = (m->m[1] + m->m[3]) / s;
        q.qz = (m->m[2] + m->m[6]) / s;
    } else if (m->m[4] > m->m[8]) {
        double s = sqrt(1.0 + m->m[4] - m->m[0] - m->m[8]) * 2.0;
        q.q0 = (m->m[2] - m->m[6]) / s;
        q.qx = (m->m[1] + m->m[3]) / s;
        q.qy = 0.25 * s;
        q.qz = (m->m[5] + m->m[7]) / s;
    } else {
        double s = sqrt(1.0 + m->m[8] - m->m[0] - m->m[4]) * 2.0;
        q.q0 = (m->m[3] - m->m[1]) / s;
        q.qx = (m->m[2] + m->m[6]) / s;
        q.qy = (m->m[5] + m->m[7]) / s;
        q.qz = 0.25 * s;
    }

    return q;
}

/**
 * @brief Quaternion multiplication
 */
apollo_quat_t apollo_quat_mul(const apollo_quat_t *a, const apollo_quat_t *b)
{
    apollo_quat_t c;
    c.q0 = a->q0 * b->q0 - a->qx * b->qx - a->qy * b->qy - a->qz * b->qz;
    c.qx = a->q0 * b->qx + a->qx * b->q0 + a->qy * b->qz - a->qz * b->qy;
    c.qy = a->q0 * b->qy - a->qx * b->qz + a->qy * b->q0 + a->qz * b->qx;
    c.qz = a->q0 * b->qz + a->qx * b->qy - a->qy * b->qx + a->qz * b->q0;
    return c;
}

/**
 * @brief Quaternion to Euler angles
 */
void apollo_quat_to_euler(const apollo_quat_t *q, double *yaw,
                          double *pitch, double *roll)
{
    /* Roll (x-axis rotation) */
    double sinr_cosp = 2.0 * (q->q0 * q->qx + q->qy * q->qz);
    double cosr_cosp = 1.0 - 2.0 * (q->qx * q->qx + q->qy * q->qy);
    *roll = atan2(sinr_cosp, cosr_cosp);

    /* Pitch (y-axis rotation) */
    double sinp = 2.0 * (q->q0 * q->qy - q->qz * q->qx);
    if (fabs(sinp) >= 1.0) {
        *pitch = copysign(HALF_PI, sinp);  /* Gimbal lock */
    } else {
        *pitch = asin(sinp);
    }

    /* Yaw (z-axis rotation) */
    double siny_cosp = 2.0 * (q->q0 * q->qz + q->qx * q->qy);
    double cosy_cosp = 1.0 - 2.0 * (q->qy * q->qy + q->qz * q->qz);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

/* =========================================================================
 * Gimbal Lock Detection
 * ========================================================================= */

/**
 * @brief Check for gimbal lock condition
 *
 * Gimbal lock occurs when the middle gimbal angle approaches 90 degrees.
 * In this configuration, the first and third gimbals rotate together,
 * causing loss of one degree of freedom.
 */
bool apollo_check_gimbal_lock(const apollo_cdu_angles_t *cdu)
{
    double abs_y = fabs(cdu->y);

    /* Gimbal lock threshold: ~90 degrees */
    #define GIMBAL_LOCK_THRESHOLD (85.0 * M_PI / 180.0)

    return (abs_y > GIMBAL_LOCK_THRESHOLD);
}

/**
 * @brief Avoid gimbal lock by selecting optimal rotation axis
 *
 * When entering gimbal lock, select an alternate rotation sequence
 * that avoids the problematic configuration.
 */
void apollo_avoid_gimbal_lock(const apollo_v3_t *desired_direction,
                               const apollo_dcm_t *current_att,
                               apollo_v3_t *optimal_axis)
{
    /*
     * Strategy: Choose rotation axis perpendicular to the desired
     * direction to minimize gimbal angles during maneuver.
     */
    apollo_v3_t z_axis = {0, 0, 1};
    apollo_v3_t cross = apollo_v3_cross(&z_axis, desired_direction);

    if (apollo_v3_mag(&cross) < 0.01) {
        /* Desired direction is along Z, use X axis */
        *optimal_axis = apollo_v3_unit_x;
    } else {
        *optimal_axis = apollo_v3_unit(&cross);
    }
}

/* =========================================================================
 * Maneuver Control
 * ========================================================================= */

/**
 * @brief Perform automatic maneuver to desired angles
 */
void apollo_maneuver_to_angles(const apollo_commanded_angles_t *target,
                                 apollo_maneuver_rate_t rate,
                                 apollo_v3_t *body_rate)
{
    /*
     * Compute body rates for maneuver.
     * The rate determines how fast we rotate.
     */
    double rate_val;
    switch (rate) {
        case APOLLO_RATE_05_DEG: rate_val = APOLLO_RATE_05_DEG_VAL; break;
        case APOLLO_RATE_2_DEG:  rate_val = APOLLO_RATE_2_DEG_VAL; break;
        case APOLLO_RATE_5_DEG:  rate_val = APOLLO_RATE_5_DEG_VAL; break;
        case APOLLO_RATE_10_DEG: rate_val = APOLLO_RATE_10_DEG_VAL; break;
        default: rate_val = APOLLO_RATE_05_DEG_VAL; break;
    }

    body_rate->x = (target->x / 360.0) * rate_val;
    body_rate->y = (target->y / 360.0) * rate_val;
    body_rate->z = (target->z / 360.0) * rate_val;
}

/* =========================================================================
 * Body Rates
 * ========================================================================= */

/**
 * @brief Compute body rates from CDU angles
 */
apollo_body_rates_t apollo_compute_body_rates(const apollo_cdu_angles_t *cdu_angles,
                                               const apollo_cdu_angles_t *cdu_rates)
{
    apollo_body_rates_t rates;
    double cx, sx, cy, sy;

    cx = cos(cdu_angles->x);
    sx = sin(cdu_angles->x);
    cy = cos(cdu_angles->y);
    sy = sin(cdu_angles->y);

    /*
     * Transform gimbal rates to body rates
     * Uses the inverse of the CDUTODCM transformation
     */
    if (fabs(cy) < 0.001) {
        /* Near gimbal lock - special handling */
        rates.p = cdu_rates->x;
        rates.q = cdu_rates->y * cx + cdu_rates->z;
        rates.r = cdu_rates->y * sx - cdu_rates->z;
    } else {
        rates.p = cdu_rates->x + cdu_rates->z * sx / cy;
        rates.q = cdu_rates->y * cx - cdu_rates->z * sx / cy;
        rates.r = cdu_rates->y * sx + cdu_rates->z * cx / cy;
    }

    return rates;
}

/* =========================================================================
 * TEST DRIVER
 * ========================================================================= */

#ifdef APOLLO_ATTITUDE_TEST

#include <stdio.h>
#include <assert.h>

#define TEST_PRECISION 1e-6

static int test_cdutodcm(void)
{
    printf("Testing CDU to DCM conversion...\n");

    apollo_cdu_angles_t cdu = {0, 0, 0};
    apollo_dcm_t m = apollo_cdutodcm(&cdu);

    /* At zero angles, should be identity */
    assert(fabs(m.m[0] - 1.0) < TEST_PRECISION);
    assert(fabs(m.m[4] - 1.0) < TEST_PRECISION);
    assert(fabs(m.m[8] - 1.0) < TEST_PRECISION);

    printf("  Zero angles: PASS\n");

    /* Test 90 degree rotation about X */
    cdu.x = M_PI / 2.0;
    cdu.y = 0;
    cdu.z = 0;
    m = apollo_cdutodcm(&cdu);

    printf("  90 deg X rotation: DCM = [%.3f, %.3f, %.3f]\n",
           m.m[0], m.m[1], m.m[2]);

    return 0;
}

static int test_quat_dcm_conversion(void)
{
    printf("Testing quaternion <-> DCM conversion...\n");

    apollo_quat_t q = {0.7071, 0.7071, 0, 0};  /* 90 deg about X */
    apollo_dcm_t m = apollo_quat_to_dcm(&q);
    apollo_quat_t q_back = apollo_dcm_to_quat(&m);

    printf("  Original q: [%.4f, %.4f, %.4f, %.4f]\n",
           q.q0, q.qx, q.qy, q.qz);
    printf("  Recovered q: [%.4f, %.4f, %.4f, %.4f]\n",
           q_back.q0, q_back.qx, q_back.qy, q_back.qz);

    double err = fabs(q.q0 - q_back.q0) + fabs(q.qx - q_back.qx);
    return (err < 0.01) ? 0 : 1;
}

static int test_gimbal_lock(void)
{
    printf("Testing gimbal lock detection...\n");

    apollo_cdu_angles_t cdu;

    /* Normal attitude */
    cdu.x = 0; cdu.y = 0; cdu.z = 0;
    assert(!apollo_check_gimbal_lock(&cdu));
    printf("  Normal attitude: No gimbal lock\n");

    /* Near gimbal lock */
    cdu.x = 0; cdu.y = 80.0 * M_PI / 180.0; cdu.z = 0;
    assert(!apollo_check_gimbal_lock(&cdu));
    printf("  80 deg Y: No gimbal lock\n");

    /* Gimbal lock */
    cdu.x = 0; cdu.y = 88.0 * M_PI / 180.0; cdu.z = 0;
    assert(apollo_check_gimbal_lock(&cdu));
    printf("  88 deg Y: GIMBAL LOCK!\n");

    return 0;
}

static int test_quat_mul(void)
{
    printf("Testing quaternion multiplication...\n");

    apollo_quat_t a = {1, 0, 0, 0};  /* Identity */
    apollo_quat_t b = {0.7071, 0.7071, 0, 0};  /* 90 deg about X */
    apollo_quat_t c = apollo_quat_mul(&a, &b);

    printf("  a * b = [%.4f, %.4f, %.4f, %.4f]\n", c.q0, c.qx, c.qy, c.qz);

    /* Should equal b (identity * b = b) */
    double err = fabs(c.qx - b.qx);
    return (err < 0.01) ? 0 : 1;
}

int main(void)
{
    int failures = 0;

    printf("=== Apollo Attitude Module Tests ===\n\n");

    failures += test_cdutodcm();
    failures += test_quat_dcm_conversion();
    failures += test_gimbal_lock();
    failures += test_quat_mul();

    printf("\n=== Summary: %d failures ===\n", failures);

    return failures;
}

#endif /* APOLLO_ATTITUDE_TEST */
