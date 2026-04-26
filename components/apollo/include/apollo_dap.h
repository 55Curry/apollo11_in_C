/**
 * @file apollo_dap.h
 * @brief Digital Autopilot
 *
 * From: RCS-CSM_DIGITAL_AUTOPILOT.agc, AUTOMATIC_MANEUVERS.agc
 */

#ifndef APOLLO_DAP_H
#define APOLLO_DAP_H

#include "apollo_types.h"
#include "apollo_vecmath.h"
#include "apollo_attitude.h"
#include "apollo_rcs.h"
#include "apollo_tvc.h"

/* =========================================================================
 * DAP MODES
 * ========================================================================= */

typedef enum {
    APOLLO_DAP_AUTO = 0,
    APOLLO_DAP_MANUAL = 1,
    APOLLO_DAP_ATT_HOLD = 2,
    APOLLO_DAP_RATE_HOLD = 3,
    APOLLO_DAP_MINIMUM_IMPULSE = 4
} apollo_dap_mode_t;

/* =========================================================================
 * DAP STATE
 * ========================================================================= */

typedef struct {
    apollo_dap_mode_t mode;
    apollo_body_rates_t rate_deadband;
    apollo_body_rates_t rate_limit;
    double att_deadband;
    bool tvc_connected;
    bool rcs_connected;
} apollo_dap_config_t;

typedef struct {
    apollo_dap_config_t config;
    apollo_body_rates_t cmd_rates;
    apollo_cdu_angles_t cmd_angles;
    apollo_body_rates_t err_rates;
    apollo_v3_t err_att;
} apollo_dap_state_t;

void apollo_dap_init(apollo_dap_state_t *state, const apollo_dap_config_t *config);
void apollo_dap_update(apollo_dap_state_t *state, double dt);
void apollo_dap_set_mode(apollo_dap_state_t *state, apollo_dap_mode_t mode);

#endif /* APOLLO_DAP_H */
