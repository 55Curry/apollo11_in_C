/**
 * @file apollo_tvc.c
 * @brief Thrust Vector Control implementation
 *
 * From: TVCDAPS.agc, TVCINITIALIZE.agc
 */

#include "apollo_tvc.h"
#include <math.h>

/* =========================================================================
 * TVC INITIALIZATION
 * ========================================================================= */

void apollo_tvc_init(const apollo_engine_config_t *config,
                      apollo_tvc_state_t *state)
{
    state->pitch.position = 0.0;
    state->pitch.rate = 0.0;
    state->pitch.cmd_position = 0.0;
    state->pitch.cmd_rate = 0.0;

    state->yaw.position = 0.0;
    state->yaw.rate = 0.0;
    state->yaw.cmd_position = 0.0;
    state->yaw.cmd_rate = 0.0;

    state->roll.position = 0.0;
    state->roll.rate = 0.0;
    state->roll.cmd_position = 0.0;
    state->roll.cmd_rate = 0.0;

    state->pitch_cmd = 0.0;
    state->yaw_cmd = 0.0;
    state->roll_cmd = 0.0;
    state->enabled = false;

    (void)config;  /* Unused in simplified version */
}

void apollo_tvc_reset(apollo_tvc_state_t *state)
{
    apollo_tvc_init(NULL, state);
}

/* =========================================================================
 * TVC CONTROL
 * ========================================================================= */

void apollo_tvc_update(apollo_tvc_state_t *state, double dt)
{
    if (!state->enabled) return;

    /* Update pitch */
    double pitch_error = state->pitch_cmd - state->pitch.position;
    state->pitch.rate = pitch_error / dt;
    state->pitch.position += state->pitch.rate * dt;

    /* Update yaw */
    double yaw_error = state->yaw_cmd - state->yaw.position;
    state->yaw.rate = yaw_error / dt;
    state->yaw.position += state->yaw.rate * dt;

    /* Update roll (if present) */
    double roll_error = state->roll_cmd - state->roll.position;
    state->roll.rate = roll_error / dt;
    state->roll.position += state->roll.rate * dt;
}

void apollo_tvc_set_cmd(apollo_tvc_state_t *state, double pitch, double yaw)
{
    state->pitch_cmd = pitch;
    state->yaw_cmd = yaw;
}

void apollo_tvc_enable(apollo_tvc_state_t *state)
{
    state->enabled = true;
}

void apollo_tvc_disable(apollo_tvc_state_t *state)
{
    state->enabled = false;
}

/* =========================================================================
 * TVC DAP
 * ========================================================================= */

double apollo_tvc_dap(double cmd, double actual, double rate,
                      const apollo_tvc_dap_gains_t *gains)
{
    double error = cmd - actual;

    /* PID control */
    double p_term = gains->kp * error;
    double d_term = gains->kd * rate;

    double output = p_term - d_term;

    /* Limit output */
    if (output > gains->limit) output = gains->limit;
    if (output < -gains->limit) output = -gains->limit;

    return output;
}

/* =========================================================================
 * TVC SERVICER
 * ========================================================================= */

void apollo_tvc_servicer_update(apollo_tvc_servicer_t *svc, double dt)
{
    if (!svc->eng_on) return;

    /* Update mass */
    svc->mass_current -= svc->mass_flow_rate * dt;

    /* Update delta-V achieved */
    double thrust = svc->mass_flow_rate * svc->isp * 9.81;
    double dv = thrust * dt / svc->mass_current;
    svc->dv_achieved += dv;
}

/* =========================================================================
 * TVC ERRORS
 * ========================================================================= */

apollo_tvc_error_t apollo_tvc_check(const apollo_tvc_state_t *state)
{
    if (!state->enabled) return APOLLO_TVC_OK;

    /* Check for actuator faults */
    if (fabs(state->pitch.rate) > 100.0) return APOLLO_TVC_ACTUATOR_FAULT;
    if (fabs(state->yaw.rate) > 100.0) return APOLLO_TVC_ACTUATOR_FAULT;

    /* Check for loop failure (error too large) */
    if (fabs(state->pitch_cmd - state->pitch.position) > 0.1) return APOLLO_TVC_LOOP_FAILURE;
    if (fabs(state->yaw_cmd - state->yaw.position) > 0.1) return APOLLO_TVC_LOOP_FAILURE;

    return APOLLO_TVC_OK;
}

#ifdef APOLLO_TVC_TEST

#include <stdio.h>

int main(void)
{
    printf("=== Apollo TVC Module Tests ===\n");

    apollo_tvc_state_t state;
    apollo_tvc_init(NULL, &state);

    printf("TVC initialized: enabled=%d\n", state.enabled);

    apollo_tvc_enable(&state);
    apollo_tvc_set_cmd(&state, 0.05, -0.03);

    for (int i = 0; i < 10; i++) {
        apollo_tvc_update(&state, 0.1);
    }

    printf("After update: pitch=%.4f, yaw=%.4f\n",
           state.pitch.position, state.yaw.position);

    return 0;
}

#endif /* APOLLO_TVC_TEST */
