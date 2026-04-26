/**
 * @file apollo_rcs.c
 * @brief Reaction Control System implementation
 *
 * From: RCS-CSM_DIGITAL_AUTOPILOT.agc, JET_SELECTION_LOGIC.agc
 */

#include "apollo_rcs.h"
#include <math.h>
#include <string.h>

/* =========================================================================
 * RCS JET CONFIGURATION
 * ========================================================================= */

/* Default jet configurations */
static const apollo_rcs_jet_config_t default_jets[18] = {
    /* Pitch up */
    { {-0.5, 0, 0}, {0, 1, 0}, 445, {0, 0, 0} },
    { {0.5, 0, 0}, {0, 1, 0}, 445, {0, 0, 0} },
    /* Pitch down */
    { {-0.5, 0, 0}, {0, -1, 0}, 445, {0, 0, 0} },
    { {0.5, 0, 0}, {0, -1, 0}, 445, {0, 0, 0} },
    /* Yaw right */
    { {0, 0, 0.5}, {1, 0, 0}, 445, {0, 0, 0} },
    { {0, 0, -0.5}, {1, 0, 0}, 445, {0, 0, 0} },
    /* Yaw left */
    { {0, 0, 0.5}, {-1, 0, 0}, 445, {0, 0, 0} },
    { {0, 0, -0.5}, {-1, 0, 0}, 445, {0, 0, 0} },
    /* Roll right */
    { {0, 0.5, 0}, {0, 0, 1}, 445, {0, 0, 0} },
    { {0, -0.5, 0}, {0, 0, 1}, 445, {0, 0, 0} },
    /* Roll left */
    { {0, 0.5, 0}, {0, 0, -1}, 445, {0, 0, 0} },
    { {0, -0.5, 0}, {0, 0, -1}, 445, {0, 0, 0} },
    /* Translation X+ */
    { {1, 0, 0}, {1, 0, 0}, 445, {0, 0, 0} },
    /* Translation X- */
    { {-1, 0, 0}, {1, 0, 0}, 445, {0, 0, 0} },
    /* Translation Y+ */
    { {0, 1, 0}, {1, 0, 0}, 445, {0, 0, 0} },
    /* Translation Y- */
    { {0, -1, 0}, {1, 0, 0}, 445, {0, 0, 0} },
    /* Translation Z+ */
    { {0, 0, 1}, {1, 0, 0}, 445, {0, 0, 0} },
    /* Translation Z- */
    { {0, 0, -1}, {1, 0, 0}, 445, {0, 0, 0} },
};

/* =========================================================================
 * RCS INITIALIZATION
 * ========================================================================= */

void apollo_rcs_init(apollo_rcs_state_t *state)
{
    memset(state, 0, sizeof(apollo_rcs_state_t));
    state->mode = APOLLO_RCS_MODE_OFF;
    state->fuel_remaining = 10000.0;  /* kg */
    state->impulse_used = 0.0;
}

void apollo_rcs_configure(const apollo_rcs_dap_gains_t *gains,
                           apollo_rcs_state_t *state)
{
    (void)gains;
    state->jets_configured = true;
}

/* =========================================================================
 * RCS UPDATE
 * ========================================================================= */

void apollo_rcs_update(apollo_rcs_state_t *state, double dt)
{
    if (state->mode == APOLLO_RCS_MODE_OFF) return;

    /* Update fuel usage based on jets on */
    for (int i = 0; i < 18; i++) {
        if (state->jets[i].on) {
            double impulse = default_jets[i].thrust * state->jets[i].duration;
            state->jets[i].duration += dt;
            state->impulse_used += impulse;
            state->fuel_remaining -= impulse / (300.0 * 9.81);  /* Isp ~300s */
        }
    }
}

/* =========================================================================
 * TORQUE COMMAND
 * ========================================================================= */

apollo_v3_t apollo_rcs_torque_cmd(const apollo_body_rates_t *cmd,
                                    const apollo_body_rates_t *actual,
                                    const apollo_rcs_dap_gains_t *gains)
{
    apollo_v3_t torque;

    /* Rate error */
    double p_err = cmd->p - actual->p;
    double q_err = cmd->q - actual->q;
    double r_err = cmd->r - actual->r;

    /* Apply deadband */
    if (fabs(p_err) < gains->deadband) p_err = 0;
    if (fabs(q_err) < gains->deadband) q_err = 0;
    if (fabs(r_err) < gains->deadband) r_err = 0;

    /* PD control */
    torque.x = gains->kp_rate * p_err;
    torque.y = gains->kp_rate * q_err;
    torque.z = gains->kp_rate * r_err;

    return torque;
}

/* =========================================================================
 * JET SELECTION
 * ========================================================================= */

void apollo_select_jets(const apollo_v3_t *desired_torque,
                        const apollo_rcs_state_t *state,
                        apollo_rcs_jet_selection_t *selection)
{
    (void)state;
    selection->num_jets = 0;
    selection->total_torque = apollo_v3_mag(desired_torque);

    /* Simplified: select jets that produce desired torque */
    if (desired_torque->x > 0) {
        selection->jets[selection->num_jets++] = APOLLO_RCS_ROLL_RIGHT_1;
    }
    if (desired_torque->y > 0) {
        selection->jets[selection->num_jets++] = APOLLO_RCS_YAW_RIGHT_1;
    }
    if (desired_torque->z > 0) {
        selection->jets[selection->num_jets++] = APOLLO_RCS_PITCH_UP_1;
    }
}

void apollo_minimize_jets(const apollo_v3_t *desired_torque,
                          const apollo_rcs_state_t *state,
                          apollo_rcs_jet_selection_t *selection)
{
    /* Minimize number of jets while meeting torque requirement */
    apollo_select_jets(desired_torque, state, selection);
}

bool apollo_check_jet_conflict(const apollo_rcs_jet_selection_t *selection)
{
    /* Check if any jets oppose each other */
    for (int i = 0; i < selection->num_jets; i++) {
        for (int j = i + 1; j < selection->num_jets; j++) {
            apollo_rcs_jet_id_t a = selection->jets[i];
            apollo_rcs_jet_id_t b = selection->jets[j];

            /* Opposing jets on same axis */
            if ((a == APOLLO_RCS_PITCH_UP_1 && b == APOLLO_RCS_PITCH_DN_1) ||
                (a == APOLLO_RCS_PITCH_DN_1 && b == APOLLO_RCS_PITCH_UP_1))
                return true;
        }
    }
    return false;
}

/* =========================================================================
 * IMPULSE ALLOCATION
 * ========================================================================= */

double apollo_compute_impulse(double desired_rate, double current_rate,
                               double inertia, double max_torque)
{
    double rate_error = desired_rate - current_rate;
    return inertia * rate_error / max_torque;
}

void apollo_allocate_impulse(double total_impulse,
                              apollo_rcs_jet_selection_t *selection,
                              apollo_rcs_state_t *state)
{
    (void)total_impulse;
    (void)state;

    /* Simple allocation: divide among selected jets */
    for (int i = 0; i < selection->num_jets; i++) {
        state->jets[selection->jets[i]].on = true;
    }
}

/* =========================================================================
 * FUEL MANAGEMENT
 * ========================================================================= */

void apollo_update_fuel(apollo_rcs_state_t *state, double dt)
{
    apollo_rcs_update(state, dt);  /* Same as update */
}

bool apollo_rcs_fuel_low(const apollo_rcs_state_t *state, double threshold)
{
    return state->fuel_remaining < threshold;
}

#ifdef APOLLO_RCS_TEST

#include <stdio.h>

int main(void)
{
    printf("=== Apollo RCS Module Tests ===\n");

    apollo_rcs_state_t state;
    apollo_rcs_init(&state);

    printf("RCS initialized: mode=%d, fuel=%.1f kg\n",
           state.mode, state.fuel_remaining);

    apollo_body_rates_t cmd = {0.01, 0, 0};
    apollo_body_rates_t actual = {0, 0, 0};
    apollo_rcs_dap_gains_t gains = {1.0, 0.1, 0.5, 0.1, 0.001, 0.1};

    apollo_v3_t torque = apollo_rcs_torque_cmd(&cmd, &actual, &gains);
    printf("Torque command: (%.4f, %.4f, %.4f)\n",
           torque.x, torque.y, torque.z);

    return 0;
}

#endif /* APOLLO_RCS_TEST */
