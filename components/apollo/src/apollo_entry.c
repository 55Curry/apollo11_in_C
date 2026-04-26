/**
 * @file apollo_entry.c
 * @brief Entry guidance implementation
 *
 * From: P65-P67.agc, REENTRY_CONTROL.agc
 */

#include "apollo_entry.h"
#include <math.h>

/* =========================================================================
 * CM ENTRY INITIALIZATION
 * ========================================================================= */

void apollo_cm_entry_init(apollo_cm_entry_t *entry,
                         const apollo_v3_t *r_entry,
                         const apollo_v3_t *v_entry,
                         double lat, double lng)
{
    entry->phase = APOLLO_ENTRY_PREENTRY;
    entry->state.r = *r_entry;
    entry->state.v = *v_entry;
    entry->state.lat = lat;
    entry->state.lng = lng;
    entry->state.mass = 5000.0;  /* kg, CM mass */
    entry->state.h = 0.0;
    entry->state.dr = 0.0;
    entry->state.lift = 0.0;
    entry->state.q = 0.0;

    entry->cos_lat = cos(lat);
    entry->target_aim = 0.0;
    entry->range_approx = 0.0;
    entry->dv_pred = 0.0;
}

/* =========================================================================
 * CM ENTRY UPDATE
 * ========================================================================= */

void apollo_cm_entry_update(apollo_cm_entry_t *entry, double dt)
{
    (void)dt;

    /* Update based on current phase */
    switch (entry->phase) {
        case APOLLO_ENTRY_PREENTRY:
            entry->phase = APOLLO_ENTRY_INTERFACE;
            break;

        case APOLLO_ENTRY_INTERFACE:
            entry->phase = APOLLO_ENTRY_SUPERSONIC;
            break;

        case APOLLO_ENTRY_SUPERSONIC:
            /* Continue until velocity threshold */
            if (apollo_v3_mag(&entry->state.v) < 3000.0) {
                entry->phase = APOLLO_ENTRY_SHALLOW;
            }
            break;

        case APOLLO_ENTRY_SHALLOW:
            if (apollo_v3_mag(&entry->state.v) < 1500.0) {
                entry->phase = APOLLO_ENTRY_ROLL;
            }
            break;

        case APOLLO_ENTRY_ROLL:
            if (apollo_v3_mag(&entry->state.v) < 1000.0) {
                entry->phase = APOLLO_ENTRY_PARACHUTE;
            }
            break;

        default:
            break;
    }

    /* Compute altitude from position magnitude */
    double r = apollo_v3_mag(&entry->state.r);
    double Re = 6378136.0;  /* Earth radius */
    entry->state.h = r - Re;

    /* Compute velocity magnitude */
    entry->state.v_e = apollo_v3_mag(&entry->state.v);
}

/* =========================================================================
 * ENTRY COMMANDS
 * ========================================================================= */

void apollo_entry_commands(const apollo_cm_entry_t *entry,
                         double *roll_cmd,
                         double *pitch_cmd,
                         double *yaw_cmd)
{
    (void)entry;

    /* Default commands */
    *roll_cmd = 0.0;
    *pitch_cmd = 0.0;
    *yaw_cmd = 0.0;
}

apollo_entry_phase_t apollo_check_phase_transition(const apollo_cm_entry_t *entry)
{
    return entry->phase;
}

/* =========================================================================
 * BALLISTIC ENTRY
 * ========================================================================= */

void apollo_ballistic_entry(apollo_entry_state_t *state, double dt)
{
    double v_mag = apollo_v3_mag(&state->v);

    if (v_mag > 1.0) {
        /* Compute drag acceleration */
        double drag = 0.5 * 0.2 * v_mag * v_mag;
        apollo_v3_t v_unit = apollo_v3_unit(&state->v);

        /* Update velocity */
        state->v.x -= drag * v_unit.x * dt;
        state->v.y -= drag * v_unit.y * dt;
        state->v.z -= drag * v_unit.z * dt;

        state->dr = drag;
    }
}

/* =========================================================================
 * LIFTING ENTRY
 * ========================================================================= */

void apollo_lifting_entry(apollo_cm_entry_t *entry, double dt)
{
    apollo_ballistic_entry(&entry->state, dt);

    /* Add lift effect */
    double lift = 0.5 * 0.3 * entry->state.q * entry->state.v_e;
    entry->state.lift = lift;
}

double apollo_bank_command(const apollo_cm_entry_t *entry)
{
    (void)entry;
    return 0.0;  /* Simplified */
}

/* =========================================================================
 * DRAG/HEATING
 * ========================================================================= */

double apollo_heat_rate(double v, double rho)
{
    /* Stagnation point heat rate: q = k * rho^0.5 * v^3 */
    return 1.9e-8 * pow(rho, 0.5) * v * v * v;
}

double apollo_heat_load(double v_entry, double h_entry,
                        double mass, double A_ref)
{
    (void)mass;
    (void)A_ref;
    /* Simplified heat load calculation */
    return 0.01 * v_entry * (h_entry / 1000.0);
}

/* =========================================================================
 * DROGUE/MAIN CHUTES
 * ========================================================================= */

bool apollo_check_drogue(const apollo_entry_state_t *state)
{
    return state->v_e < 1500.0 && state->h < 7000.0;
}

bool apollo_check_main(const apollo_entry_state_t *state)
{
    return state->v_e < 300.0 && state->h < 3000.0;
}

/* =========================================================================
 * LANDING
 * ========================================================================= */

void apollo_landing_update(apollo_landing_t *land, const apollo_entry_state_t *state)
{
    land->v_vertical = state->v.y;  /* Simplified */
    land->v_horizontal = state->v.x;
    land->h_surf = state->h;
}

#ifdef APOLLO_ENTRY_TEST

#include <stdio.h>

int main(void)
{
    printf("=== Apollo Entry Module Tests ===\n");

    apollo_cm_entry_t entry;
    apollo_v3_t r = {6500e3, 0, 0};
    apollo_v3_t v = {-7500.0, 0, 0};

    apollo_cm_entry_init(&entry, &r, &v, 0.0, 0.0);
    printf("Entry initialized: phase=%d, h=%.1f km\n",
           entry.phase, entry.state.h / 1000.0);

    apollo_cm_entry_update(&entry, 1.0);
    printf("After update: phase=%d, v_e=%.1f m/s\n",
           entry.phase, entry.state.v_e);

    return 0;
}

#endif /* APOLLO_ENTRY_TEST */
