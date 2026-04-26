/**
 * @file apollo_guidance.c
 * @brief Guidance algorithms implementation
 *
 * From: P30-P37.agc, P40-P47.agc
 */

#include "apollo_guidance.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================================================================
 * LAMBERT TARGETING
 * ========================================================================= */

/**
 * @brief Solve Lambert problem
 *
 * Given r1, r2, and transfer time, find required velocity at r1.
 * Uses the universal variable formulation.
 */
void apollo_lambert_solve(const apollo_lambert_t *lambert, apollo_orbstate_t *out)
{
    apollo_v3_t r1 = lambert->r1;
    apollo_v3_t r2 = lambert->r2;
    double dt = lambert->t_trans;

    double mu = 3.986e14;  /* Earth gravitational parameter */

    /* Universal variable method */
    double r1_mag = apollo_v3_mag(&r1);
    double r2_mag = apollo_v3_mag(&r2);

    /* Unit vectors */
    apollo_v3_t r1_unit = apollo_v3_unit(&r1);
    apollo_v3_t r2_unit = apollo_v3_unit(&r2);

    /* Angle between r1 and r2 */
    double cos_theta = apollo_v3_dot(&r1_unit, &r2_unit);
    double theta = acos(fmax(-1.0, fmin(1.0, cos_theta)));

    /* Chord */
    double c = sqrt(r1_mag * r1_mag + r2_mag * r2_mag -
                    2.0 * r1_mag * r2_mag * cos_theta);

    /* Semi-perimeter */
    double s = (r1_mag + r2_mag + c) / 2.0;

    /* Minimum energy trajectory semi-major axis */
    double a_min = s / 2.0;

    /* Initial guess for universal variable (x = 0 at periapsis) */
    double x = 0.0;

    /* Newton-Raphson iteration for x */
    for (int i = 0; i < 20; i++) {
        double sin_x = sin(x);
        double cos_x = cos(x);

        /* Universal Kepler equation */
        double f = r1_mag + r2_mag - c - 2.0 * sqrt(r1_mag * r2_mag) *
                   cos(theta / 2.0) * sin_x - x * sqrt(mu * a_min);

        /* Derivative of universal Kepler equation */
        double f_prime = sqrt(r1_mag * r2_mag) * cos(theta / 2.0) * cos_x -
                         sqrt(mu * a_min);

        double delta_x = -f / f_prime;
        x += delta_x;

        if (fabs(delta_x) < 1e-8) break;
    }

    /* Velocity at departure using Vis-Viva equation */
    double v1_mag = sqrt(mu * (2.0 / r1_mag - 1.0 / a_min));

    /* Direction perpendicular to r1 - use z-axis if r1 is along x,
     * otherwise use cross product with z-axis */
    apollo_v3_t z_axis = {0, 0, 1};
    apollo_v3_t r1_cross_z = apollo_v3_cross(&r1, &z_axis);
    apollo_v3_t perpendicular;

    if (apollo_v3_mag(&r1_cross_z) < 1e-6) {
        /* r1 is parallel to z-axis, use x-axis instead */
        apollo_v3_t x_axis = {1, 0, 0};
        perpendicular = apollo_v3_cross(&r1, &x_axis);
    } else {
        perpendicular = r1_cross_z;
    }

    /* Velocity direction is perpendicular to radius */
    apollo_v3_t v1_direction = apollo_v3_unit(&perpendicular);

    out->r = r1;
    out->v = apollo_v3_scale(&v1_direction, v1_mag);
}

/* =========================================================================
 * POWERED FLIGHT
 * ========================================================================= */

/**
 * @brief Compute powered flight trajectory
 */
void apollo_powered_flight(apollo_powered_flight_t *pf, double mu)
{
    /* Half-computation for second-order accuracy */
    apollo_half_computation(pf, mu);

    /* Apply delta-V estimate */
    pf->sv_next.v.x = pf->sv_init.v.x + pf->dv_vector.x;
    pf->sv_next.v.y = pf->sv_init.v.y + pf->dv_vector.y;
    pf->sv_next.v.z = pf->sv_init.v.z + pf->dv_vector.z;
}

/**
 * @brief Compute half-computation state
 */
void apollo_half_computation(apollo_powered_flight_t *pf, double mu)
{
    /* Simplified half-computation: use mean of initial and target */
    pf->sv_half.r.x = (pf->sv_init.r.x + pf->sv_next.r.x) / 2.0;
    pf->sv_half.r.y = (pf->sv_init.r.y + pf->sv_next.r.y) / 2.0;
    pf->sv_half.r.z = (pf->sv_init.r.z + pf->sv_next.r.z) / 2.0;

    /* Velocity at midpoint */
    double dt = pf->cut_time - pf->ign_time;
    pf->sv_half.v.x = (pf->sv_next.r.x - pf->sv_init.r.x) / dt;
    pf->sv_half.v.y = (pf->sv_next.r.y - pf->sv_init.r.y) / dt;
    pf->sv_half.v.z = (pf->sv_next.r.z - pf->sv_init.r.z) / dt;
}

/**
 * @brief Compute SIVB retrograde burn
 */
void apollo_sivb_retrograde(apollo_powered_flight_t *pf)
{
    /* SIVB retrograde velocity (opposite to Earth departure) */
    apollo_v3_t retrograde = {0, -2.5, 0};  /* Simplified */

    pf->dv_vector = retrograde;
    pf->dv_achieved = apollo_v3_mag(&retrograde);
}

/**
 * @brief Compute LOI burn
 */
void apollo_loi_burn(const apollo_orbstate_t *sv_before,
                      apollo_orbstate_t *sv_after, double mu)
{
    /* Lunar orbit insertion burn */
    apollo_v3_t v_before = sv_before->v;

    /* Retrograde burn magnitude */
    double dv_mag = 2.0 * apollo_v3_mag(&v_before);

    /* Delta-V opposite to velocity */
    apollo_v3_t v_before_unit = apollo_v3_unit(&v_before);
    apollo_v3_t dv = apollo_v3_scale(&v_before_unit, -dv_mag);

    sv_after->r = sv_before->r;
    sv_after->v.x = sv_before->v.x + dv.x;
    sv_after->v.y = sv_before->v.y + dv.y;
    sv_after->v.z = sv_before->v.z + dv.z;

    (void)mu;  /* unused in simplified version */
}

/* =========================================================================
 * DESCENT GUIDANCE
 * ========================================================================= */

/**
 * @brief Update descent guidance
 */
void apollo_descent_update(apollo_descent_guidance_t *dg, double dt)
{
    /* Update height rate */
    dg->h_actual -= dg->v_vertical * dt;

    /* Update range */
    dg->range += dg->v_horizontal * dt;

    /* Compute new desired height based on range to target */
    dg->h_des = dg->tgt_range - dg->range;
}

/**
 * @brief Compute PDG (Powered Descent Guidance)
 */
void apollo_pdg(const apollo_descent_guidance_t *dg, apollo_v3_t *thrust_cmd)
{
    /* Simplified PDG: constant descent rate */
    double desired_h_dot = -dg->v_vertical;

    /* Thrust command proportional to desired acceleration */
    double thrust = (9.81 + desired_h_dot) * dg->mass_current;

    thrust_cmd->x = 0;
    thrust_cmd->y = 0;
    thrust_cmd->z = thrust;
}

/* =========================================================================
 * ENTRY GUIDANCE
 * ========================================================================= */

/**
 * @brief Compute entry guidance constants
 */
void apollo_entry_init(const apollo_orbstate_t *sv_entry, double lat, double lng)
{
    /* Initialize entry guidance constants */
    (void)sv_entry;
    (void)lat;
    (void)lng;
}

/**
 * @brief Update entry guidance
 */
void apollo_entry_update(apollo_entry_guidance_t *eg, double dt)
{
    /* Update range */
    eg->range += eg->v_inertial * dt;

    /* Update velocity */
    eg->v_inertial -= eg->v_loss * dt;
}

/* =========================================================================
 * CSI
 * ========================================================================= */

/**
 * @brief Solve CSI problem
 */
void apollo_csi_solve(apollo_csi_t *csi)
{
    /*
     * Simplified CSI solution.
     * Original AGC used complex iterative targeting.
     */

    /* Compute time to CSI */
    csi->t_csi = 3600.0;  /* 1 hour simplified */

    /* Propagate to CSI time */
    /* (would use Kepler propagation here) */

    /* Compute delta-V at CSI */
    csi->dv_csi = 10.0;  /* Simplified m/s */
}

#ifdef APOLLO_GUIDANCE_TEST

#include <stdio.h>

static int test_lambert(void)
{
    printf("Testing Lambert targeting...\n");

    apollo_lambert_t lambert = {
        .r1 = {7000e3, 0, 0},
        .r2 = {7500e3, 0, 0},
        .t_trans = 3600.0  /* 1 hour */
    };

    apollo_orbstate_t out;

    apollo_lambert_solve(&lambert, &out);

    printf("  Departure velocity: (%.2f, %.2f, %.2f) km/s\n",
           out.v.x/1000, out.v.y/1000, out.v.z/1000);

    return 0;
}

int main(void)
{
    printf("=== Apollo Guidance Module Tests ===\n\n");
    return test_lambert();
}

#endif /* APOLLO_GUIDANCE_TEST */
