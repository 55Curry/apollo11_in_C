/**
 * @file apollo_multithread.c
 * @brief Multi-threaded example using pthreads
 *
 * Demonstrates running guidance, navigation, and attitude calculations
 * in parallel threads, similar to how the AGC's Executive/Waitlist
 * system managed concurrent tasks.
 */

#include "apollo.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Thread arguments structure */
typedef struct {
    double dt;
    int thread_id;
} thread_args_t;

/* Shared navigation state protected by mutex */
typedef struct {
    apollo_orbstate_t sv;
    apollo_cdu_angles_t cdu;
    pthread_mutex_t lock;
    int update_count;
} shared_nav_state_t;

static shared_nav_state_t g_nav_state = {0};

/* =========================================================================
 * NAVIGATION THREAD
 * ========================================================================= */

void *nav_thread_func(void *arg)
{
    thread_args_t *args = (thread_args_t *)arg;

    printf("[NavThread %d] Starting navigation update\n", args->thread_id);

    for (int i = 0; i < 5; i++) {
        pthread_mutex_lock(&g_nav_state.lock);

        /* Simulate state propagation */
        apollo_propagate_sv(&g_nav_state.sv, args->dt, APOLLO_MU_EARTH);

        /* Convert quaternion to DCM and back to quaternion (using declared functions) */
        apollo_quat_t q = {0.7071, 0.0, 0.7071, 0.0};
        apollo_dcm_t dcm = apollo_quat_to_dcm(&q);
        (void)dcm;

        g_nav_state.update_count++;

        pthread_mutex_unlock(&g_nav_state.lock);

        printf("[NavThread %d] Update %d: pos=(%.1f, %.1f, %.1f) km\n",
               args->thread_id, i,
               g_nav_state.sv.r.x/1000, g_nav_state.sv.r.y/1000, g_nav_state.sv.r.z/1000);

        /* Simulate computation time */
        struct timespec ts = {0, 10000000}; /* 10ms */
        nanosleep(&ts, NULL);
    }

    printf("[NavThread %d] Finished\n", args->thread_id);
    return NULL;
}

/* =========================================================================
 * GUIDANCE THREAD
 * ========================================================================= */

void *guidance_thread_func(void *arg)
{
    thread_args_t *args = (thread_args_t *)arg;

    printf("[GuidanceThread %d] Starting guidance computation\n", args->thread_id);

    apollo_lambert_t lambert = {
        .r1 = {7000e3, 0, 0},
        .r2 = {7500e3, 0, 0},
        .t_trans = 3600.0
    };

    for (int i = 0; i < 3; i++) {
        apollo_orbstate_t result;

        /* Solve Lambert targeting (computationally intensive) */
        apollo_lambert_solve(&lambert, &result);

        printf("[GuidanceThread %d] Lambert solve %d: dv=(%.2f, %.2f, %.2f) km/s\n",
               args->thread_id, i,
               result.v.x/1000, result.v.y/1000, result.v.z/1000);

        /* Simulate computation time */
        struct timespec ts = {0, 20000000}; /* 20ms */
        nanosleep(&ts, NULL);
    }

    printf("[GuidanceThread %d] Finished\n", args->thread_id);
    return NULL;
}

/* =========================================================================
 * ATTITUDE THREAD
 * ========================================================================= */

void *attitude_thread_func(void *arg)
{
    thread_args_t *args = (thread_args_t *)arg;

    printf("[AttitudeThread %d] Starting attitude determination\n", args->thread_id);

    apollo_quat_t q = {0.7071, 0.7071, 0, 0};

    for (int i = 0; i < 4; i++) {
        /* Convert quaternion to DCM */
        apollo_dcm_t dcm = apollo_quat_to_dcm(&q);

        /* Convert DCM back to quaternion to verify */
        apollo_quat_t q_back = apollo_dcm_to_quat(&dcm);

        /* Check gimbal lock directly with quaternion-derived CDU angles (simplified) */
        double yaw, pitch, roll;
        apollo_quat_to_euler(&q_back, &yaw, &pitch, &roll);

        printf("[AttitudeThread %d] Update %d: yaw=%.2f, pitch=%.2f, roll=%.2f deg\n",
               args->thread_id, i,
               yaw * 180/3.14159, pitch * 180/3.14159, roll * 180/3.14159);

        /* Simulate computation time */
        struct timespec ts = {0, 15000000}; /* 15ms */
        nanosleep(&ts, NULL);
    }

    printf("[AttitudeThread %d] Finished\n", args->thread_id);
    return NULL;
}

/* =========================================================================
 * RCS CONTROL THREAD
 * ========================================================================= */

void *rcs_thread_func(void *arg)
{
    thread_args_t *args = (thread_args_t *)arg;

    printf("[RCSThread %d] Starting RCS control\n", args->thread_id);

    apollo_rcs_state_t rcs;
    apollo_rcs_init(&rcs);

    apollo_body_rates_t cmd = {0.01, 0.005, 0.0};
    apollo_body_rates_t actual = {0.0, 0.0, 0.0};
    apollo_rcs_dap_gains_t gains = {1.0, 0.1, 0.5, 0.1, 0.001, 0.1};

    for (int i = 0; i < 3; i++) {
        /* Compute torque command */
        apollo_v3_t torque = apollo_rcs_torque_cmd(&cmd, &actual, &gains);

        /* Select jets */
        apollo_rcs_jet_selection_t selection;
        apollo_select_jets(&torque, &rcs, &selection);

        /* Update RCS state */
        apollo_rcs_update(&rcs, args->dt);

        printf("[RCSThread %d] Update %d: fuel=%.1f kg, jets_on=%d\n",
               args->thread_id, i, rcs.fuel_remaining, selection.num_jets);

        /* Simulate computation time */
        struct timespec ts = {0, 5000000}; /* 5ms */
        nanosleep(&ts, NULL);
    }

    printf("[RCSThread %d] Finished\n", args->thread_id);
    return NULL;
}

/* =========================================================================
 * MAIN
 * ========================================================================= */

int main(void)
{
    printf("=== Apollo-11 Multi-threaded Simulation ===\n\n");

    /* Initialize shared navigation state */
    pthread_mutex_init(&g_nav_state.lock, NULL);
    g_nav_state.sv.r.x = 6700e3;
    g_nav_state.sv.r.y = 0;
    g_nav_state.sv.r.z = 0;
    g_nav_state.sv.v.x = 0;
    g_nav_state.sv.v.y = 7500.0;
    g_nav_state.sv.v.z = 0;
    g_nav_state.cdu.x = 0;
    g_nav_state.cdu.y = 0;
    g_nav_state.cdu.z = 0;
    g_nav_state.update_count = 0;

    /* Thread handles */
    pthread_t nav_thread, guidance_thread, attitude_thread, rcs_thread;

    /* Thread arguments */
    thread_args_t nav_args = {.dt = 1.0, .thread_id = 1};
    thread_args_t guidance_args = {.dt = 0.0, .thread_id = 2};
    thread_args_t attitude_args = {.dt = 0.1, .thread_id = 3};
    thread_args_t rcs_args = {.dt = 0.1, .thread_id = 4};

    printf("Creating threads...\n\n");

    /* Create threads - each runs a different Apollo subsystem */
    pthread_create(&nav_thread, NULL, nav_thread_func, &nav_args);
    pthread_create(&guidance_thread, NULL, guidance_thread_func, &guidance_args);
    pthread_create(&attitude_thread, NULL, attitude_thread_func, &attitude_args);
    pthread_create(&rcs_thread, NULL, rcs_thread_func, &rcs_args);

    printf("Threads created, waiting for completion...\n\n");

    /* Wait for all threads to complete */
    pthread_join(nav_thread, NULL);
    pthread_join(guidance_thread, NULL);
    pthread_join(attitude_thread, NULL);
    pthread_join(rcs_thread, NULL);

    printf("\n=== All threads completed ===\n");
    printf("Navigation updates: %d\n", g_nav_state.update_count);

    pthread_mutex_destroy(&g_nav_state.lock);

    return 0;
}
