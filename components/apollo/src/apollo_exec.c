/**
 * @file apollo_exec.c
 * @brief Executive scheduler implementation
 *
 * From: EXECUTIVE.agc, NEW_JOB.agc
 */

#include "apollo_exec.h"
#include <string.h>

/* =========================================================================
 * EXECUTIVE STATE (GLOBAL)
 * ========================================================================= */

static apollo_exec_state_t g_exec = {0};

/* =========================================================================
 * JOB CONTROL
 * ========================================================================= */

void apollo_exec_init(void)
{
    memset(&g_exec, 0, sizeof(apollo_exec_state_t));
    g_exec.current_job = -1;
    g_exec.current_priority = -1;
}

void apollo_exec_run(void)
{
    /* Find highest priority job and run it */
    int16_t highest_priority = 999;
    int16_t highest_idx = -1;

    for (int i = 0; i < APOLLO_MAX_JOBS; i++) {
        if (g_exec.jobs[i].func != NULL &&
            g_exec.jobs[i].state == 0 &&
            g_exec.jobs[i].priority < highest_priority) {
            highest_priority = g_exec.jobs[i].priority;
            highest_idx = i;
        }
    }

    if (highest_idx >= 0) {
        g_exec.current_job = highest_idx;
        g_exec.current_priority = highest_priority;
        g_exec.jobs[highest_idx].func();
    }
}

int apollo_exec_findvac(int16_t priority)
{
    (void)priority;
    for (int i = 0; i < APOLLO_MAX_JOBS; i++) {
        if (g_exec.jobs[i].func == NULL) {
            return i;
        }
    }
    return -1;
}

void apollo_exec_set_task(int16_t pid, apollo_job_func_t func)
{
    for (int i = 0; i < APOLLO_MAX_JOBS; i++) {
        if (g_exec.jobs[i].pid == pid) {
            g_exec.jobs[i].func = func;
            g_exec.jobs[i].state = 0;
            return;
        }
    }

    int slot = apollo_exec_findvac(0);
    if (slot >= 0) {
        g_exec.jobs[slot].pid = pid;
        g_exec.jobs[slot].func = func;
        g_exec.jobs[slot].priority = 0;
        g_exec.jobs[slot].state = 0;
    }
}

void apollo_exec_kill_task(int16_t pid)
{
    for (int i = 0; i < APOLLO_MAX_JOBS; i++) {
        if (g_exec.jobs[i].pid == pid) {
            g_exec.jobs[i].func = NULL;
            g_exec.jobs[i].state = -1;
            return;
        }
    }
}

/* =========================================================================
 * WAITLIST
 * ========================================================================= */

static apollo_waitlist_state_t g_wl = {0};

void apollo_waitlist_init(void)
{
    memset(&g_wl, 0, sizeof(apollo_waitlist_state_t));
}

void apollo_waitlist_add(int16_t time, apollo_waitlist_func_t func, int16_t taskid)
{
    for (int i = 0; i < APOLLO_MAX_WAITLIST; i++) {
        if (g_wl.entries[i].func == NULL) {
            g_wl.entries[i].time = time;
            g_wl.entries[i].func = func;
            g_wl.entries[i].taskid = taskid;
            return;
        }
    }
}

void apollo_waitlist_remove(int16_t taskid)
{
    for (int i = 0; i < APOLLO_MAX_WAITLIST; i++) {
        if (g_wl.entries[i].taskid == taskid) {
            g_wl.entries[i].func = NULL;
            g_wl.entries[i].taskid = -1;
            return;
        }
    }
}

void apollo_waitlist_service(void)
{
    /* Find and execute due waitlist entries */
    for (int i = 0; i < APOLLO_MAX_WAITLIST; i++) {
        if (g_wl.entries[i].func != NULL &&
            g_wl.entries[i].time <= g_wl.current_time) {
            g_wl.entries[i].func();
            g_wl.entries[i].func = NULL;
        }
    }
}

#ifdef APOLLO_EXEC_TEST

#include <stdio.h>

static void test_job(void)
{
    printf("  Test job executed!\n");
}

int main(void)
{
    printf("=== Apollo Executive Module Tests ===\n");

    apollo_exec_init();
    printf("Executive initialized\n");

    apollo_exec_set_task(1, test_job);
    printf("Set task 1\n");

    apollo_exec_run();
    printf("Ran executive\n");

    apollo_waitlist_init();
    printf("Waitlist initialized\n");

    apollo_waitlist_add(100, test_job, 2);
    printf("Added waitlist entry\n");

    return 0;
}

#endif /* APOLLO_EXEC_TEST */
