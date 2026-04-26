/**
 * @file apollo_exec.h
 * @brief Executive (Task Scheduling)
 *
 * From: EXECUTIVE.agc, WAITLIST.agc
 */

#ifndef APOLLO_EXEC_H
#define APOLLO_EXEC_H

#include "apollo_types.h"

/* =========================================================================
 * JOB CONTROL
 * ========================================================================= */

#define APOLLO_MAX_JOBS 8
#define APOLLO_MAX_PRIORITY 99

typedef void (*apollo_job_func_t)(void);

typedef struct {
    int16_t pid;
    int16_t priority;
    apollo_job_func_t func;
    int16_t state;
} apollo_job_t;

typedef struct {
    apollo_job_t jobs[APOLLO_MAX_JOBS];
    int16_t current_job;
    int16_t current_priority;
} apollo_exec_state_t;

/* =========================================================================
 * WAITLIST
 * ========================================================================= */

#define APOLLO_MAX_WAITLIST 10

typedef void (*apollo_waitlist_func_t)(void);

typedef struct {
    int16_t time;
    apollo_waitlist_func_t func;
    int16_t taskid;
} apollo_waitlist_entry_t;

typedef struct {
    apollo_waitlist_entry_t entries[APOLLO_MAX_WAITLIST];
    int16_t current_time;
} apollo_waitlist_state_t;

void apollo_exec_init(void);
void apollo_exec_run(void);
int apollo_exec_findvac(int16_t priority);
void apollo_exec_set_task(int16_t pid, apollo_job_func_t func);
void apollo_exec_kill_task(int16_t pid);

void apollo_waitlist_init(void);
void apollo_waitlist_add(int16_t time, apollo_waitlist_func_t func, int16_t taskid);
void apollo_waitlist_remove(int16_t taskid);
void apollo_waitlist_service(void);

#endif /* APOLLO_EXEC_H */
