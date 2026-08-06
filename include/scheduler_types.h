/******************************************************************************
 *
 * Module: Scheduler
 *
 * File Name: scheduler_types.h
 *
 * Description: Type definitions for cooperative time-triggered scheduler
 *
 ******************************************************************************/

#ifndef SCHEDULER_TYPES_H_
#define SCHEDULER_TYPES_H_

#include "Std_Types.h"
#include "scheduler_cfg.h"

/*******************************************************************************
 *                          Status Types                                       *
 *******************************************************************************/

typedef enum {
    SCHED_OK = 0,
    SCHED_ERROR_NULL_PTR,
    SCHED_ERROR_INVALID_TASK_ID,
    SCHED_ERROR_INVALID_PERIOD,
    SCHED_ERROR_INVALID_PRIORITY,
    SCHED_ERROR_TASK_EXISTS,
    SCHED_ERROR_TASK_NOT_FOUND,
    SCHED_ERROR_MAX_TASKS_REACHED,
    SCHED_ERROR_NOT_INITIALIZED
} Sched_StatusType;

typedef enum {
    SCHED_STATE_UNINIT = 0,
    SCHED_STATE_INIT,
    SCHED_STATE_RUNNING,
    SCHED_STATE_STOPPED,
    SCHED_STATE_PAUSED
} Sched_StateType;

/*******************************************************************************
 *                          Task Types                                         *
 *******************************************************************************/

typedef enum {
    TASK_STATE_DISABLED = 0,
    TASK_STATE_READY,
    TASK_STATE_RUNNING,
    TASK_STATE_SUSPENDED
} Sched_TaskStateType;

/* Task callback function type */
/* void TaskFunction(void *arg) */
typedef void (*Sched_TaskCallback_t)(void *arg);

/*******************************************************************************
 *                          Task Configuration Structure                       *
 *******************************************************************************/

typedef struct {
    const char              *name;          /* Task name (for debugging) */
    Sched_TaskCallback_t    callback;       /* Task function pointer */
    void                    *arg;           /* Argument passed to callback */
    uint32                  periodMs;       /* Execution period in ms */
    uint32                  offsetMs;       /* Initial delay before first run */
    uint8                   priority;       /* Priority (0 = highest) */
    boolean                 autoStart;      /* Start enabled on Sched_Start() */
} Sched_TaskConfig_t;

/*******************************************************************************
 *                          Task Runtime Structure (Internal)                  *
 *******************************************************************************/

typedef struct {
    Sched_TaskConfig_t      config;         /* Task configuration */
    Sched_TaskStateType     state;          /* Current task state */
    uint32                  periodTicks;    /* Period in scheduler ticks */
    uint32                  offsetTicks;    /* Remaining offset ticks */
    uint32                  nextRunTick;    /* Next tick to run */
    uint32                  runCount;       /* Number of times executed */
    boolean                 registered;     /* Is task registered */
#if (SCHED_ENABLE_STATISTICS == 1U)
    uint32                  lastRunTime;    /* Last execution time (us) */
    uint32                  maxRunTime;     /* Maximum execution time (us) */
    uint32                  totalRunTime;   /* Cumulative execution time */
    uint32                  missedDeadlines;/* Missed deadline count */
#endif
} Sched_TaskHandle_t;

/*******************************************************************************
 *                          Scheduler Statistics                               *
 *******************************************************************************/

#if (SCHED_ENABLE_STATISTICS == 1U)
typedef struct {
    uint32                  tickCount;      /* Total scheduler ticks */
    uint32                  taskSwitches;   /* Total task executions */
    uint32                  idleTime;       /* Time spent idle (us) */
    uint32                  busyTime;       /* Time spent in tasks (us) */
    uint8                   cpuLoad;        /* CPU load percentage (0-100) */
    uint32                  maxTickTime;    /* Maximum time in one tick (us) */
    uint32                  missedTicks;    /* Number of overrun ticks */
} Sched_Stats_t;
#endif

/*******************************************************************************
 *                          Callback Types                                     *
 *******************************************************************************/

/* Idle hook - called when no tasks ready */
typedef void (*Sched_IdleHook_t)(void);

/* Error hook - called on scheduler error */
typedef void (*Sched_ErrorHook_t)(Sched_StatusType error);

/* Tick hook - called every scheduler tick */
typedef void (*Sched_TickHook_t)(uint32 tickCount);

#endif /* SCHEDULER_TYPES_H_ */
