/******************************************************************************
 *
 * Module: Scheduler
 *
 * File Name: scheduler_cfg.h
 *
 * Description: Configuration header for cooperative time-triggered scheduler
 *              Designed for TM4C123GH6PM motor control application
 *
 ******************************************************************************/

#ifndef SCHEDULER_CFG_H_
#define SCHEDULER_CFG_H_

/*******************************************************************************
 *                          Scheduler Configuration                            *
 *******************************************************************************/

/* Maximum number of tasks that can be registered */
#define SCHED_MAX_TASKS                 (8U)

/* Scheduler tick period in milliseconds */
/* This is the base time unit - all task periods are multiples of this */
/* For motor control: 1ms tick gives good resolution for PID loops */
#define SCHED_TICK_PERIOD_MS            (1U)

/* Enable/disable scheduler statistics */
/* 1 = Track execution time, CPU load, missed deadlines */
/* 0 = Disable for smaller code size */
#define SCHED_ENABLE_STATISTICS         (1U)

/* Enable/disable task runtime management */
/* 1 = Allow enable/disable/pause tasks at runtime */
/* 0 = Static task list only */
#define SCHED_ENABLE_RUNTIME_CONTROL    (1U)

/* Enable idle hook function */
/* Called when no tasks are ready to run */
#define SCHED_ENABLE_IDLE_HOOK          (0U)

/* Enable watchdog refresh in scheduler */
#define SCHED_ENABLE_WATCHDOG           (0U)

/*******************************************************************************
 *                          Task Definitions                                   *
 *******************************************************************************/

/*
 * Define your application tasks here.
 * These are used as task IDs for registration and control.
 * 
 * For motor controller application:
 * - PID control loop (fast, ~10ms)
 * - Encoder reading (fast, ~5-10ms)
 * - UART communication (medium, ~20-50ms)
 * - Telemetry/logging (slow, ~100ms)
 */

typedef enum {
    /* High priority tasks (index = priority, lower = higher priority) */
    // TASK_ENCODER_UPDATE = 0,    /* Read encoders, update distance */
     TASK_PID_CONTROL,           /* PID loop for motor speed control */
     TASK_UART_RX_PROCESS,       /* Process received UART commands */
     TASK_UART_TX_TELEMETRY,     /* Send telemetry data to Pi */
     TASK_ODOMETRY_UPDATE,       /* Update robot odometry */
     TASK_SAFETY_MONITOR,        /* Check for errors, stall detection */
     TASK_LED_HEARTBEAT,         /* Status LED indicator */
     TASK_USER_1,                /* Available for user */
    TASK_DISTANCE_CONTROL=0,
    TASK_UART_STATUS=1,
    TASK_UART_COMMAND=2,
    SCHED_NUM_TASKS=3,

    /* Must be last */
    SCHED_TASK_COUNT
} Sched_TaskId_t;

/* Compile-time check */
#if (SCHED_TASK_COUNT > SCHED_MAX_TASKS)
    #error "SCHED_TASK_COUNT exceeds SCHED_MAX_TASKS"
#endif

/*******************************************************************************
 *                          Recommended Task Periods                           *
 *******************************************************************************/

/*
 * Task period guidelines for motor control:
 * 
 * | Task              | Period    | Rationale                          |
 * |-------------------|-----------|-----------------------------------|
 * | Encoder update    | 5-10 ms   | Fast enough for velocity calc     |
 * | PID control       | 10 ms     | Standard for motor PID            |
 * | UART RX process   | 10-20 ms  | Responsive to commands            |
 * | UART TX telemetry | 50-100 ms | Don't flood Pi with data          |
 * | Odometry          | 20-50 ms  | Position update rate              |
 * | Safety monitor    | 100 ms    | Error checking                    |
 * | LED heartbeat     | 500 ms    | Visual indicator                  |
 */

#define TASK_PERIOD_ENCODER_MS          (5U)
#define TASK_PERIOD_PID_MS              (10U)
#define TASK_PERIOD_UART_RX_MS          (10U)
#define TASK_PERIOD_UART_TX_MS          (50U)
#define TASK_PERIOD_ODOMETRY_MS         (20U)
#define TASK_PERIOD_SAFETY_MS           (100U)
#define TASK_PERIOD_LED_MS              (500U)

#endif /* SCHEDULER_CFG_H_ */
