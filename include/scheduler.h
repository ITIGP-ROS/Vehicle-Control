/******************************************************************************
 *
 * Module: Scheduler
 *
 * File Name: scheduler.h
 *
 * Description: Header file for cooperative time-triggered scheduler
 *              Designed for TM4C123GH6PM motor control application
 *
 ******************************************************************************/

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "scheduler_types.h"
#include "scheduler_cfg.h"

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

/**
 * @brief  Initialize the scheduler
 * @param  tickPeriodMs: Base tick period in milliseconds (0 = use config default)
 * @return SCHED_OK on success
 * @note   This initializes SysTick timer internally
 */
Sched_StatusType Sched_Init(uint32 tickPeriodMs);

/**
 * @brief  De-initialize the scheduler
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_DeInit(void);

/*******************************************************************************
 *                          Task Registration                                  *
 *******************************************************************************/

/**
 * @brief  Register a task with the scheduler
 * @param  taskId: Task ID from Sched_TaskId_t enum
 * @param  config: Pointer to task configuration
 * @return SCHED_OK on success
 * 
 * @example
 *   Sched_TaskConfig_t pidConfig = {
 *       .name = "PID",
 *       .callback = PID_Task,
 *       .arg = NULL,
 *       .periodMs = 10,
 *       .offsetMs = 0,
 *       .priority = 1,
 *       .autoStart = TRUE
 *   };
 *   Sched_RegisterTask(TASK_PID_CONTROL, &pidConfig);
 */
Sched_StatusType Sched_RegisterTask(uint8 taskId, const Sched_TaskConfig_t *config);

/**
 * @brief  Register a task with minimal parameters
 * @param  taskId: Task ID
 * @param  callback: Task function
 * @param  periodMs: Execution period
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_RegisterTaskSimple(uint8 taskId, Sched_TaskCallback_t callback, uint32 periodMs);

/**
 * @brief  Unregister a task
 * @param  taskId: Task ID to remove
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_UnregisterTask(uint8 taskId);

/*******************************************************************************
 *                          Scheduler Control                                  *
 *******************************************************************************/

/**
 * @brief  Start the scheduler (enters infinite loop)
 * @note   This function never returns under normal operation
 */
void Sched_Start(void);

/**
 * @brief  Stop the scheduler
 * @return SCHED_OK on success
 * @note   Call from within a task to stop scheduler after current tick
 */
Sched_StatusType Sched_Stop(void);

/**
 * @brief  Pause the scheduler (stops executing tasks but keeps timing)
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_Pause(void);

/**
 * @brief  Resume the scheduler after pause
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_Resume(void);

/*******************************************************************************
 *                          Task Control                                       *
 *******************************************************************************/

/**
 * @brief  Enable a task
 * @param  taskId: Task ID to enable
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_EnableTask(uint8 taskId);

/**
 * @brief  Disable a task
 * @param  taskId: Task ID to disable
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_DisableTask(uint8 taskId);

/**
 * @brief  Suspend a task temporarily
 * @param  taskId: Task ID to suspend
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_SuspendTask(uint8 taskId);

/**
 * @brief  Resume a suspended task
 * @param  taskId: Task ID to resume
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_ResumeTask(uint8 taskId);

/**
 * @brief  Change task period at runtime
 * @param  taskId: Task ID
 * @param  periodMs: New period in milliseconds
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_SetTaskPeriod(uint8 taskId, uint32 periodMs);

/**
 * @brief  Get task state
 * @param  taskId: Task ID
 * @return Task state
 */
Sched_TaskStateType Sched_GetTaskState(uint8 taskId);

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

/**
 * @brief  Get scheduler state
 * @return Current scheduler state
 */
Sched_StateType Sched_GetState(void);

/**
 * @brief  Get current tick count
 * @return Number of ticks since scheduler started
 */
uint32 Sched_GetTickCount(void);

/**
 * @brief  Get elapsed time in milliseconds
 * @return Milliseconds since scheduler started
 */
uint32 Sched_GetTimeMs(void);

/**
 * @brief  Check if scheduler is running
 * @return TRUE if running
 */
boolean Sched_IsRunning(void);

/*******************************************************************************
 *                          Statistics Functions                               *
 *******************************************************************************/

#if (SCHED_ENABLE_STATISTICS == 1U)

/**
 * @brief  Get scheduler statistics
 * @param  stats: Pointer to statistics structure to fill
 * @return SCHED_OK on success
 */
Sched_StatusType Sched_GetStats(Sched_Stats_t *stats);

/**
 * @brief  Get task execution count
 * @param  taskId: Task ID
 * @return Number of times task has executed
 */
uint32 Sched_GetTaskRunCount(uint8 taskId);

/**
 * @brief  Get task maximum execution time
 * @param  taskId: Task ID
 * @return Maximum execution time in microseconds
 */
uint32 Sched_GetTaskMaxTime(uint8 taskId);

/**
 * @brief  Get CPU load percentage
 * @return CPU load (0-100%)
 */
uint8 Sched_GetCpuLoad(void);

/**
 * @brief  Reset all statistics
 */
void Sched_ResetStats(void);

#endif /* SCHED_ENABLE_STATISTICS */

/*******************************************************************************
 *                          Hook Functions                                     *
 *******************************************************************************/

/**
 * @brief  Set idle hook (called when no tasks to run)
 * @param  hook: Idle hook function (NULL to disable)
 */
void Sched_SetIdleHook(Sched_IdleHook_t hook);

/**
 * @brief  Set error hook (called on scheduler error)
 * @param  hook: Error hook function (NULL to disable)
 */
void Sched_SetErrorHook(Sched_ErrorHook_t hook);

/**
 * @brief  Set tick hook (called every tick)
 * @param  hook: Tick hook function (NULL to disable)
 */
void Sched_SetTickHook(Sched_TickHook_t hook);

/*******************************************************************************
 *                          Utility Functions                                  *
 *******************************************************************************/

/**
 * @brief  Delay for specified milliseconds (blocking)
 * @param  ms: Milliseconds to delay
 * @note   Use sparingly - blocks scheduler during delay
 */
void Sched_DelayMs(uint32 ms);

/**
 * @brief  Get task name (for debugging)
 * @param  taskId: Task ID
 * @return Task name string or "Unknown"
 */
const char* Sched_GetTaskName(uint8 taskId);

#endif /* SCHEDULER_H_ */
