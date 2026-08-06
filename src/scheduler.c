/******************************************************************************
 *
 * Module: Scheduler
 *
 * File Name: scheduler.c
 *
 * Description: Source file for cooperative time-triggered scheduler
 *              Designed for TM4C123GH6PM motor control application
 *
 ******************************************************************************/

#include "scheduler.h"
#include "systick.h"

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

/* Scheduler state */
static Sched_StateType Sched_State = SCHED_STATE_UNINIT;

/* Task array */
static Sched_TaskHandle_t Sched_Tasks[SCHED_MAX_TASKS];

/* Timing */
static volatile uint32 Sched_TickCount = 0;
static volatile boolean Sched_TickFlag = FALSE;
static uint32 Sched_TickPeriodMs = SCHED_TICK_PERIOD_MS;

/* Control flags */
static volatile boolean Sched_StopRequest = FALSE;
static volatile boolean Sched_PauseRequest = FALSE;

/* Hook functions */
static Sched_IdleHook_t Sched_IdleHook = NULL_PTR;
static Sched_ErrorHook_t Sched_ErrorHook = NULL_PTR;
static Sched_TickHook_t Sched_TickHook = NULL_PTR;

/* Statistics */
#if (SCHED_ENABLE_STATISTICS == 1U)
static Sched_Stats_t Sched_Statistics = {0};
static uint32 Sched_LastTickStart = 0;
#endif

/*******************************************************************************
 *                          Private Functions                                  *
 *******************************************************************************/

/**
 * @brief  SysTick callback - sets tick flag
 */
static void Sched_SysTickCallback(void)
{
    Sched_TickFlag = TRUE;
    Sched_TickCount++;
}

/**
 * @brief  Convert milliseconds to ticks
 */
static uint32 Sched_MsToTicks(uint32 ms)
{
    if (Sched_TickPeriodMs == 0)
    {
        return ms;
    }
    return (ms + Sched_TickPeriodMs - 1) / Sched_TickPeriodMs;
}

/**
 * @brief  Report error via hook
 */
static void Sched_ReportError(Sched_StatusType error)
{
    if (Sched_ErrorHook != NULL_PTR)
    {
        Sched_ErrorHook(error);
    }
}

/**
 * @brief  Execute ready tasks for current tick
 */
static void Sched_ExecuteTasks(void)
{
    uint8 i;
    Sched_TaskHandle_t *task;
    uint32 currentTick = Sched_TickCount;
    
#if (SCHED_ENABLE_STATISTICS == 1U)
    uint32 taskStartTime, taskEndTime, runTime;
#endif
    
    /* Execute tasks in priority order (lower index = higher priority) */
    for (i = 0; i < SCHED_MAX_TASKS; i++)
    {
        task = &Sched_Tasks[i];
        
        /* Skip unregistered or disabled tasks */
        if (!task->registered || task->state != TASK_STATE_READY)
        {
            continue;
        }
        
        /* Check if task should run this tick */
        if (currentTick >= task->nextRunTick)
        {
            /* Handle initial offset */
            if (task->offsetTicks > 0)
            {
                task->offsetTicks--;
                task->nextRunTick = currentTick + 1;
                continue;
            }
            
            /* Execute task */
            if (task->config.callback != NULL_PTR)
            {
                task->state = TASK_STATE_RUNNING;
                
#if (SCHED_ENABLE_STATISTICS == 1U)
                taskStartTime = SysTick_GetTickCount();
#endif
                
                /* Call task function */
                task->config.callback(task->config.arg);
                
#if (SCHED_ENABLE_STATISTICS == 1U)
                taskEndTime = SysTick_GetTickCount();

                /* Calculate execution time */
                runTime = taskEndTime - taskStartTime;
                task->lastRunTime = runTime;
                task->totalRunTime += runTime;
                if (runTime > task->maxRunTime)
                {
                    task->maxRunTime = runTime;
                }
                
                Sched_Statistics.taskSwitches++;
                Sched_Statistics.busyTime += runTime;
#endif
                
                task->runCount++;
                task->state = TASK_STATE_READY;
            }
            
            /* Schedule next run */
            task->nextRunTick = currentTick + task->periodTicks;
            
#if (SCHED_ENABLE_STATISTICS == 1U)
            /* Check for missed deadline */
            if (task->nextRunTick <= currentTick)
            {
                task->missedDeadlines++;
                task->nextRunTick = currentTick + task->periodTicks;
            }
#endif
        }
    }
}

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

Sched_StatusType Sched_Init(uint32 tickPeriodMs)
{
    uint8 i;
    SysTick_StatusType sysStatus;
    
    /* Use default if not specified */
    if (tickPeriodMs == 0)
    {
        tickPeriodMs = SCHED_TICK_PERIOD_MS;
    }
    
    Sched_TickPeriodMs = tickPeriodMs;
    
    /* Initialize task array */
    for (i = 0; i < SCHED_MAX_TASKS; i++)
    {
        Sched_Tasks[i].registered = FALSE;
        Sched_Tasks[i].state = TASK_STATE_DISABLED;
        Sched_Tasks[i].runCount = 0;
        Sched_Tasks[i].nextRunTick = 0;
        Sched_Tasks[i].offsetTicks = 0;
        
#if (SCHED_ENABLE_STATISTICS == 1U)
        Sched_Tasks[i].lastRunTime = 0;
        Sched_Tasks[i].maxRunTime = 0;
        Sched_Tasks[i].totalRunTime = 0;
        Sched_Tasks[i].missedDeadlines = 0;
#endif
    }
    
    /* Reset control flags */
    Sched_TickCount = 0;
    Sched_TickFlag = FALSE;
    Sched_StopRequest = FALSE;
    Sched_PauseRequest = FALSE;
    
#if (SCHED_ENABLE_STATISTICS == 1U)
    Sched_Statistics.tickCount = 0;
    Sched_Statistics.taskSwitches = 0;
    Sched_Statistics.idleTime = 0;
    Sched_Statistics.busyTime = 0;
    Sched_Statistics.cpuLoad = 0;
    Sched_Statistics.maxTickTime = 0;
    Sched_Statistics.missedTicks = 0;
#endif
    
    /* Initialize SysTick for scheduler timing */
    sysStatus = SysTick_Init(tickPeriodMs);
    if (sysStatus != SYSTICK_OK)
    {
        return SCHED_ERROR_INVALID_PERIOD;
    }
    
    /* Set SysTick callback */
    SysTick_SetCallback(Sched_SysTickCallback);
    
    /* Stop SysTick until Sched_Start() is called */
    SysTick_Stop();
    
    Sched_State = SCHED_STATE_INIT;
    
    return SCHED_OK;
}

Sched_StatusType Sched_DeInit(void)
{
    SysTick_Stop();
    SysTick_DeInit();
    
    Sched_State = SCHED_STATE_UNINIT;
    
    return SCHED_OK;
}

/*******************************************************************************
 *                          Task Registration                                  *
 *******************************************************************************/

Sched_StatusType Sched_RegisterTask(uint8 taskId, const Sched_TaskConfig_t *config)
{
    Sched_TaskHandle_t *task;
    
    if (Sched_State == SCHED_STATE_UNINIT)
    {
        return SCHED_ERROR_NOT_INITIALIZED;
    }
    
    if (config == NULL_PTR)
    {
        return SCHED_ERROR_NULL_PTR;
    }
    
    if (taskId >= SCHED_MAX_TASKS)
    {
        return SCHED_ERROR_INVALID_TASK_ID;
    }
    
    if (config->periodMs == 0)
    {
        return SCHED_ERROR_INVALID_PERIOD;
    }
    
    task = &Sched_Tasks[taskId];
    
    if (task->registered)
    {
        return SCHED_ERROR_TASK_EXISTS;
    }
    
    /* Copy configuration */
    task->config = *config;
    
    /* Convert times to ticks */
    task->periodTicks = Sched_MsToTicks(config->periodMs);
    task->offsetTicks = Sched_MsToTicks(config->offsetMs);
    
    /* Ensure period is at least 1 tick */
    if (task->periodTicks == 0)
    {
        task->periodTicks = 1;
    }
    
    /* Initialize runtime state */
    task->nextRunTick = task->offsetTicks;
    task->runCount = 0;
    task->registered = TRUE;
    task->state = config->autoStart ? TASK_STATE_READY : TASK_STATE_DISABLED;
    
#if (SCHED_ENABLE_STATISTICS == 1U)
    task->lastRunTime = 0;
    task->maxRunTime = 0;
    task->totalRunTime = 0;
    task->missedDeadlines = 0;
#endif
    
    return SCHED_OK;
}

Sched_StatusType Sched_RegisterTaskSimple(uint8 taskId, Sched_TaskCallback_t callback, uint32 periodMs)
{
    Sched_TaskConfig_t config = {
        .name = "Task",
        .callback = callback,
        .arg = NULL_PTR,
        .periodMs = periodMs,
        .offsetMs = 0,
        .priority = taskId,
        .autoStart = TRUE
    };
    
    return Sched_RegisterTask(taskId, &config);
}

Sched_StatusType Sched_UnregisterTask(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS)
    {
        return SCHED_ERROR_INVALID_TASK_ID;
    }
    
    if (!Sched_Tasks[taskId].registered)
    {
        return SCHED_ERROR_TASK_NOT_FOUND;
    }
    
    Sched_Tasks[taskId].registered = FALSE;
    Sched_Tasks[taskId].state = TASK_STATE_DISABLED;
    
    return SCHED_OK;
}

/*******************************************************************************
 *                          Scheduler Control                                  *
 *******************************************************************************/

void Sched_Start(void)
{
    if (Sched_State == SCHED_STATE_UNINIT)
    {
        Sched_ReportError(SCHED_ERROR_NOT_INITIALIZED);
        return;
    }
    
    Sched_State = SCHED_STATE_RUNNING;
    Sched_StopRequest = FALSE;
    Sched_TickCount = 0;
    
    /* Start SysTick timer */
    SysTick_Start();
    
    /* Main scheduler loop */
    while (!Sched_StopRequest)
    {
        /* Wait for tick */
        if (Sched_TickFlag)
        {
            Sched_TickFlag = FALSE;
            
#if (SCHED_ENABLE_STATISTICS == 1U)
            Sched_Statistics.tickCount++;
            Sched_LastTickStart = SysTick_GetTickCount();
#endif
            
            /* Call tick hook if registered */
            if (Sched_TickHook != NULL_PTR)
            {
                Sched_TickHook(Sched_TickCount);
            }
            
            /* Execute tasks if not paused */
            if (!Sched_PauseRequest)
            {
                Sched_ExecuteTasks();
            }
            
#if (SCHED_ENABLE_STATISTICS == 1U)
            /* Track tick time */
            uint32 tickTime = SysTick_GetTickCount() - Sched_LastTickStart;
            if (tickTime > Sched_Statistics.maxTickTime)
            {
                Sched_Statistics.maxTickTime = tickTime;
            }
            
            /* Calculate CPU load (simple moving average) */
            uint32 totalTime = Sched_Statistics.busyTime + Sched_Statistics.idleTime;
            if (totalTime > 0)
            {
                Sched_Statistics.cpuLoad = (uint8)((Sched_Statistics.busyTime * 100) / totalTime);
            }
#endif
        }
        else
        {
            /* Idle - no tick pending */
#if (SCHED_ENABLE_STATISTICS == 1U)
            Sched_Statistics.idleTime++;
#endif
            
#if (SCHED_ENABLE_IDLE_HOOK == 1U)
            if (Sched_IdleHook != NULL_PTR)
            {
                Sched_IdleHook();
            }
#endif
            
#if (SCHED_ENABLE_WATCHDOG == 1U)
            /* Refresh watchdog here if enabled */
#endif
        }
    }
    
    /* Scheduler stopped */
    SysTick_Stop();
    Sched_State = SCHED_STATE_STOPPED;
}

Sched_StatusType Sched_Stop(void)
{
    Sched_StopRequest = TRUE;
    return SCHED_OK;
}

Sched_StatusType Sched_Pause(void)
{
    Sched_PauseRequest = TRUE;
    Sched_State = SCHED_STATE_PAUSED;
    return SCHED_OK;
}

Sched_StatusType Sched_Resume(void)
{
    Sched_PauseRequest = FALSE;
    Sched_State = SCHED_STATE_RUNNING;
    return SCHED_OK;
}

/*******************************************************************************
 *                          Task Control                                       *
 *******************************************************************************/

Sched_StatusType Sched_EnableTask(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS)
    {
        return SCHED_ERROR_INVALID_TASK_ID;
    }
    
    if (!Sched_Tasks[taskId].registered)
    {
        return SCHED_ERROR_TASK_NOT_FOUND;
    }
    
    Sched_Tasks[taskId].state = TASK_STATE_READY;
    /* Reset next run to current tick */
    Sched_Tasks[taskId].nextRunTick = Sched_TickCount;
    
    return SCHED_OK;
}

Sched_StatusType Sched_DisableTask(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS)
    {
        return SCHED_ERROR_INVALID_TASK_ID;
    }
    
    if (!Sched_Tasks[taskId].registered)
    {
        return SCHED_ERROR_TASK_NOT_FOUND;
    }
    
    Sched_Tasks[taskId].state = TASK_STATE_DISABLED;
    
    return SCHED_OK;
}

Sched_StatusType Sched_SuspendTask(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS)
    {
        return SCHED_ERROR_INVALID_TASK_ID;
    }
    
    if (!Sched_Tasks[taskId].registered)
    {
        return SCHED_ERROR_TASK_NOT_FOUND;
    }
    
    Sched_Tasks[taskId].state = TASK_STATE_SUSPENDED;
    
    return SCHED_OK;
}

Sched_StatusType Sched_ResumeTask(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS)
    {
        return SCHED_ERROR_INVALID_TASK_ID;
    }
    
    if (!Sched_Tasks[taskId].registered)
    {
        return SCHED_ERROR_TASK_NOT_FOUND;
    }
    
    if (Sched_Tasks[taskId].state == TASK_STATE_SUSPENDED)
    {
        Sched_Tasks[taskId].state = TASK_STATE_READY;
    }
    
    return SCHED_OK;
}

Sched_StatusType Sched_SetTaskPeriod(uint8 taskId, uint32 periodMs)
{
    if (taskId >= SCHED_MAX_TASKS)
    {
        return SCHED_ERROR_INVALID_TASK_ID;
    }
    
    if (!Sched_Tasks[taskId].registered)
    {
        return SCHED_ERROR_TASK_NOT_FOUND;
    }
    
    if (periodMs == 0)
    {
        return SCHED_ERROR_INVALID_PERIOD;
    }
    
    Sched_Tasks[taskId].config.periodMs = periodMs;
    Sched_Tasks[taskId].periodTicks = Sched_MsToTicks(periodMs);
    
    if (Sched_Tasks[taskId].periodTicks == 0)
    {
        Sched_Tasks[taskId].periodTicks = 1;
    }
    
    return SCHED_OK;
}

Sched_TaskStateType Sched_GetTaskState(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS || !Sched_Tasks[taskId].registered)
    {
        return TASK_STATE_DISABLED;
    }
    
    return Sched_Tasks[taskId].state;
}

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

Sched_StateType Sched_GetState(void)
{
    return Sched_State;
}

uint32 Sched_GetTickCount(void)
{
    return Sched_TickCount;
}

uint32 Sched_GetTimeMs(void)
{
    return Sched_TickCount * Sched_TickPeriodMs;
}

boolean Sched_IsRunning(void)
{
    return (Sched_State == SCHED_STATE_RUNNING);
}

/*******************************************************************************
 *                          Statistics Functions                               *
 *******************************************************************************/

#if (SCHED_ENABLE_STATISTICS == 1U)

Sched_StatusType Sched_GetStats(Sched_Stats_t *stats)
{
    if (stats == NULL_PTR)
    {
        return SCHED_ERROR_NULL_PTR;
    }
    
    *stats = Sched_Statistics;
    
    return SCHED_OK;
}

uint32 Sched_GetTaskRunCount(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS || !Sched_Tasks[taskId].registered)
    {
        return 0;
    }
    
    return Sched_Tasks[taskId].runCount;
}

uint32 Sched_GetTaskMaxTime(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS || !Sched_Tasks[taskId].registered)
    {
        return 0;
    }
    
    return Sched_Tasks[taskId].maxRunTime;
}

uint8 Sched_GetCpuLoad(void)
{
    return Sched_Statistics.cpuLoad;
}

void Sched_ResetStats(void)
{
    uint8 i;
    
    Sched_Statistics.tickCount = 0;
    Sched_Statistics.taskSwitches = 0;
    Sched_Statistics.idleTime = 0;
    Sched_Statistics.busyTime = 0;
    Sched_Statistics.cpuLoad = 0;
    Sched_Statistics.maxTickTime = 0;
    Sched_Statistics.missedTicks = 0;
    
    for (i = 0; i < SCHED_MAX_TASKS; i++)
    {
        if (Sched_Tasks[i].registered)
        {
            Sched_Tasks[i].lastRunTime = 0;
            Sched_Tasks[i].maxRunTime = 0;
            Sched_Tasks[i].totalRunTime = 0;
            Sched_Tasks[i].missedDeadlines = 0;
        }
    }
}

#endif /* SCHED_ENABLE_STATISTICS */

/*******************************************************************************
 *                          Hook Functions                                     *
 *******************************************************************************/

void Sched_SetIdleHook(Sched_IdleHook_t hook)
{
    Sched_IdleHook = hook;
}

void Sched_SetErrorHook(Sched_ErrorHook_t hook)
{
    Sched_ErrorHook = hook;
}

void Sched_SetTickHook(Sched_TickHook_t hook)
{
    Sched_TickHook = hook;
}

/*******************************************************************************
 *                          Utility Functions                                  *
 *******************************************************************************/

void Sched_DelayMs(uint32 ms)
{
    uint32 startTick = Sched_TickCount;
    uint32 delayTicks = Sched_MsToTicks(ms);
    
    while ((Sched_TickCount - startTick) < delayTicks)
    {
        /* Busy wait - use sparingly */
    }
}

const char* Sched_GetTaskName(uint8 taskId)
{
    if (taskId >= SCHED_MAX_TASKS || !Sched_Tasks[taskId].registered)
    {
        return "Unknown";
    }
    
    if (Sched_Tasks[taskId].config.name == NULL_PTR)
    {
        return "Unnamed";
    }
    
    return Sched_Tasks[taskId].config.name;
}
