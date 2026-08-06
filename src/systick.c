/******************************************************************************
 *
 * Module: SysTick
 *
 * File Name: systick.c
 *
 * Description: Source file for TM4C123GH6PM SysTick driver
 *              Supports periodic interrupts, busy-wait delays, and scheduler timing
 *
 ******************************************************************************/

#include "systick.h"
#include "systick_private.h"

/*******************************************************************************
 *                          Private Definitions                                *
 *******************************************************************************/

/* PIOSC/4 is a fixed 4MHz clock source, independent of SYSTICK_SYSTEM_CLOCK_HZ
 * (which describes the system clock, a different clock source entirely) */
#define SYSTICK_PIOSC_DIV4_CLOCK_HZ     (4000000UL)

/* Effective clock source for SysTick_Init/SysTick_InitUs, kept in sync with
 * SYSTICK_USE_SYSTEM_CLOCK (systick_cfg.h) so that cfg switch actually has
 * an effect on both the CLK_SRC bit and the reload math together */
#define SYSTICK_DEFAULT_CLOCK_SOURCE \
    ((SYSTICK_USE_SYSTEM_CLOCK == 1U) ? SYSTICK_CLOCK_SYSTEM : SYSTICK_CLOCK_PIOSC_DIV4)

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

/* Current state of SysTick */
static SysTick_StateType SysTick_State = SYSTICK_STATE_UNINIT;

/* Callback function pointer */
static volatile SysTick_CallbackType SysTick_Callback = NULL_PTR;

/* Current reload value (for elapsed calculation) */
static uint32 SysTick_ReloadValue = 0;

/* Tick counter (incremented on each interrupt) */
static volatile uint32 SysTick_TickCount = 0;

/*******************************************************************************
 *                          Private Functions                                  *
 *******************************************************************************/

/**
 * @brief  Calculate reload value from microseconds
 * @param  timeUs: Time in microseconds
 * @param  clockSource: Clock source that will actually be selected in STCTRL
 * @param  reloadValue: Pointer to store calculated reload value
 * @return SYSTICK_OK if valid, error code otherwise
 */
static SysTick_StatusType SysTick_CalculateReload(uint32 timeUs, SysTick_ClockSourceType clockSource, uint32 *reloadValue)
{
    uint32 ticks;
    uint32 ticksPerUs;
    uint32 maxTimeUs;

    if (reloadValue == NULL_PTR)
    {
        return SYSTICK_ERROR_NULL_PTR;
    }

    if (timeUs == 0)
    {
        return SYSTICK_ERROR_INVALID_TIME;
    }

    /* Ticks per microsecond for whichever clock source will actually be
     * written to CLK_SRC - must match, or the period is wrong by 4x */
    ticksPerUs = (clockSource == SYSTICK_CLOCK_PIOSC_DIV4) ?
                 (SYSTICK_PIOSC_DIV4_CLOCK_HZ / 1000000UL) :
                 SYSTICK_TICKS_PER_US;

    /* Reject any timeUs whose tick count would not fit in the 24-bit reload
     * range BEFORE multiplying, so the multiplication itself can never
     * overflow uint32 (computing this after multiplying is too late - the
     * wraparound has already happened and may land back in-range) */
    maxTimeUs = (SYSTICK_MAX_RELOAD_VALUE + 1UL) / ticksPerUs;
    if (timeUs > maxTimeUs)
    {
        return SYSTICK_ERROR_OVERFLOW;
    }

    ticks = ticksPerUs * timeUs;

    /* Reload value is ticks - 1 (counter counts from RELOAD to 0) */
    if (ticks == 0)
    {
        return SYSTICK_ERROR_INVALID_TIME;
    }

    ticks = ticks - 1;

    /* Check if within 24-bit limit (redundant with the guard above, kept as a safety net) */
    if (!SYSTICK_IS_VALID_RELOAD(ticks))
    {
        return SYSTICK_ERROR_OVERFLOW;
    }

    *reloadValue = ticks;
    return SYSTICK_OK;
}

/**
 * @brief  Configure SysTick registers
 * @param  reloadValue: Reload register value
 * @param  enableInterrupt: TRUE to enable interrupt
 */
static void SysTick_Configure(uint32 reloadValue, boolean enableInterrupt)
{
    /* Disable SysTick during configuration */
    SYSTICK_CTRL_REG = 0;
    
    /* Set reload value */
    SYSTICK_RELOAD_REG = reloadValue;
    
    /* Clear current value */
    SYSTICK_CURRENT_REG = 0;
    
    /* Store reload value for elapsed calculation */
    SysTick_ReloadValue = reloadValue;
    
    /* Configure control register - clock source follows SYSTICK_USE_SYSTEM_CLOCK */
    uint32 ctrl = (SYSTICK_DEFAULT_CLOCK_SOURCE == SYSTICK_CLOCK_SYSTEM) ? SYSTICK_CTRL_CLKSRC_MASK : 0UL;

    if (enableInterrupt)
    {
        ctrl |= SYSTICK_CTRL_INTEN_MASK;
    }
    
    ctrl |= SYSTICK_CTRL_ENABLE_MASK;  /* Enable timer */
    
    SYSTICK_CTRL_REG = ctrl;
}

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

SysTick_StatusType SysTick_Init(uint32 timeMs)
{
    /* Convert ms to us and call InitUs */
    if (timeMs > (0xFFFFFFFFUL / 1000UL))
    {
        return SYSTICK_ERROR_OVERFLOW;
    }
    
    return SysTick_InitUs(timeMs * 1000UL);
}

SysTick_StatusType SysTick_InitUs(uint32 timeUs)
{
    uint32 reloadValue;
    SysTick_StatusType status;
    
    /* Calculate reload value (must match the clock source SysTick_Configure selects) */
    status = SysTick_CalculateReload(timeUs, SYSTICK_DEFAULT_CLOCK_SOURCE, &reloadValue);
    if (status != SYSTICK_OK)
    {
        return status;
    }

    /* Reset tick counter */
    SysTick_TickCount = 0;

    /* Configure and start SysTick with interrupt enabled */
    SysTick_Configure(reloadValue, TRUE);
    
    SysTick_State = SYSTICK_STATE_RUNNING;
    
    return SYSTICK_OK;
}

SysTick_StatusType SysTick_InitWithConfig(const SysTick_ConfigType *config)
{
    uint32 reloadValue;
    SysTick_StatusType status;
    
    if (config == NULL_PTR)
    {
        return SYSTICK_ERROR_NULL_PTR;
    }
    
    /* Calculate reload value for whichever clock source the caller selected */
    status = SysTick_CalculateReload(config->periodUs, config->clockSource, &reloadValue);
    if (status != SYSTICK_OK)
    {
        return status;
    }

    /* Set callback if provided */
    SysTick_Callback = config->callback;
    
    /* Reset tick counter */
    SysTick_TickCount = 0;
    
    /* Disable SysTick during configuration */
    SYSTICK_CTRL_REG = 0;
    
    /* Set reload value */
    SYSTICK_RELOAD_REG = reloadValue;
    SysTick_ReloadValue = reloadValue;
    
    /* Clear current value */
    SYSTICK_CURRENT_REG = 0;
    
    /* Build control register value */
    uint32 ctrl = 0;
    
    if (config->clockSource == SYSTICK_CLOCK_SYSTEM)
    {
        ctrl |= SYSTICK_CTRL_CLKSRC_MASK;
    }
    
    if (config->mode == SYSTICK_MODE_INTERRUPT)
    {
        ctrl |= SYSTICK_CTRL_INTEN_MASK;
    }
    
    ctrl |= SYSTICK_CTRL_ENABLE_MASK;
    
    SYSTICK_CTRL_REG = ctrl;
    
    SysTick_State = SYSTICK_STATE_RUNNING;
    
    return SYSTICK_OK;
}

SysTick_StatusType SysTick_DeInit(void)
{
    /* Disable SysTick */
    SYSTICK_CTRL_REG = 0;
    
    /* Clear registers */
    SYSTICK_RELOAD_REG = 0;
    SYSTICK_CURRENT_REG = 0;
    
    /* Clear state */
    SysTick_State = SYSTICK_STATE_UNINIT;
    SysTick_Callback = NULL_PTR;
    SysTick_ReloadValue = 0;
    SysTick_TickCount = 0;
    
    return SYSTICK_OK;
}

/*******************************************************************************
 *                          Control Functions                                  *
 *******************************************************************************/

SysTick_StatusType SysTick_Start(void)
{
    if (SysTick_State == SYSTICK_STATE_UNINIT)
    {
        return SYSTICK_ERROR_NOT_INITIALIZED;
    }
    
    SYSTICK_CTRL_REG |= SYSTICK_CTRL_ENABLE_MASK;
    SysTick_State = SYSTICK_STATE_RUNNING;
    
    return SYSTICK_OK;
}

SysTick_StatusType SysTick_Stop(void)
{
    SYSTICK_CTRL_REG &= ~SYSTICK_CTRL_ENABLE_MASK;
    
    if (SysTick_State != SYSTICK_STATE_UNINIT)
    {
        SysTick_State = SYSTICK_STATE_IDLE;
    }
    
    return SYSTICK_OK;
}

SysTick_StatusType SysTick_Reset(void)
{
    /* Writing any value to CURRENT clears it and clears COUNT flag */
    SYSTICK_CURRENT_REG = 0;
    
    return SYSTICK_OK;
}

/*******************************************************************************
 *                          Callback Functions                                 *
 *******************************************************************************/

SysTick_StatusType SysTick_SetCallback(SysTick_CallbackType callback)
{
    /* Disable interrupts during modification for atomicity */
    SYSTICK_DISABLE_INTERRUPTS();
    
    SysTick_Callback = callback;
    
    SYSTICK_ENABLE_INTERRUPTS();
    
    return SYSTICK_OK;
}

/*******************************************************************************
 *                          Delay Functions (Blocking)                         *
 *******************************************************************************/

SysTick_StatusType SysTick_DelayMs(uint32 timeMs)
{
    uint32 remainingMs = timeMs;
    uint32 delayChunk;
    SysTick_StatusType status;
    
    if (timeMs == 0)
    {
        return SYSTICK_ERROR_INVALID_TIME;
    }
    
    /* Handle delays longer than max by chunking */
    while (remainingMs > 0)
    {
        /* Calculate chunk size (max is SYSTICK_MAX_DELAY_MS) */
        if (remainingMs > SYSTICK_MAX_DELAY_MS)
        {
            delayChunk = SYSTICK_MAX_DELAY_MS;
        }
        else
        {
            delayChunk = remainingMs;
        }
        
        /* Perform delay for this chunk */
        status = SysTick_DelayUs(delayChunk * 1000UL);
        if (status != SYSTICK_OK)
        {
            return status;
        }
        
        remainingMs -= delayChunk;
    }
    
    return SYSTICK_OK;
}

SysTick_StatusType SysTick_DelayUs(uint32 timeUs)
{
    uint32 reloadValue;
    SysTick_StatusType status;
    uint32 ctrlBackup;
    uint32 reloadBackup;
    SysTick_StateType stateBackup;

    /* Calculate reload value (this one-shot delay always runs on system clock) */
    status = SysTick_CalculateReload(timeUs, SYSTICK_CLOCK_SYSTEM, &reloadValue);
    if (status != SYSTICK_OK)
    {
        return status;
    }

    /* Save current SysTick configuration so it can be restored after the delay */
    ctrlBackup = SYSTICK_CTRL_REG;
    reloadBackup = SysTick_ReloadValue;
    stateBackup = SysTick_State;

    /* Disable SysTick and configure for polling mode */
    SYSTICK_CTRL_REG = 0;
    SYSTICK_RELOAD_REG = reloadValue;
    SYSTICK_CURRENT_REG = 0;

    /* Enable with system clock, no interrupt */
    SYSTICK_CTRL_REG = SYSTICK_CTRL_CLKSRC_MASK | SYSTICK_CTRL_ENABLE_MASK;

    /* Wait for COUNT flag */
    while ((SYSTICK_CTRL_REG & SYSTICK_CTRL_COUNT_MASK) == 0)
    {
        /* Busy wait */
    }

    /* Restore SysTick to whatever it was doing before this delay (running and
     * interrupting if it was, idle if it was) instead of leaving it disabled */
    SYSTICK_CTRL_REG = 0;
    SYSTICK_RELOAD_REG = reloadBackup;
    SysTick_ReloadValue = reloadBackup;
    SYSTICK_CURRENT_REG = 0;
    SYSTICK_CTRL_REG = ctrlBackup;
    SysTick_State = stateBackup;

    return SYSTICK_OK;
}

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

SysTick_StateType SysTick_GetState(void)
{
    return SysTick_State;
}

uint32 SysTick_GetCurrentValue(void)
{
    return SYSTICK_CURRENT_REG;
}

uint32 SysTick_GetElapsedTicks(void)
{
    /* Counter counts down, so elapsed = reload - current */
    return SysTick_ReloadValue - SYSTICK_CURRENT_REG;
}

boolean SysTick_HasElapsed(void)
{
    /* Reading CTRL clears the COUNT flag */
    return ((SYSTICK_CTRL_REG & SYSTICK_CTRL_COUNT_MASK) != 0) ? TRUE : FALSE;
}

uint32 SysTick_GetTickCount(void)
{
    return SysTick_TickCount;
}

/*******************************************************************************
 *                          Period Configuration                               *
 *******************************************************************************/

SysTick_StatusType SysTick_SetPeriodMs(uint32 timeMs)
{
    if (timeMs > (0xFFFFFFFFUL / 1000UL))
    {
        return SYSTICK_ERROR_OVERFLOW;
    }
    
    return SysTick_SetPeriodUs(timeMs * 1000UL);
}

SysTick_StatusType SysTick_SetPeriodUs(uint32 timeUs)
{
    uint32 reloadValue;
    SysTick_StatusType status;
    uint32 ctrlBackup;
    SysTick_ClockSourceType currentClockSource;

    /* Save control register state (read first so its CLK_SRC bit can be used below) */
    ctrlBackup = SYSTICK_CTRL_REG;

    /* Compute the new reload for whichever clock source is currently selected,
     * instead of silently re-assuming system clock */
    currentClockSource = (ctrlBackup & SYSTICK_CTRL_CLKSRC_MASK) ? SYSTICK_CLOCK_SYSTEM : SYSTICK_CLOCK_PIOSC_DIV4;

    /* Calculate new reload value */
    status = SysTick_CalculateReload(timeUs, currentClockSource, &reloadValue);
    if (status != SYSTICK_OK)
    {
        return status;
    }

    /* Disable SysTick */
    SYSTICK_CTRL_REG = 0;
    
    /* Set new reload value */
    SYSTICK_RELOAD_REG = reloadValue;
    SysTick_ReloadValue = reloadValue;
    
    /* Clear current value */
    SYSTICK_CURRENT_REG = 0;
    
    /* Restore control register */
    SYSTICK_CTRL_REG = ctrlBackup;
    
    return SYSTICK_OK;
}

/*******************************************************************************
 *                          Interrupt Handler                                  *
 *******************************************************************************/

void SysTick_Handler(void)
{
    /* Increment tick counter */
    SysTick_TickCount++;
    
    /* Call user callback if registered */
    if (SysTick_Callback != NULL_PTR)
    {
        SysTick_Callback();
    }
}
