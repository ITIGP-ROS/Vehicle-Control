/******************************************************************************
 *
 * Module: Watchdog Timer (WDT)
 *
 * File Name: wdt.c
 *
 * Description: Implementation of TM4C123GH6PM Hardware Watchdog Timer driver
 *              NO standard library dependencies
 *
 ******************************************************************************/

#include "wdt.h"

/*******************************************************************************
 *                    Compile-time checks for W24-5 / W24-6                    *
 *
 * The build is gnu90 (no -std=, GCC 4.8), so _Static_assert is unavailable and
 * the #if preprocessor cannot evaluate the casts inside the macros. The classic
 * negative-array-size idiom works in C90 and emits no code.
 *
 * These pin the exact values REVIEW 24 measured, so a future edit to the
 * conversion macros or to WDT_CLOCK_FREQ_HZ that reintroduces the overflow
 * fails the BUILD instead of silently lying at runtime again.
 *******************************************************************************/

typedef char WDT_AssertMsToLoad2000[(WDT_MS_TO_LOAD(2000u) == 32000000u) ? 1 : -1];
/* The regression itself: this returned 120 before the uint64 widening. */
typedef char WDT_AssertLoadToMs32M[(WDT_LOAD_TO_MS(32000000u) == 2000u) ? 1 : -1];
/* Round-trip through both macros at the configured default. */
typedef char WDT_AssertRoundTrip[(WDT_LOAD_TO_MS(WDT_MS_TO_LOAD(2000u)) == 2000u) ? 1 : -1];
/* WDT_SEC_TO_LOAD has no call site, so only an assert can catch a bad edit. */
typedef char WDT_AssertSecToLoad2[(WDT_SEC_TO_LOAD(2u) == 32000000u) ? 1 : -1];
/* The range bound is exact: the largest accepted timeout still fits WDTLOAD... */
typedef char WDT_AssertMaxFits[(WDT_MS_TO_LOAD(WDT_MAX_TIMEOUT_MS) <= WDT_MAX_LOAD_VALUE) ? 1 : -1];
/* ...and it is the LARGEST such value (one ms more overflows the register). */
typedef char WDT_AssertMaxIsTight[((((uint64)(WDT_MAX_TIMEOUT_MS) + 1u) * (uint64)WDT_CLOCK_FREQ_HZ / 1000u)
                                   > (uint64)WDT_MAX_LOAD_VALUE) ? 1 : -1];

/*******************************************************************************
 *                          Private Data Types                                 *
 *******************************************************************************/

/**
 * @brief Watchdog control block (stores runtime state)
 */
typedef struct {
    WDT_StateType state;                    /* Current state */
    WDT_ConfigType config;                  /* Configuration */
    uint32 baseAddress;                     /* Base address of registers */
    boolean initialized;                    /* Initialization flag */
} WDT_ControlBlockType;

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

static WDT_ControlBlockType g_WDT_ControlBlocks[WDT_ID_MAX];

/*******************************************************************************
 *                          Private Helper Functions                           *
 *******************************************************************************/

/**
 * @brief  Get base address for a watchdog timer
 * @param  wdtId: Watchdog timer ID
 * @return Base address
 */
static uint32 WDT_GetBaseAddress(WDT_IdType wdtId)
{
    return (wdtId == WDT_ID_0) ? WDT0_BASE_ADDRESS : WDT1_BASE_ADDRESS;
}

/**
 * @brief  Enable clock to watchdog peripheral
 * @param  wdtId: Watchdog timer ID
 */
static void WDT_EnableClock(WDT_IdType wdtId)
{
    if (wdtId == WDT_ID_0) {
        SYSCTL_RCGCWD_REG |= SYSCTL_RCGCWD_R0;
        /* Wait for peripheral to be ready */
        while ((SYSCTL_PRWD_REG & SYSCTL_PRWD_R0) == 0);
    } else {
        SYSCTL_RCGCWD_REG |= SYSCTL_RCGCWD_R1;
        /* Wait for peripheral to be ready */
        while ((SYSCTL_PRWD_REG & SYSCTL_PRWD_R1) == 0);
    }
}

/**
 * @brief  Disable clock to watchdog peripheral
 * @param  wdtId: Watchdog timer ID
 */
static void WDT_DisableClock(WDT_IdType wdtId)
{
    if (wdtId == WDT_ID_0) {
        SYSCTL_RCGCWD_REG &= ~SYSCTL_RCGCWD_R0;
    } else {
        SYSCTL_RCGCWD_REG &= ~SYSCTL_RCGCWD_R1;
    }
}

/**
 * @brief  Read watchdog register
 * @param  baseAddr: Base address
 * @param  offset: Register offset
 * @return Register value
 */
static uint32 WDT_ReadReg(uint32 baseAddr, uint32 offset)
{
    return *((volatile uint32 *)(baseAddr + offset));
}

/**
 * @brief  Write watchdog register
 * @param  baseAddr: Base address
 * @param  offset: Register offset
 * @param  value: Value to write
 */
static void WDT_WriteReg(uint32 baseAddr, uint32 offset, uint32 value)
{
    *((volatile uint32 *)(baseAddr + offset)) = value;
}

/*******************************************************************************
 *                          Public Function Implementation                     *
 *******************************************************************************/

/**
 * @brief  Initialize the watchdog timer module
 */
WDT_StatusType WDT_Init(WDT_IdType wdtId, const WDT_ConfigType *config)
{
    uint32 baseAddr;
    uint32 loadValue;
    uint32 ctlValue;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (config == NULL) {
        return WDT_ERROR_NULL_PTR;
    }
    
    /* W24-6: reject out-of-range as well as zero. Above WDT_MAX_TIMEOUT_MS the
     * load no longer fits WDTLOAD, and the old code let it wrap silently to a
     * tiny value = a near-instant reset. Refusing the config is the correct
     * failure direction for a watchdog. */
    if ((config->timeoutMs == 0) || (config->timeoutMs > WDT_MAX_TIMEOUT_MS)) {
        return WDT_ERROR_INVALID_TIMEOUT;
    }
    
    /* Enable clock to watchdog peripheral */
    WDT_EnableClock(wdtId);
    
    /* Store configuration */
    g_WDT_ControlBlocks[wdtId].config = *config;
    g_WDT_ControlBlocks[wdtId].baseAddress = WDT_GetBaseAddress(wdtId);
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Unlock watchdog registers */
    WDT_WriteReg(baseAddr, WDT_LOCK_OFFSET, WDT_LOCK_UNLOCK_KEY);
    
    /* Disable watchdog during configuration */
    WDT_WriteReg(baseAddr, WDT_CTL_OFFSET, 0);
    
    /* Calculate and set load value */
    loadValue = WDT_MS_TO_LOAD(config->timeoutMs);
    if (loadValue < WDT_MIN_LOAD_VALUE) {
        loadValue = WDT_MIN_LOAD_VALUE;
    }
    WDT_WriteReg(baseAddr, WDT_LOAD_OFFSET, loadValue);
    
    /* Configure control register */
    ctlValue = 0;
    
    if (config->mode == WDT_MODE_RESET) {
        /* Enable reset on timeout */
        ctlValue |= WDT_CTL_RESEN;
    } else {
        /* Enable interrupt on timeout */
        ctlValue |= WDT_CTL_INTEN;
        
        /* Configure interrupt type */
        if (config->intType == WDT_INT_NMI) {
            ctlValue |= WDT_CTL_INTTYPE;
        }
    }
    
    WDT_WriteReg(baseAddr, WDT_CTL_OFFSET, ctlValue);
    
    /* Configure test register (stall on debug) */
    if (config->enableStallDebug) {
        WDT_WriteReg(baseAddr, WDT_TEST_OFFSET, WDT_TEST_STALL);
    } else {
        WDT_WriteReg(baseAddr, WDT_TEST_OFFSET, 0);
    }
    
    /* Clear any pending interrupts */
    WDT_WriteReg(baseAddr, WDT_ICR_OFFSET, WDT_ICR_CLEAR);
    
    /* Mark as initialized but not running */
    g_WDT_ControlBlocks[wdtId].state = WDT_STATE_STOPPED;
    g_WDT_ControlBlocks[wdtId].initialized = TRUE;
    
    return WDT_OK;
}

/**
 * @brief  Deinitialize the watchdog timer
 */
WDT_StatusType WDT_DeInit(WDT_IdType wdtId)
{
    uint32 baseAddr;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Unlock and disable watchdog */
    WDT_WriteReg(baseAddr, WDT_LOCK_OFFSET, WDT_LOCK_UNLOCK_KEY);
    WDT_WriteReg(baseAddr, WDT_CTL_OFFSET, 0);
    
    /* Disable clock */
    WDT_DisableClock(wdtId);
    
    /* Clear state */
    g_WDT_ControlBlocks[wdtId].initialized = FALSE;
    g_WDT_ControlBlocks[wdtId].state = WDT_STATE_UNINIT;
    
    return WDT_OK;
}

/**
 * @brief  Start the watchdog timer
 */
WDT_StatusType WDT_Start(WDT_IdType wdtId)
{
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }

    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }

    /* Watchdog starts counting immediately when INTEN or RESEN is set */
    /* It's already configured in Init, so it's already running */
    /* This function just updates our state tracking */

    g_WDT_ControlBlocks[wdtId].state = WDT_STATE_RUNNING;
    
    return WDT_OK;
}

/**
 * @brief  Stop the watchdog timer
 */
WDT_StatusType WDT_Stop(WDT_IdType wdtId)
{
    uint32 baseAddr;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Check if locked */
    if (WDT_IsLocked(wdtId)) {
        return WDT_ERROR_LOCKED;
    }
    
    /* Unlock registers */
    WDT_WriteReg(baseAddr, WDT_LOCK_OFFSET, WDT_LOCK_UNLOCK_KEY);
    
    /* Disable watchdog by clearing control register */
    WDT_WriteReg(baseAddr, WDT_CTL_OFFSET, 0);
    
    g_WDT_ControlBlocks[wdtId].state = WDT_STATE_STOPPED;
    
    return WDT_OK;
}

/**
 * @brief  Kick (feed) the watchdog timer
 */
WDT_StatusType WDT_Kick(WDT_IdType wdtId)
{
    uint32 baseAddr;
    uint32 loadValue;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Read current load value and write it back to reload counter */
    /* This is the "kick" operation */
    loadValue = WDT_ReadReg(baseAddr, WDT_LOAD_OFFSET);
    
    /* Writing to WDTLOAD reloads the counter */
    WDT_WriteReg(baseAddr, WDT_LOAD_OFFSET, loadValue);
    
    return WDT_OK;
}

/**
 * @brief  Lock the watchdog configuration
 */
WDT_StatusType WDT_Lock(WDT_IdType wdtId)
{
    uint32 baseAddr;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Write any value other than unlock key to lock */
    WDT_WriteReg(baseAddr, WDT_LOCK_OFFSET, WDT_LOCK_LOCKED);
    
    return WDT_OK;
}

/**
 * @brief  Unlock the watchdog configuration
 */
WDT_StatusType WDT_Unlock(WDT_IdType wdtId)
{
    uint32 baseAddr;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Write unlock key */
    WDT_WriteReg(baseAddr, WDT_LOCK_OFFSET, WDT_LOCK_UNLOCK_KEY);
    
    return WDT_OK;
}

/**
 * @brief  Check if watchdog is locked
 */
boolean WDT_IsLocked(WDT_IdType wdtId)
{
    uint32 baseAddr;
    uint32 lockValue;
    
    if (wdtId >= WDT_ID_MAX || !g_WDT_ControlBlocks[wdtId].initialized) {
        return TRUE;  /* Consider invalid watchdog as locked */
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    lockValue = WDT_ReadReg(baseAddr, WDT_LOCK_OFFSET);
    
    return (lockValue == WDT_LOCK_LOCKED) ? TRUE : FALSE;
}

/**
 * @brief  Change watchdog timeout period
 */
WDT_StatusType WDT_SetTimeout(WDT_IdType wdtId, uint32 timeoutMs)
{
    uint32 baseAddr;
    uint32 loadValue;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }
    
    /* W24-6: see WDT_Init - same bound, same reason. */
    if ((timeoutMs == 0) || (timeoutMs > WDT_MAX_TIMEOUT_MS)) {
        return WDT_ERROR_INVALID_TIMEOUT;
    }
    
    /* Check if locked */
    if (WDT_IsLocked(wdtId)) {
        return WDT_ERROR_LOCKED;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Unlock registers */
    WDT_WriteReg(baseAddr, WDT_LOCK_OFFSET, WDT_LOCK_UNLOCK_KEY);
    
    /* Calculate and set new load value */
    loadValue = WDT_MS_TO_LOAD(timeoutMs);
    if (loadValue < WDT_MIN_LOAD_VALUE) {
        loadValue = WDT_MIN_LOAD_VALUE;
    }
    WDT_WriteReg(baseAddr, WDT_LOAD_OFFSET, loadValue);
    
    /* Update stored configuration */
    g_WDT_ControlBlocks[wdtId].config.timeoutMs = timeoutMs;
    
    return WDT_OK;
}

/**
 * @brief  Get current watchdog state
 */
WDT_StateType WDT_GetState(WDT_IdType wdtId)
{
    if (wdtId >= WDT_ID_MAX) {
        return WDT_STATE_UNINIT;
    }
    
    return g_WDT_ControlBlocks[wdtId].state;
}

/**
 * @brief  Get current counter value
 */
WDT_StatusType WDT_GetCurrentValue(WDT_IdType wdtId, uint32 *value)
{
    uint32 baseAddr;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (value == NULL) {
        return WDT_ERROR_NULL_PTR;
    }
    
    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Read current value register */
    *value = WDT_ReadReg(baseAddr, WDT_VALUE_OFFSET);
    
    return WDT_OK;
}

/**
 * @brief  Get time remaining until timeout
 */
WDT_StatusType WDT_GetTimeRemaining(WDT_IdType wdtId, uint32 *timeRemainingMs)
{
    uint32 currentValue;
    WDT_StatusType status;
    
    /* Validate inputs */
    if (timeRemainingMs == NULL) {
        return WDT_ERROR_NULL_PTR;
    }
    
    /* Get current counter value */
    status = WDT_GetCurrentValue(wdtId, &currentValue);
    if (status != WDT_OK) {
        return status;
    }
    
    /* Convert to milliseconds */
    *timeRemainingMs = WDT_LOAD_TO_MS(currentValue);
    
    return WDT_OK;
}

/**
 * @brief  Check if watchdog interrupt is pending
 */
boolean WDT_IsInterruptPending(WDT_IdType wdtId)
{
    uint32 baseAddr;
    uint32 misValue;
    
    if (wdtId >= WDT_ID_MAX || !g_WDT_ControlBlocks[wdtId].initialized) {
        return FALSE;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Read masked interrupt status */
    misValue = WDT_ReadReg(baseAddr, WDT_MIS_OFFSET);
    
    return (misValue != 0) ? TRUE : FALSE;
}

/**
 * @brief  Clear watchdog interrupt
 */
WDT_StatusType WDT_ClearInterrupt(WDT_IdType wdtId)
{
    uint32 baseAddr;
    
    /* Validate inputs */
    if (wdtId >= WDT_ID_MAX) {
        return WDT_ERROR_INVALID_ID;
    }
    
    if (!g_WDT_ControlBlocks[wdtId].initialized) {
        return WDT_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = g_WDT_ControlBlocks[wdtId].baseAddress;
    
    /* Write to interrupt clear register */
    WDT_WriteReg(baseAddr, WDT_ICR_OFFSET, WDT_ICR_CLEAR);
    
    return WDT_OK;
}

/**
 * @brief  Watchdog 0 interrupt handler
 */
void WDT0_IntHandler(void)
{
    /* Clear interrupt */
    WDT_ClearInterrupt(WDT_ID_0);
    
    /* Call user callback if configured */
    if (g_WDT_ControlBlocks[WDT_ID_0].config.callback != NULL) {
        g_WDT_ControlBlocks[WDT_ID_0].config.callback(WDT_ID_0);
    }
}

/**
 * @brief  Watchdog 1 interrupt handler
 */
void WDT1_IntHandler(void)
{
    /* Clear interrupt */
    WDT_ClearInterrupt(WDT_ID_1);
    
    /* Call user callback if configured */
    if (g_WDT_ControlBlocks[WDT_ID_1].config.callback != NULL) {
        g_WDT_ControlBlocks[WDT_ID_1].config.callback(WDT_ID_1);
    }
}
