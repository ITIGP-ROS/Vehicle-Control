/******************************************************************************
 * can_tx_queue.c - the single CAN transmit path. See can_tx_queue.h for why.
 *
 * B6, 2026-08-06.
 *****************************************************************************/

#include "can_tx_queue.h"

#ifdef USE_FREERTOS

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "app_priorities.h"

/*----------------------------------------------------------------------------
 * SIZING
 *
 * DEPTH = 8. The worst simultaneous burst is 5 - 0x110, 0x130, 0x200, 0x210 and
 * the 0x7A1 ping all becoming due in the same millisecond. The slot residues in
 * main.c currently prevent that, but they are pacing only from B6 on and B7
 * retires them, so the queue must survive the burst on its own. Draining 5
 * costs 5 x 222 us = 1.11 ms, during which at most one further frame falls due
 * (the fastest producer is 100 Hz), so 6 is the true high-water. 8 leaves 33 %
 * headroom for 128 bytes of RAM. CanTxQueue_GetPeakDepth() reports the real
 * figure so this can be tightened from evidence rather than argument.
 *--------------------------------------------------------------------------*/
#define CANTX_QUEUE_DEPTH          (8U)
/* Trimmed 2026-08-06: deep-path mission peak 52 words; policy floor 128.
 * See the stack-trim banner in include/app_cfg.h. */
#define CANTX_TASK_STACK_WORDS     (128U)

/* Medium-high (plan section 3). ABOVE the telemetry producers so a posted frame
 * drains promptly and the queue stays shallow, but BELOW tVelocity (4) and
 * tSafety (5) so draining telemetry can never delay a control update or the
 * failsafe. Above tSuperLoop (2) so a post does not wait for the loop's 1 ms
 * yield before reaching the wire. */
#define CANTX_TASK_PRIORITY        APP_PRIO_CAN_TX

/* Escape hatch, NOT a pacing value: ~45x the 222 us frame time, so it cannot
 * fire on a healthy bus. It exists solely for the "nothing ACKs, so TX never
 * completes" case that would otherwise park tCanTx forever - see
 * Can_WaitTxComplete() in can.h. */
#define CANTX_COMPLETE_TIMEOUT_MS  (10U)

typedef struct
{
    uint32 id;
    uint8  data[8];
    uint8  dlc;
} CanTxMsgType;

static StaticQueue_t canTxQueueBuf;
static uint8         canTxQueueStorage[CANTX_QUEUE_DEPTH * sizeof(CanTxMsgType)];
static QueueHandle_t canTxQueue = NULL;

static StaticTask_t  canTxTcb;
static StackType_t   canTxStack[CANTX_TASK_STACK_WORDS];

/* A4-3 counters + observability. All plain uint32 => single aligned word, so
 * loads and stores are atomic on Cortex-M4 and no guard is needed even though
 * tCanTx writes them while tSuperLoop reads them for the heartbeat. */
static volatile uint32 canTx_queueFullDrops = 0U;
static volatile uint32 canTx_txFailCount    = 0U;
static volatile uint32 canTx_txTimeoutCount = 0U;
static volatile uint32 canTx_txCount        = 0U;
static volatile uint32 canTx_peakDepth      = 0U;
static volatile uint32 canTx_stackHwm       = 0U;

/*----------------------------------------------------------------------------
 * tCanTx - the ONLY caller of Can_Transmit in the image.
 *
 * Blocks on the queue with portMAX_DELAY: correct for a purely event-driven
 * task, and it costs zero CPU while idle. After each accepted frame it blocks
 * again on the ISR's TX-complete edge, so the ~222 us of wire time is spent
 * ASLEEP rather than spinning on CAN_ERROR_TX_BUSY - which is the whole reason
 * the semaphore exists.
 *--------------------------------------------------------------------------*/
static void CanTxQueue_Task(void *pvParameters)
{
    CanTxMsgType msg;

    (void)pvParameters;

    for (;;)
    {
        if (xQueueReceive(canTxQueue, &msg, portMAX_DELAY) != pdTRUE)
        {
            continue;                   /* unreachable with portMAX_DELAY */
        }

        /* ⚠️ Discard a stale edge before every frame. A frame that completes
         * AFTER its waiter timed out leaves the semaphore signalled; without
         * this, the next frame's wait would be satisfied instantly by the
         * PREVIOUS frame's completion and tCanTx would believe the object was
         * free while a transmit was still in flight. */
        Can_TxCompleteClear();

        if (Can_Transmit(msg.id, msg.data, msg.dlc) == CAN_OK)
        {
            if (canTx_txCount < 0xFFFFFFFFUL) { canTx_txCount++; }

            /* THIS is what serialises transmits from B6 on - not slot
             * arithmetic. Sleep until the hardware says the object is free. */
            if (Can_WaitTxComplete(CANTX_COMPLETE_TIMEOUT_MS) == FALSE)
            {
                if (canTx_txTimeoutCount < 0xFFFFFFFFUL) { canTx_txTimeoutCount++; }
            }
        }
        else
        {
            /* A4-3: previously a silent (void). Can_Transmit only refuses once
             * past the C6-2 in-flight limit, so reaching here means the bus is
             * genuinely unhappy, not merely busy. */
            if (canTx_txFailCount < 0xFFFFFFFFUL) { canTx_txFailCount++; }
        }

        canTx_stackHwm = (uint32)uxTaskGetStackHighWaterMark(NULL);
    }
}

Std_ReturnType CanTxQueue_Init(void)
{
    canTxQueue = xQueueCreateStatic(CANTX_QUEUE_DEPTH,
                                    sizeof(CanTxMsgType),
                                    canTxQueueStorage,
                                    &canTxQueueBuf);
    if (canTxQueue == NULL)
    {
        return E_NOT_OK;
    }

    /* Create the ISR's TX-complete semaphore before the task that waits on it. */
    Can_TxCompleteSignalInit();

    if (xTaskCreateStatic(CanTxQueue_Task,
                          "CANTX",
                          CANTX_TASK_STACK_WORDS,
                          NULL,
                          CANTX_TASK_PRIORITY,
                          canTxStack,
                          &canTxTcb) == NULL)
    {
        return E_NOT_OK;
    }

    return E_OK;
}

Can_StatusType CanTxQueue_Post(uint32 id, const uint8 *data, uint8 dlc)
{
    CanTxMsgType msg;
    uint8        i;
    UBaseType_t  waiting;

    if ((data == NULL_PTR) || (dlc > 8U))
    {
        return CAN_ERROR_INVALID_DLC;
    }
    if (canTxQueue == NULL)
    {
        return CAN_ERROR_TX_BUSY;       /* posted before Init - nothing to do */
    }

    msg.id  = id;
    msg.dlc = dlc;
    for (i = 0U; i < 8U; i++) { msg.data[i] = (i < dlc) ? data[i] : 0U; }

    /* ⚠️ ZERO TIMEOUT, ALWAYS. Never block a producer on a full queue: from
     * B9/B10 the producers include tVelocity and tSafety, and no telemetry
     * frame is worth delaying a control update or a failsafe. Drop and count. */
    if (xQueueSend(canTxQueue, &msg, 0) != pdTRUE)
    {
        if (canTx_queueFullDrops < 0xFFFFFFFFUL) { canTx_queueFullDrops++; }
        return CAN_ERROR_TX_BUSY;
    }

    waiting = uxQueueMessagesWaiting(canTxQueue);
    if ((uint32)waiting > canTx_peakDepth) { canTx_peakDepth = (uint32)waiting; }

    return CAN_OK;
}

uint32 CanTxQueue_GetQueueFullDrops(void) { return canTx_queueFullDrops; }
uint32 CanTxQueue_GetTxFailCount(void)    { return canTx_txFailCount;    }
uint32 CanTxQueue_GetTxTimeoutCount(void) { return canTx_txTimeoutCount; }
uint32 CanTxQueue_GetTxCount(void)        { return canTx_txCount;        }
uint32 CanTxQueue_GetPeakDepth(void)      { return canTx_peakDepth;      }
uint32 CanTxQueue_GetStackHwm(void)       { return canTx_stackHwm;       }

#else /* !USE_FREERTOS ------------------------------------------------------- */

/*----------------------------------------------------------------------------
 * NON-RTOS BUILD: a direct pass-through, so the 18 bench harnesses and the B3
 * rollback build transmit EXACTLY as they did before B6 - same function, same
 * return value, same call site. The comm modules therefore need no #ifdef.
 *
 * The counters exist so the heartbeat and any diagnostic compile in both modes;
 * they simply stay at 0, because in this mode a failure is reported through the
 * return value at the call site as it always was.
 *--------------------------------------------------------------------------*/
Std_ReturnType CanTxQueue_Init(void)
{
    return E_OK;                        /* nothing to bring up */
}

Can_StatusType CanTxQueue_Post(uint32 id, const uint8 *data, uint8 dlc)
{
    return Can_Transmit(id, data, dlc);
}

uint32 CanTxQueue_GetQueueFullDrops(void) { return 0U; }
uint32 CanTxQueue_GetTxFailCount(void)    { return 0U; }
uint32 CanTxQueue_GetTxTimeoutCount(void) { return 0U; }
uint32 CanTxQueue_GetTxCount(void)        { return 0U; }
uint32 CanTxQueue_GetPeakDepth(void)      { return 0U; }
uint32 CanTxQueue_GetStackHwm(void)       { return 0U; }

#endif /* USE_FREERTOS */
