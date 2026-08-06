/******************************************************************************
 * can_tx_queue.h - the single CAN transmit path (Tier 3)
 *
 * ARCHITECTURE (ARCHITECTURE_app_layer.md section 2, Tier 3): "Both call a CAN TX
 * QUEUE, never Can_Transmit directly." This module IS that queue. From B6
 * onward, every frame this node sends goes through CanTxQueue_Post().
 *
 * ---------------------------------------------------------------------------
 * WHY IT EXISTS
 *
 * The TM4C123 CAN controller has exactly ONE hardware TX message object, and an
 * 8-byte standard frame occupies it for ~222 us at 500 kbps. Transmits are
 * therefore strictly serial, and something has to enforce that.
 *
 * BEFORE B6 that something was HAND-TIMED SLOT ARITHMETIC in main.c: five
 * senders placed on mutually-exclusive millisecond residues (0 and 5 mod 10; 9,
 * 47, 51 mod 100), with a written proof in the file header that no two could
 * ever coincide. It worked - but it was CONVENTIONAL, not structural: it held
 * only as long as every future author re-derived the residue arithmetic before
 * adding a sixth frame.
 *
 * AFTER B6 the enforcement is STRUCTURAL. Senders post; ONE task (tCanTx) is the
 * only caller of Can_Transmit in the entire image, and it sleeps on the ISR's
 * TX-complete edge between frames. One task plus one hardware object cannot
 * overlap two transmits, whatever anyone adds later.
 * ---------------------------------------------------------------------------
 *
 * TWO BUILD MODES, one API:
 *   USE_FREERTOS   - Post() enqueues; tCanTx drains and transmits.
 *   (non-RTOS)     - Post() is a direct pass-through to Can_Transmit(), so the
 *                    18 bench harnesses and the rollback build behave EXACTLY
 *                    as they did before B6. Nothing in the comm modules needs
 *                    to know which mode it is in - which is why they contain no
 *                    #ifdef at all.
 *
 * Returns Can_StatusType deliberately: the five migrated call sites already read
 * `if (X(...) != CAN_OK)`, so adopting the queue was a one-identifier change per
 * site with no logic edit anywhere.
 *****************************************************************************/

#ifndef CAN_TX_QUEUE_H_
#define CAN_TX_QUEUE_H_

#include "Platform_Types.h"
#include "Std_Types.h"
#include "can.h"

/**
 * @brief  Bring up the queue and start tCanTx.
 * @return E_OK on success; E_NOT_OK if the task could not be created.
 * @note   FreeRTOS builds ONLY (a no-op returning E_OK otherwise). Call once,
 *         AFTER Can_Init() and BEFORE vTaskStartScheduler(). Statically
 *         allocated throughout - there is no heap to exhaust.
 */
Std_ReturnType CanTxQueue_Init(void);

/**
 * @brief  Submit one frame for transmission. THE ONLY WAY TO SEND FROM B6 ON.
 * @param  id    standard 11-bit frame id.
 * @param  data  payload, >= dlc bytes.
 * @param  dlc   0..8.
 * @return CAN_OK             accepted (queued, or transmitted in non-RTOS builds).
 *         CAN_ERROR_TX_BUSY  queue full - FRAME DROPPED, and counted.
 *         CAN_ERROR          bad argument.
 *
 * @note   ⚠️ NEVER BLOCKS. The enqueue uses a ZERO timeout, always. A full queue
 *         drops the frame rather than stalling the caller, and that is a
 *         deliberate priority judgement: from B9/B10 the callers include
 *         tVelocity and tSafety, and no telemetry frame is worth delaying a
 *         control update or a failsafe. Drops are counted, never silent
 *         (CanTxQueue_GetQueueFullDrops) - which is what closes A4-3.
 */
Can_StatusType CanTxQueue_Post(uint32 id, const uint8 *data, uint8 dlc);

/*==============================================================================
 *  A4-3 OBSERVABILITY - "every TX failure is silently discarded" CLOSED HERE.
 *
 *  The finding was that all six Can_Transmit call sites `(void)`'d the return,
 *  so failures vanished - unlike Can_GetIfTimeoutCount / UART_GetTxDroppedCount,
 *  and unlike node_ping.c which already did it right.
 *
 *  The queue splits that single silence into THREE causes, counted separately
 *  BECAUSE THEY MEAN DIFFERENT THINGS AND DEMAND DIFFERENT FIXES. A lumped
 *  "dropped" counter would have made them indistinguishable:
 *============================================================================*/

/** @brief Frames dropped because the queue was full.
 *  @note  A SIZING problem: a producer outran the drainer. Expect 0; a non-zero
 *         value means either the queue is too shallow or a sender is too eager. */
uint32 CanTxQueue_GetQueueFullDrops(void);

/** @brief Frames tCanTx accepted but Can_Transmit then refused.
 *  @note  A BUS problem. Can_Transmit only refuses after the C6-2 in-flight
 *         limit, so this means arbitration loss or a genuinely stuck object. */
uint32 CanTxQueue_GetTxFailCount(void);

/** @brief Frames transmitted whose TX-complete edge never arrived in time.
 *  @note  The classic "no other node on the bus" signature: nothing ACKs, the
 *         controller retransmits forever and the TX-complete source never
 *         fires. NOT a defect in itself - see Can_WaitTxComplete. */
uint32 CanTxQueue_GetTxTimeoutCount(void);

/** @brief Frames successfully handed to the hardware since boot. */
uint32 CanTxQueue_GetTxCount(void);

/** @brief Deepest the queue has ever been, in messages. Sizes the depth. */
uint32 CanTxQueue_GetPeakDepth(void);

/** @brief tCanTx stack high-water mark, in words. 0 before the task has run. */
uint32 CanTxQueue_GetStackHwm(void);

#endif /* CAN_TX_QUEUE_H_ */
