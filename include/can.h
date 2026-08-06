/******************************************************************************
 *
 * Module: CAN (MCAL)
 *
 * File Name: can.h
 *
 * Description: Public API for the CAN0 transport MCAL. TRANSPORT ONLY: moves
 *              raw bytes (frame_id + uint8[8] + dlc) on/off the bus. It has NO
 *              knowledge of message meaning, never calls pack/unpack, and never
 *              includes the cantools robot.h. (Classic CAN, 11-bit standard IDs.)
 *
 *              This step implements INIT + TX. RX is reserved (IF2) and not yet
 *              implemented. The design is interrupt-ready; for bring-up, TX
 *              completion is verified by polling (see can.c).
 *
 ******************************************************************************/

#ifndef CAN_H_
#define CAN_H_

#include "Platform_Types.h"
#include "can_cfg.h"

typedef enum {
    CAN_OK = 0,
    CAN_ERROR_INVALID_DLC,     /* dlc > 8                                          */
    CAN_ERROR_TX_TIMEOUT,      /* poll expired, no specific cause (LEC 0/7)        */
    CAN_ERROR_NO_ACK,          /* TX failed, CANSTS.LEC = 3 (nobody acknowledged)  */
    CAN_ERROR_BUS_OFF,         /* CANSTS.BOFF set - controller off the bus         */
    CAN_ERROR_BUS_ERROR,       /* TX failed, LEC = stuff/form/bit/CRC              */
    CAN_RECOVERING,            /* Can_RecoverBusOff cleared INIT; rejoin pending   */
    CAN_NO_DATA,               /* RX: no new frame in the RX object                */
    CAN_RX_OVERRUN,            /* RX: frame delivered, but >=1 earlier frame lost  */
    CAN_ERROR_TX_BUSY          /* TX: a previously staged frame is still pending   */
} Can_StatusType;

/**
 * @brief  One-time CAN0 bring-up: clock + ready-wait, bit timing (500 kbps),
 *         message-object init, leave init mode. Call once at boot, AFTER
 *         Port_Init() (PE4/PE5 are muxed by the PORT driver - Option B).
 */
void Can_Init(void);

/**
 * @brief  Transmit one Classic-CAN standard (11-bit) data frame via IF1 / TX
 *         message object. Blocks (bring-up) until the frame is sent or a bounded
 *         timeout elapses.
 * @param  frame_id : 11-bit standard identifier (0..0x7FF).
 * @param  data     : pointer to up to 8 payload bytes (byte 0 first on the bus).
 * @param  dlc      : data length 0..8.
 * @return CAN_OK             : frame staged and IRQ 39 software-pended (queued).
 *         CAN_ERROR_INVALID_DLC : dlc > 8.
 *         CAN_ERROR_TX_BUSY   : a previously staged frame has not completed yet.
 * @note   NON-BLOCKING. The frame is loaded into TX object 1 BY THE ISR (the
 *         main thread touches no IF bank). TX completion is observed via
 *         Can_IsTxComplete(); a no-ACK / bus-off shows up asynchronously via
 *         Can_GetLastBusError() (status-interrupt path), not in this return.
 */
Can_StatusType Can_Transmit(uint32 frame_id, const uint8 *data, uint8 dlc);

/**
 * @brief  Whether the most recently staged TX frame has completed (ISR set this
 *         on object-1 transmit-complete; cleared when the next Can_Transmit
 *         stages a frame). Replaces the old blocking TXRQST poll.
 * @return TRUE if the last staged TX has completed.
 */
boolean Can_IsTxComplete(void);

/**
 * @brief  Most recent bus condition recorded by the ISR's status-interrupt path
 *         (CAN_OK / CAN_ERROR_NO_ACK / CAN_ERROR_BUS_OFF / CAN_ERROR_BUS_ERROR).
 * @note   OBSERVE ONLY - recovery remains the caller's job via Can_RecoverBusOff.
 * @return Last recorded bus status.
 */
Can_StatusType Can_GetLastBusError(void);

/**
 * @brief  Total RX frames lost since boot (VISIBLE overrun): the RX FIFO's
 *         drop-on-full count plus any hardware message-object losses (MSGLST).
 *         Monotonic, saturating. OBSERVE only - lets a high-rate flood show
 *         up as a count instead of silent loss.
 * @return cumulative dropped/lost RX frame count.
 */
uint32 Can_GetRxOverrunCount(void);

/**
 * @brief  Number of times an IF-register BUSY wait hit its iteration cap
 *         (C6-3). Non-zero means the CAN message-RAM interface stalled and an
 *         IF operation was aborted - a hardware-level fault indicator, not a
 *         normal condition. Should stay 0 forever in a healthy system.
 * @return Saturating lifetime count.
 */
uint32 Can_GetIfTimeoutCount(void);

/**
 * @brief  Number of in-flight TX frames abandoned because they did not complete
 *         within CAN_CFG_TX_INFLIGHT_LIMIT consecutive Can_Transmit attempts
 *         (C6-2). Non-zero means frames were overwritten before reaching the
 *         bus - expected when there is no ACK partner, otherwise a red flag.
 * @return Saturating lifetime count.
 */
uint32 Can_GetTxAbandonCount(void);

/**
 * @brief  Bus-off recovery MECHANISM (policy-free). If the controller is in
 *         bus-off (CANSTS.BOFF set), clear CANCTL.INIT to start the recovery
 *         sequence; hardware then waits 129 x 11 recessive bits before
 *         rejoining and resets the error counters (p1069). Does nothing if not
 *         bus-off. This is NEVER called automatically by the driver - the
 *         caller invokes it by policy.
 * @return CAN_OK         : not bus-off, nothing to do.
 *         CAN_RECOVERING : was bus-off, INIT cleared, rejoin in progress
 *                          (re-check with a later Can_Transmit / BOFF poll).
 */
Can_StatusType Can_RecoverBusOff(void);

/**
 * @brief  Poll the RX message object and, if a frame has arrived, deliver it as
 *         raw bytes. Non-blocking. Uses IF2 exclusively (TX uses IF1).
 * @param  frame_id : out, received 11-bit standard identifier (0..0x7FF).
 * @param  data     : out, buffer of >= 8 bytes; byte 0 is first on the bus.
 * @param  dlc      : out, received data length 0..8.
 * @note   NON-BLOCKING read of the single-frame buffer the ISR stores on receive
 *         (the IF2 transfer happens in the ISR; the caller touches no IF bank).
 * @return CAN_OK         : a frame was delivered.
 *         CAN_RX_OVERRUN : a frame was delivered, but at least one earlier frame
 *                          was overwritten before it was read (MSGLST).
 *         CAN_NO_DATA    : nothing new - outputs untouched.
 */
Can_StatusType Can_Receive(uint32 *frame_id, uint8 *data, uint8 *dlc);

#ifdef USE_FREERTOS
/*==============================================================================
 *  B6 - TX-COMPLETE SIGNAL. FreeRTOS builds only.
 *
 *  The controller has ONE hardware TX message object, so transmits are strictly
 *  serial: ~222 us per 8-byte frame at 500 kbps. tCanTx must therefore wait for
 *  the object to free between frames - and it must WAIT, not poll, or it would
 *  burn a whole core spinning on CAN_ERROR_TX_BUSY.
 *
 *  These three functions expose the ISR's TX-complete edge as a blocking wait.
 *  Can_TxDone / Can_IsTxComplete are unchanged and still drive the C6-2 state
 *  machine; this is an ADDITIONAL edge, not a replacement.
 *
 *  Intended use, in tCanTx and nowhere else:
 *      Can_TxCompleteClear();                  // discard any stale edge
 *      if (Can_Transmit(...) == CAN_OK) {
 *          (void)Can_WaitTxComplete(timeout);  // sleep until the wire is free
 *      }
 *============================================================================*/

/** @brief Create the TX-complete semaphore. Call ONCE, after Can_Init() and
 *         BEFORE vTaskStartScheduler(). Statically allocated - cannot fail. */
void Can_TxCompleteSignalInit(void);

/** @brief Discard a pending TX-complete edge before starting a new transmit.
 *
 * @note  ⚠️ NOT optional. A frame that completes AFTER its waiter timed out
 *        leaves the semaphore signalled; without this the next frame's wait
 *        would return immediately on the PREVIOUS frame's edge and tCanTx would
 *        believe the object was free while a transmit was still in flight.
 *        Never blocks. */
void Can_TxCompleteClear(void);

/** @brief Create the RX "ring non-empty" semaphore. Call ONCE, after Can_Init()
 *         and BEFORE vTaskStartScheduler(). Statically allocated. */
void Can_RxReadySignalInit(void);

/** @brief Block until the CAN0 ISR reports a frame in the RX ring, or timeout.
 * @param  timeoutMs  maximum block, milliseconds.
 * @return TRUE  a frame arrived (drain the ring with Can_Receive until NO_DATA).
 *         FALSE timed out - normal on an idle bus, not an error.
 *
 * @note   BINARY, so it saturates at one: N frames arriving between wakes give
 *         ONE wake. That is correct because the caller drains the WHOLE ring per
 *         wake; a counting semaphore would add a second source of truth about
 *         occupancy alongside the ring itself.
 * @note   The ring stays the BUFFER (it decouples ISR rate from task rate and
 *         absorbs bursts); this is only the wake edge.
 */
boolean Can_WaitRxReady(uint32 timeoutMs);

/** @brief Block until the in-flight frame completes, or the timeout expires.
 * @param  timeoutMs  maximum block, milliseconds.
 * @return TRUE  the TX-complete interrupt fired.
 *         FALSE timed out.
 *
 * @note   ⚠️ A TIMEOUT IS A NORMAL, EXPECTED OUTCOME, not a defect - and the
 *         timeout is what stops this from wedging. With no other node on the
 *         bus nothing ACKs, so the controller retransmits forever, TXRQST never
 *         clears and the TX-complete source NEVER fires (the exact case the
 *         C6-2 comment in Can_Transmit describes, reproduced on this bench
 *         every time the CANable is unplugged). An untimed wait would park
 *         tCanTx permanently and take all telemetry down with it. On timeout
 *         the caller should count it and carry on; the next Can_Transmit hits
 *         the C6-2 in-flight limit and abandons the stuck frame. */
boolean Can_WaitTxComplete(uint32 timeoutMs);
#endif /* USE_FREERTOS */

#endif /* CAN_H_ */
