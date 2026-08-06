/******************************************************************************
 * CAN Driver (MCAL) - CAN0 transport, TX-first bring-up
 *
 * Classic CAN 2.0A (11-bit standard IDs), 500 kbps @ 16 MHz. Transport only:
 * moves raw bytes; no knowledge of message meaning. IF1 = TX, IF2 = RX (later).
 *
 * Pins PE4 (CAN0Rx) / PE5 (CAN0Tx) are muxed by the PORT driver
 * (PORT_PBCFG.c, mode 8) - this MCAL does not touch GPIO [Option B].
 *
 * Init/TX sequence follows CAN_PERIPHERAL_NOTES.md section 5 / 6.
 ******************************************************************************/

#include "Platform_Types.h"
#include "tm4c123gh6pm_registers.h"   /* SYSCTL_RCGCCAN/PRCAN (clock + ready)  */
#include "can.h"
#include "can_private.h"
#include "can_frame.h"                /* Can_FrameType (raw RX/TX frame)        */
#include "RING_BUFFER.h"             /* RX FIFO (SPSC: ISR push, main pop)     */

#ifdef USE_FREERTOS
/* B6 only: the ISR raises a TX-complete edge for tCanTx. See the block below
 * the shared-state declarations for why the MCAL is allowed to know this. */
#include "FreeRTOS.h"
#include "semphr.h"
#endif

/* SYSCTL_RCGCCAN/PRCAN bit 0 = CAN0 (already defined in the registers header). */
#define CAN_PRIV_RCGCCAN_CAN0_MASK   (1UL << 0)
#define CAN_PRIV_PRCAN_CAN0_MASK     (1UL << 0)

/*******************************************************************************
 *            Shared state between the CAN0 ISR and the main thread            *
 *  All volatile (written by CAN0_Handler, read/written by the Can_* API).      *
 *******************************************************************************/
static volatile boolean        Can_TxPending  = FALSE;   /* staged frame waiting for ISR load */
static volatile boolean        Can_TxDone     = TRUE;    /* ISR set on obj-1 TX complete; TRUE = idle */
/* C6-2: consecutive Can_Transmit refusals while a frame is still in flight.
 * Bounds how long a never-completing frame may block TX (see Can_Transmit). */
static volatile uint32         Can_TxBusyStreak   = 0U;
static volatile uint32         Can_TxAbandonCount = 0U;
static volatile Can_StatusType Can_BusStatus  = CAN_OK;  /* ISR-recorded BOFF/LEC (observe)    */
/* C6-3: IF-register BUSY wait timeouts. Written by the ISR paths; also by the
 * init paths, which run before the NVIC line is enabled, so it stays
 * single-writer at any instant (same argument as RingBuffer_Init). */
static volatile uint32 Can_IfTimeoutCount = 0U;

static volatile uint32 Can_TxStageId;                    /* staged TX frame (single-frame)     */
static volatile uint8  Can_TxStageData[8];
static volatile uint8  Can_TxStageDlc;

/* RX now goes through the SPSC ring buffer (RING_BUFFER.*): the ISR pushes,
 * Can_Receive pops. The single-frame Can_RxFrame/Can_RxNew/Can_RxOverrun store
 * is retired. Can_RxHwLost counts frames lost at the hardware message object
 * (MSGLST) - a loss the ring buffer cannot see because that frame never reached
 * RingBuffer_Push. ISR is the only writer. */
static volatile uint32 Can_RxHwLost = 0U;

/* TODO(TX FIFO, future): the ring buffer is a single instance used for RX only.
 * A TX FIFO would need a second instance / a parameterized buffer. */

/*******************************************************************************
 *  B6: TX-COMPLETE SIGNAL FOR tCanTx (FreeRTOS builds only)                    *
 *                                                                              *
 *  The CAN controller has exactly ONE hardware TX message object, and a frame  *
 *  occupies it for ~222 us at 500 kbps. Pre-B6 the super-loop kept transmits   *
 *  apart by hand, one per millisecond slot. B6 replaces that with a queue      *
 *  drained by a single tCanTx task - which means tCanTx needs to know WHEN the *
 *  object is free again, and it must learn that by SLEEPING, not by spinning   *
 *  on CAN_ERROR_TX_BUSY.                                                       *
 *                                                                              *
 *  So the ISR's existing TX-complete branch - which already sets Can_TxDone -  *
 *  additionally gives a binary semaphore. Can_TxDone is KEPT, unchanged: it is *
 *  still the C6-2 state machine's flag and still what non-RTOS builds use.     *
 *  The semaphore is purely an additional wake-up edge.                         *
 *                                                                              *
 *  ⚠️ LAYERING NOTE. This is the MCAL knowing about the RTOS, which the        *
 *  architecture otherwise forbids. It is unavoidable and deliberate: the       *
 *  interrupt lives in this file, so the ISR->task edge can only be raised from *
 *  here. It is confined to (a) this block, (b) two lines in CAN0_Handler, and  *
 *  (c) Can_WaitTxComplete below, all inside #ifdef USE_FREERTOS. Every         *
 *  non-RTOS env compiles a byte-identical driver.                              *
 *                                                                              *
 *  ⚠️ A BINARY SEMAPHORE, NOT A MUTEX - it is a SIGNAL, not a lock. It is      *
 *  given by an ISR (which owns nothing and cannot inherit priority) and taken  *
 *  by a task. A mutex here would be a category error.                          *
 *                                                                              *
 *  ⚠️ LEGAL ONLY BECAUSE CAN_CFG_NVIC_PRIORITY IS 5 (>= the syscall ceiling).  *
 *  See the long note in can_cfg.h. This is why B6 does the priority change     *
 *  FIRST.                                                                      *
 *******************************************************************************/
#ifdef USE_FREERTOS
static StaticSemaphore_t Can_TxDoneSemBuf;
static SemaphoreHandle_t Can_TxDoneSem = NULL;

void Can_TxCompleteSignalInit(void)
{
    /* Binary semaphore, created EMPTY (xSemaphoreCreateBinaryStatic starts
     * empty), which is correct: at init no transmit is outstanding, so there is
     * no completion to collect. */
    Can_TxDoneSem = xSemaphoreCreateBinaryStatic(&Can_TxDoneSemBuf);
}

void Can_TxCompleteClear(void)
{
    if (Can_TxDoneSem != NULL)
    {
        /* Discard a stale give left over from a previous frame - e.g. one that
         * completed after tCanTx had already timed out waiting for it. Without
         * this, the NEXT frame's wait would return instantly on the previous
         * frame's signal and tCanTx would believe the object was free while a
         * transmit was still in flight. Zero block time: this never waits. */
        (void)xSemaphoreTake(Can_TxDoneSem, 0);
    }
}

/*----------------------------------------------------------------------------
 * B8: RX "ring non-empty" wake signal for tRosRx.
 *
 * The SPSC ring buffer remains THE buffer - it is what decouples the ISR's
 * arrival rate from the task's drain rate, and it absorbs a burst. This
 * semaphore is only the WAKE EDGE that says "there is something to drain", so
 * tRosRx can sleep at portMAX_DELAY instead of polling the ring on a timer.
 *
 * ⚠️ A BINARY semaphore, deliberately, NOT counting. It SATURATES at one, so N
 * frames pushed between wakes produce ONE wake - and that is exactly right,
 * because tRosRx drains the WHOLE ring per wake. A counting semaphore would
 * make the task loop once per frame for no benefit and would have to be kept
 * in step with the ring's own occupancy, which is a second source of truth.
 *
 * ⚠️ Legal only because CAN_CFG_NVIC_PRIORITY is 5, at/below
 * configMAX_SYSCALL_INTERRUPT_PRIORITY - the BUDGET-2 change already made in
 * B6 for the TX-complete edge. B8 inherits it rather than introducing it.
 *--------------------------------------------------------------------------*/
static StaticSemaphore_t Can_RxReadySemBuf;
static SemaphoreHandle_t Can_RxReadySem = NULL;

void Can_RxReadySignalInit(void)
{
    Can_RxReadySem = xSemaphoreCreateBinaryStatic(&Can_RxReadySemBuf);
}

boolean Can_WaitRxReady(uint32 timeoutMs)
{
    if (Can_RxReadySem == NULL)
    {
        return FALSE;
    }
    return (xSemaphoreTake(Can_RxReadySem, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)
               ? TRUE : FALSE;
}

boolean Can_WaitTxComplete(uint32 timeoutMs)
{
    if (Can_TxDoneSem == NULL)
    {
        return FALSE;
    }
    return (xSemaphoreTake(Can_TxDoneSem, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)
               ? TRUE : FALSE;
}
#endif /* USE_FREERTOS */

/*---------------------------------------------------------------------------
 * Wait until IF1 finishes a RAM<->IF transfer (BUSY clears). p1080.
 *-------------------------------------------------------------------------*/
static boolean Can_WaitIf1NotBusy(void)
{
    uint32 spins = CAN_PRIV_IF_BUSY_SPIN_CAP;

    while (((CAN0_IF1CRQ_REG & CAN_PRIV_CRQ_BUSY_MASK) != 0UL) && (spins > 0UL))
    {
        spins--;
    }

    if (spins == 0UL)
    {
        if (Can_IfTimeoutCount < 0xFFFFFFFFUL) { Can_IfTimeoutCount++; }
        return FALSE;                     /* caller MUST abort the IF operation */
    }

    return TRUE;
}

/*---------------------------------------------------------------------------
 * Invalidate all message objects (clear MSGVAL) so RAM holds no garbage.
 * CAN_PERIPHERAL_NOTES.md section 5 step 6. Uses IF1.
 *-------------------------------------------------------------------------*/
static void Can_InvalidateAllObjects(void)
{
    uint8 obj;
    for (obj = 1U; obj <= (uint8)CAN_CFG_NUM_MSG_OBJECTS; obj++)
    {
        (void)Can_WaitIf1NotBusy();
        CAN0_IF1CMSK_REG = CAN_PRIV_CMSK_WRNRD_MASK | CAN_PRIV_CMSK_ARB_MASK;
        CAN0_IF1ARB2_REG = 0UL;                 /* MSGVAL = 0 -> not valid */
        CAN0_IF1CRQ_REG  = (uint32)obj & CAN_PRIV_CRQ_MNUM_MASK;
    }
    (void)Can_WaitIf1NotBusy();
}

/*---------------------------------------------------------------------------
 * Wait until IF2 finishes a RAM<->IF transfer (BUSY clears). p1080.
 *-------------------------------------------------------------------------*/
static boolean Can_WaitIf2NotBusy(void)
{
    uint32 spins = CAN_PRIV_IF_BUSY_SPIN_CAP;

    while (((CAN0_IF2CRQ_REG & CAN_PRIV_CRQ_BUSY_MASK) != 0UL) && (spins > 0UL))
    {
        spins--;
    }

    if (spins == 0UL)
    {
        if (Can_IfTimeoutCount < 0xFFFFFFFFUL) { Can_IfTimeoutCount++; }
        return FALSE;                     /* caller MUST abort the IF operation */
    }

    return TRUE;
}

/*---------------------------------------------------------------------------
 * Configure the single RX message object once (called from Can_Init). Standard
 * 11-bit, DIR=0 (receive), MSGVAL=1, acceptance mask enabled (UMASK), EOB=1.
 * Uses IF2 exclusively. CAN_PERIPHERAL_NOTES.md section 6.3 / p1055.
 *-------------------------------------------------------------------------*/
static void Can_ConfigRxObject(uint8 msgObj, uint16 acceptId, uint16 idMask)
{
    (void)Can_WaitIf2NotBusy();

    /* CMSK: write IF2 -> RAM, transfer MASK + ARB + CONTROL (no data). p1081 */
    CAN0_IF2CMSK_REG = CAN_PRIV_CMSK_RX_CONFIG;

    /* MSK: exact-ID filter. MSK1 unused for 11-bit -> 0. MSK2: ID mask in 12:2,
     * MXTD=1 (match standard frames only), MDIR=0 (don't filter on direction). p1085 */
    CAN0_IF2MSK1_REG = 0UL;
    CAN0_IF2MSK2_REG = CAN_PRIV_MSK2_MXTD_MASK |
                       (((uint32)idMask & 0x7FFUL) << CAN_PRIV_MSK2_MSK_STD_POS);

    /* ARB2: accept ID in 12:2, XTD=0 (standard), DIR=0 (receive), MSGVAL=1. p1088 */
    CAN0_IF2ARB1_REG = 0UL;
    CAN0_IF2ARB2_REG = CAN_PRIV_ARB2_MSGVAL_MASK |
                       (((uint32)acceptId & 0x7FFUL) << CAN_PRIV_ARB2_ID_STD_POS);

    /* MCTL: UMASK=1 (enable filter), EOB=1 (single object), RXIE=1 (interrupt on
     * receive), DLC max. Do NOT set NEWDAT/TXRQST. p1090 */
    CAN0_IF2MCTL_REG = CAN_PRIV_MCTL_UMASK_MASK | CAN_PRIV_MCTL_EOB_MASK |
                       CAN_PRIV_MCTL_RXIE_MASK  | (8UL & CAN_PRIV_MCTL_DLC_MASK);

    /* Commit to RAM. p1080 */
    CAN0_IF2CRQ_REG = (uint32)msgObj & CAN_PRIV_CRQ_MNUM_MASK;
    (void)Can_WaitIf2NotBusy();
}

void Can_Init(void)
{
    volatile uint32 delay;

    /* 0. Init ISR<->main shared state before any interrupt source is enabled.
     *    RingBuffer_Init() MUST precede the NVIC enable (step 8) - it is not
     *    ISR-safe (RB.h). */
    Can_TxPending      = FALSE;
    Can_TxDone         = TRUE;    /* nothing in flight at boot */
    Can_TxBusyStreak   = 0U;
    Can_TxAbandonCount = 0U;
    Can_BusStatus = CAN_OK;
    Can_RxHwLost  = 0U;
    (void)RingBuffer_Init();

    /* 1. Enable CAN0 clock, wait for ready, then >=3 clock settle (p1066). */
    SYSCTL_RCGCCAN_REG |= CAN_PRIV_RCGCCAN_CAN0_MASK;
    while ((SYSCTL_PRCAN_REG & CAN_PRIV_PRCAN_CAN0_MASK) != CAN_PRIV_PRCAN_CAN0_MASK) { }
    for (delay = 0U; delay < 3U; delay++) { /* 3-system-clock settle */ }

    /* 2. Pins: PE4/PE5 muxed by PORT driver (Option B) - nothing to do here. */

    /* 3. Enter init + enable config change (both required to write CANBIT). p1050 */
    CAN0_CTL_REG |= CAN_PRIV_CTL_INIT_MASK;
    CAN0_CTL_REG |= CAN_PRIV_CTL_CCE_MASK;

    /* 4. Bit timing: 500 kbps @ 16 MHz (register-encoded values, can_cfg.h). p1075 */
    CAN0_BIT_REG = (CAN_CFG_TSEG2_REG << CAN_PRIV_BIT_TSEG2_POS) |
                   (CAN_CFG_TSEG1_REG << CAN_PRIV_BIT_TSEG1_POS) |
                   (CAN_CFG_SJW_REG   << CAN_PRIV_BIT_SJW_POS)   |
                   (CAN_CFG_BRP_REG   << CAN_PRIV_BIT_BRP_POS);
    /* CANBRPE left at 0 (BRP <= 64, no extension needed). */

    /* 5. Mark every message object not-valid. p1050 step 6 */
    Can_InvalidateAllObjects();

    /* 6. Leave init: clear CCE then INIT -> controller joins the bus. p1050 step 7 */
    CAN0_CTL_REG &= ~CAN_PRIV_CTL_CCE_MASK;
    CAN0_CTL_REG &= ~CAN_PRIV_CTL_INIT_MASK;

    /* 7. Set up the RX message objects (IF2), each with RXIE. p1055
     *    obj 2: the Jetson command filter, mask 0x7DF -> {0x100, 0x120}.
     *    obj 3: the HOST liveness ping, EXACT match on 0x7A0 - it cannot share
     *           obj 2's mask (see the disjointness proof in can_cfg.h). */
    Can_ConfigRxObject((uint8)CAN_CFG_RX_MSG_OBJ,
                       (uint16)CAN_CFG_RX_ACCEPT_ID, (uint16)CAN_CFG_RX_ID_MASK);
    Can_ConfigRxObject((uint8)CAN_CFG_RX_PING_MSG_OBJ,
                       (uint16)CAN_CFG_RX_ACCEPT_PING_ID, (uint16)CAN_CFG_RX_PING_ID_MASK);
    Can_ConfigRxObject((uint8)CAN_CFG_RX_RESET_MSG_OBJ,
                       (uint16)CAN_CFG_RX_ACCEPT_RESET_ID, (uint16)CAN_CFG_RX_RESET_ID_MASK);

    /* 8. Interrupt enable chain (CAN_INTERRUPT_NOTES section 2). Module + status
     *    sources first; obj-2 RXIE is already set above; obj-1 TXIE is set per
     *    transmit in the ISR's IF1 load. Enable the NVIC line LAST so nothing
     *    fires before the driver state and message objects are ready. */
    CAN0_CTL_REG |= (CAN_PRIV_CTL_IE_MASK | CAN_PRIV_CTL_SIE_MASK | CAN_PRIV_CTL_EIE_MASK); /* p1069 */

    CAN_PRIV_NVIC_PRI9 = (CAN_PRIV_NVIC_PRI9 & ~(0x7UL << CAN_PRIV_NVIC_PRI9_INTD_POS)) |
                         ((uint32)CAN_CFG_NVIC_PRIORITY << CAN_PRIV_NVIC_PRI9_INTD_POS);     /* p152 */
    CAN_PRIV_NVIC_EN1  = (1UL << CAN_PRIV_NVIC_CAN0_BIT);   /* enable IRQ 39 (write-1) p142 */
}

Can_StatusType Can_Transmit(uint32 frame_id, const uint8 *data, uint8 dlc)
{
    uint8 i;

    if (dlc > 8U)
    {
        return CAN_ERROR_INVALID_DLC;
    }
    /* --- C6-2: the TX object is free only when the previous frame has actually
     * been TRANSMITTED, not merely loaded into the message object. ---
     *
     * State machine:
     *   idle            : TxPending == FALSE && TxDone == TRUE
     *   staged          : Can_Transmit sets TxDone = FALSE, TxPending = TRUE
     *   loaded          : ISR calls Can_LoadTxObject, clears TxPending
     *   transmitted     : ISR TX-complete source sets TxDone = TRUE  -> idle
     *
     * Gating on TxPending alone (the old behaviour) allowed a new stage as soon
     * as the frame was LOADED, so under arbitration loss or retransmission the
     * object could be rewritten and the earlier frame silently never reached
     * the bus. */
    if (Can_TxPending)
    {
        return CAN_ERROR_TX_BUSY;            /* staged, ISR has not loaded it yet */
    }

    if (Can_TxDone == FALSE)
    {
        /* Loaded but not yet transmitted. Refuse - BUT bounded, because a frame
         * can legitimately never complete: with no other node on the bus nothing
         * ACKs, the controller retransmits forever, TXRQST never clears and the
         * TX-complete interrupt never fires. Blocking on that unconditionally
         * would wedge TX permanently, which this very bench reproduces whenever
         * the CANable is unplugged. It also covers a missed TX interrupt.
         *
         * After CAN_CFG_TX_INFLIGHT_LIMIT consecutive refusals we abandon the
         * in-flight frame and overwrite the object - i.e. we fall back to the
         * OLD lossy behaviour, but only after genuinely waiting, and we count it
         * so the loss stays visible. */
        Can_TxBusyStreak++;
        if (Can_TxBusyStreak < CAN_CFG_TX_INFLIGHT_LIMIT)
        {
            return CAN_ERROR_TX_BUSY;
        }
        if (Can_TxAbandonCount < 0xFFFFFFFFUL) { Can_TxAbandonCount++; }
    }

    Can_TxBusyStreak = 0U;

    /* Stage the frame (no IF access - IF1 is owned by the ISR). */
    Can_TxStageId  = frame_id;
    Can_TxStageDlc = dlc;
    for (i = 0U; i < 8U; i++) { Can_TxStageData[i] = (i < dlc) ? data[i] : 0U; }

    Can_TxDone    = FALSE;
    Can_TxPending = TRUE;                     /* publish AFTER the data is staged */

    /* Kick the ISR to perform the IF1 load: software-pend IRQ 39. p157 */
    CAN_PRIV_NVIC_PEND1 = (1UL << CAN_PRIV_NVIC_CAN0_BIT);

    return CAN_OK;
}

Can_StatusType Can_RecoverBusOff(void)
{
    if ((CAN0_STS_REG & CAN_PRIV_STS_BOFF_MASK) == 0UL)
    {
        return CAN_OK;                       /* not bus-off - nothing to do */
    }

    /* Bus-off recovery: clear INIT (hardware auto-set it on bus-off). The
     * controller then waits 129 x 11 recessive bits before rejoining and resets
     * the error counters. Rejoin is asynchronous, so report "recovering", not
     * "recovered". p1069 / CAN_PERIPHERAL_NOTES.md section 5. */
    CAN0_CTL_REG &= ~CAN_PRIV_CTL_INIT_MASK;

    return CAN_RECOVERING;
}

Can_StatusType Can_Receive(uint32 *frame_id, uint8 *data, uint8 *dlc)
{
    static uint32 lastOverrun = 0U;   /* consumer-only (main); no race */
    Can_FrameType frame;
    uint8  i;
    uint32 nowOverrun;

    /* Non-blocking pop from the RX FIFO (lock-free SPSC; no IF access, no mask). */
    if (RingBuffer_Pop(&frame) != RING_BUFFER_OK)
    {
        return CAN_NO_DATA;
    }

    *frame_id = frame.id;
    *dlc      = frame.dlc;
    for (i = 0U; i < 8U; i++) { data[i] = frame.data[i]; }

    /* Synthesize CAN_RX_OVERRUN: >=1 frame was dropped/lost since the previous
     * successful pop. The absolute total is Can_GetRxOverrunCount(). */
    nowOverrun = Can_GetRxOverrunCount();
    if (nowOverrun != lastOverrun)
    {
        lastOverrun = nowOverrun;
        return CAN_RX_OVERRUN;
    }
    return CAN_OK;
}

/*---------------------------------------------------------------------------
 * ISR-only: load the staged frame into TX object 1 (IF1) and request transmit.
 * IF1 is owned exclusively by the ISR. TXIE is set so completion raises an
 * interrupt. p1081/p1088/p1090/p1093/p1080.
 *-------------------------------------------------------------------------*/
static void Can_LoadTxObject(void)
{
    uint32 arb2;

    if (Can_WaitIf1NotBusy() == FALSE)
    {
        return;                 /* IF1 stuck: abort this load, next IRQ retries */
    }
    CAN0_IF1CMSK_REG = CAN_PRIV_CMSK_TX_WRITE;

    arb2 = CAN_PRIV_ARB2_MSGVAL_MASK | CAN_PRIV_ARB2_DIR_MASK |
           (((uint32)Can_TxStageId & 0x7FFUL) << CAN_PRIV_ARB2_ID_STD_POS);
    CAN0_IF1ARB1_REG = 0UL;
    CAN0_IF1ARB2_REG = arb2;

    /* MCTL: DLC, single object (EOB), TX interrupt enable (TXIE), request TX. p1090 */
    CAN0_IF1MCTL_REG = ((uint32)Can_TxStageDlc & CAN_PRIV_MCTL_DLC_MASK) |
                       CAN_PRIV_MCTL_EOB_MASK | CAN_PRIV_MCTL_TXIE_MASK |
                       CAN_PRIV_MCTL_TXRQST_MASK;

    CAN0_IF1DA1_REG = (uint32)Can_TxStageData[0] | ((uint32)Can_TxStageData[1] << 8);
    CAN0_IF1DA2_REG = (uint32)Can_TxStageData[2] | ((uint32)Can_TxStageData[3] << 8);
    CAN0_IF1DB1_REG = (uint32)Can_TxStageData[4] | ((uint32)Can_TxStageData[5] << 8);
    CAN0_IF1DB2_REG = (uint32)Can_TxStageData[6] | ((uint32)Can_TxStageData[7] << 8);

    CAN0_IF1CRQ_REG = (uint32)CAN_CFG_TX_MSG_OBJ & CAN_PRIV_CRQ_MNUM_MASK;
    (void)Can_WaitIf1NotBusy();   /* completion wait; bounded, result advisory */
}

/*---------------------------------------------------------------------------
 * ISR-only: read RX object 2 out of IF2 into the RX store. CAN_PRIV_CMSK_RX_READ
 * includes CLRINTPND, so the one transfer clears NEWDAT + INTPND together.
 * IF2 is owned exclusively by the ISR. p1056/p1081/p1090/p1093.
 *-------------------------------------------------------------------------*/
static void Can_ReadRxObject(uint8 msgObj)
{
    uint32 mctl, da1, da2, db1, db2;
    Can_FrameType frame;

    if (Can_WaitIf2NotBusy() == FALSE)
    {
        return;                 /* IF2 stuck: abort this read, next IRQ retries */
    }
    CAN0_IF2CMSK_REG = CAN_PRIV_CMSK_RX_READ;
    CAN0_IF2CRQ_REG  = (uint32)msgObj & CAN_PRIV_CRQ_MNUM_MASK;
    (void)Can_WaitIf2NotBusy();

    mctl      = CAN0_IF2MCTL_REG;
    frame.id  = (CAN0_IF2ARB2_REG >> CAN_PRIV_ARB2_ID_STD_POS) & 0x7FFUL;
    frame.dlc = (uint8)(mctl & CAN_PRIV_MCTL_DLC_MASK);

    /* Data: byte 0 in DA1[7:0] ... byte 7 in DB2[15:8] - SAME mapping as TX. p1093 */
    da1 = CAN0_IF2DA1_REG; da2 = CAN0_IF2DA2_REG;
    db1 = CAN0_IF2DB1_REG; db2 = CAN0_IF2DB2_REG;
    frame.data[0]=(uint8)(da1&0xFFU);  frame.data[1]=(uint8)((da1>>8)&0xFFU);
    frame.data[2]=(uint8)(da2&0xFFU);  frame.data[3]=(uint8)((da2>>8)&0xFFU);
    frame.data[4]=(uint8)(db1&0xFFU);  frame.data[5]=(uint8)((db1>>8)&0xFFU);
    frame.data[6]=(uint8)(db2&0xFFU);  frame.data[7]=(uint8)((db2>>8)&0xFFU);

    /* Overrun at the HARDWARE object: MSGLST means a frame arrived while the
     * previous was unread (lost before it ever reached the FIFO). Count it so
     * the loss stays visible, and clear it (force NEWDAT=0 in the write-back so
     * the IF2 pre-clear NEWDAT copy does not re-arm the object). p1090 / p1056 */
    if ((mctl & CAN_PRIV_MCTL_MSGLST_MASK) != 0UL)
    {
        if (Can_RxHwLost < 0xFFFFFFFFUL) { Can_RxHwLost++; }
        CAN0_IF2MCTL_REG = mctl & ~(CAN_PRIV_MCTL_MSGLST_MASK | CAN_PRIV_MCTL_NEWDAT_MASK);
        (void)Can_WaitIf2NotBusy();
        CAN0_IF2CMSK_REG = CAN_PRIV_CMSK_RX_LSTCLR;     /* write MCTL back (CONTROL) */
        CAN0_IF2CRQ_REG  = (uint32)msgObj & CAN_PRIV_CRQ_MNUM_MASK;
        (void)Can_WaitIf2NotBusy();
    }

    /* Publish the frame into the RX FIFO. On full, Push drops the newest and
     * bumps RingBuffer_GetDropCount() - the loss is VISIBLE (the whole point). */
    (void)RingBuffer_Push(&frame);

#ifdef USE_FREERTOS
    /* B8: wake tRosRx - the ring has something to drain.
     *
     * Given unconditionally, including when Push dropped: the ring is non-empty
     * either way, so there IS work, and suppressing the wake on a drop would be
     * the one moment we most want the consumer to run.
     *
     * portYIELD_FROM_ISR matters here more than for TX: tRosRx is priority 7,
     * above everything currently running except tVelocity/tSafety, so without
     * the yield a freshly-arrived command would wait up to a full tick before
     * being routed - adding 1 ms to command latency and to the failsafe's
     * re-arm, for every frame. */
    if (Can_RxReadySem != NULL)
    {
        BaseType_t higherWoken = pdFALSE;

        (void)xSemaphoreGiveFromISR(Can_RxReadySem, &higherWoken);
        portYIELD_FROM_ISR(higherWoken);
    }
#endif
}

boolean Can_IsTxComplete(void)
{
    return Can_TxDone;
}

Can_StatusType Can_GetLastBusError(void)
{
    return Can_BusStatus;
}

uint32 Can_GetTxAbandonCount(void)
{
    return Can_TxAbandonCount;
}

uint32 Can_GetIfTimeoutCount(void)
{
    return Can_IfTimeoutCount;
}

uint32 Can_GetRxOverrunCount(void)
{
    /* Total visible RX loss: FIFO drop-on-full + hardware MSGLST losses. */
    return RingBuffer_GetDropCount() + Can_RxHwLost;
}

/*---------------------------------------------------------------------------
 * CAN0 interrupt handler (IRQ 39 / vector 55). Installed in startup_tm4c123.c,
 * mirroring UART1_Handler. SOLE owner of IF1 (TX) and IF2 (RX) after init.
 * CAN_INTERRUPT_NOTES.md section 3/4/5/6.
 *-------------------------------------------------------------------------*/
void CAN0_Handler(void)
{
    uint32 intid;

    /* Software-pend kick from Can_Transmit: start the staged TX (ISR owns IF1). */
    if (Can_TxPending)
    {
        Can_LoadTxObject();
        Can_TxPending = FALSE;
    }

    /* Service-to-zero: drain all pending sources (level-style line). section 3/7 */
    for (;;)
    {
        intid = CAN0_INT_REG & 0xFFFFUL;             /* CANINT.INTID, p1076 */
        if (intid == CAN_PRIV_INT_NONE)
        {
            break;
        }

        if (intid == CAN_PRIV_INT_STATUS)
        {
            /* Status path: read CANSTS (clears the status source). OBSERVE ONLY -
             * record BOFF/LEC, never recover (recovery stays Can_RecoverBusOff,
             * the caller's tool). section 4b/6. p1071 */
            uint32 sts = CAN0_STS_REG;
            uint32 lec = sts & CAN_PRIV_STS_LEC_MASK;
            if ((sts & CAN_PRIV_STS_BOFF_MASK) != 0UL)
            {
                Can_BusStatus = CAN_ERROR_BUS_OFF;
            }
            else if (lec == CAN_PRIV_STS_LEC_ACK)
            {
                Can_BusStatus = CAN_ERROR_NO_ACK;
            }
            else if ((lec != CAN_PRIV_STS_LEC_NO_ERROR) &&
                     (lec != CAN_PRIV_STS_LEC_NO_EVENT))
            {
                Can_BusStatus = CAN_ERROR_BUS_ERROR;
            }
            else
            {
                Can_BusStatus = CAN_OK;
            }
        }
        else if (intid == (uint32)CAN_CFG_TX_MSG_OBJ)    /* TX done (object 1) */
        {
            /* Mark complete + clear INTPND via an IF1 read with CLRINTPND. section 5a/p1059 */
            Can_TxDone = TRUE;

#ifdef USE_FREERTOS
            /* B6: wake tCanTx - the hardware TX object is free again.
             * Legal only because CAN_CFG_NVIC_PRIORITY is 5, i.e. at or below
             * configMAX_SYSCALL_INTERRUPT_PRIORITY (see can_cfg.h). */
            if (Can_TxDoneSem != NULL)
            {
                BaseType_t higherWoken = pdFALSE;

                (void)xSemaphoreGiveFromISR(Can_TxDoneSem, &higherWoken);

                /* Request the context switch on ISR exit if tCanTx now outranks
                 * whatever was interrupted. Without this the wake-up would wait
                 * for the next tick - up to 1 ms of dead air on the bus for
                 * every single frame, which would cap throughput at ~1 kHz and
                 * silently under-run the 100 Hz feedback frames. */
                portYIELD_FROM_ISR(higherWoken);
            }
#endif
            if (Can_WaitIf1NotBusy() != FALSE)
            {
                CAN0_IF1CMSK_REG = CAN_PRIV_CMSK_TX_INTACK;
                CAN0_IF1CRQ_REG  = (uint32)CAN_CFG_TX_MSG_OBJ & CAN_PRIV_CRQ_MNUM_MASK;
                (void)Can_WaitIf1NotBusy();
            }
        }
        else if (intid == (uint32)CAN_CFG_RX_MSG_OBJ)    /* RX commands (object 2) */
        {
            Can_ReadRxObject((uint8)CAN_CFG_RX_MSG_OBJ);   /* IF2 read-out clears NEWDAT+INTPND; stores frame. section 5b */
        }
        else if (intid == (uint32)CAN_CFG_RX_RESET_MSG_OBJ) /* RX ResetCommand (object 4) */
        {
            Can_ReadRxObject((uint8)CAN_CFG_RX_RESET_MSG_OBJ);
        }
        else if (intid == (uint32)CAN_CFG_RX_PING_MSG_OBJ) /* RX liveness ping (object 3) */
        {
            /* Same read-out path, different object. The frame lands in the same
             * ring buffer; the dispatch routes on the id. */
            Can_ReadRxObject((uint8)CAN_CFG_RX_PING_MSG_OBJ);
        }
        else
        {
            /* Unexpected object: clear its INTPND defensively (avoid a stuck line). */
            if (Can_WaitIf1NotBusy() != FALSE)
            {
                CAN0_IF1CMSK_REG = CAN_PRIV_CMSK_TX_INTACK;
                CAN0_IF1CRQ_REG  = intid & CAN_PRIV_CRQ_MNUM_MASK;
                (void)Can_WaitIf1NotBusy();
            }
        }
    }
}
