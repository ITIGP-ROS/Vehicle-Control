/******************************************************************************
 *
 * Module: CAN (MCAL)
 *
 * File Name: can_cfg.h
 *
 * Description: Compile-time configuration for the CAN0 MCAL: clock, target
 *              bitrate, bit-timing field values (500 kbps @ 16 MHz), pin
 *              selection, and message-object assignment. Hardware facts only -
 *              no message meaning lives here.
 *
 ******************************************************************************/

#ifndef CAN_CFG_H_
#define CAN_CFG_H_

/*******************************************************************************
 *                          CAN ID SURFACE (at-a-glance map)                   *
 *  The complete set of CAN ids this node touches, and their direction. RX and *
 *  TX are separate concerns - each has its own section lower in this file.    *
 *                                                                             *
 *   Direction    | ID    | Meaning (DBC name)   | Notes                       *
 *   -------------+-------+----------------------+---------------------------- *
 *   RX -> Tiva   | 0x100 | VelocityCommand      | from Jetson                 *
 *   RX -> Tiva   | 0x120 | SteeringCommand      | from Jetson                 *
 *   RX -> Tiva   | 0x140 | ResetCommand         | from Jetson (own msg object)*
 *   RX -> Tiva   | 0x7A0 | NodePingRequest      | from HOST (own msg object)  *
 *   TX -> HOST   | 0x7A1 | NodePingRespTiva     | echo, unconditional         *
 *   TX <- Tiva   | 0x200 | VehicleStatus        | LIVE, sent by cluster_comm  *
 *   TX <- Tiva   | 0x110 | VelocityFeedback     | reserved, not yet sent      *
 *   TX <- Tiva   | 0x130 | SteeringFeedback     | reserved, not yet sent      *
 *                                                                             *
 *  RX ids: hardware-accepted by object 2 (mask 0x7DF) - see "WHAT WE RECEIVE".*
 *  TX ids: owned by the DBC (robot.h) / cluster_comm.c - see "WHAT WE SEND".  *
 *******************************************************************************/

/*******************************************************************************
 *                          System / Bitrate                                   *
 *******************************************************************************/
#define CAN_CFG_SYSTEM_CLOCK_HZ     (16000000UL)  /* fsys feeding the CAN core */
#define CAN_CFG_BITRATE_BPS         (500000UL)    /* target 500 kbps           */

/*******************************************************************************
 *                          Bit Timing (500 kbps @ 16 MHz)                     *
 *  Formula (CAN_PERIPHERAL_NOTES.md section 4.3, p1061-1063):                 *
 *    bitrate = fsys / ( BRP * (1 + TSEG1 + TSEG2) )    [functional values]    *
 *  Functional:  BRP=2, TSEG1=11, TSEG2=4, SJW=2                               *
 *    bit = 1 + 11 + 4 = 16 tq ; tq = BRP/fsys = 2/16MHz = 125 ns              *
 *    bit time = 16 * 125 ns = 2.0 us  -> 500 kbps  (exact)                    *
 *    sample point = (1 + TSEG1)/(1+TSEG1+TSEG2) = 12/16 = 75%                 *
 *  Register-encoded = functional - 1 (off-by-one, p1062 Table 17-4):         *
 *    BRP_REG=1, TSEG1_REG=10, TSEG2_REG=3, SJW_REG=1                          *
 *  Composed CANBIT = (3<<12)|(10<<8)|(1<<6)|(1<<0) = 0x3A41                   *
 *  (BRP <= 64 so CANBRPE is NOT needed; left at 0.)                          *
 *******************************************************************************/
#define CAN_CFG_BRP_REG             (1UL)    /* BRP   = 2 - 1  */
#define CAN_CFG_SJW_REG             (1UL)    /* SJW   = 2 - 1  */
#define CAN_CFG_TSEG1_REG           (10UL)   /* TSEG1 = 11 - 1 */
#define CAN_CFG_TSEG2_REG           (3UL)    /* TSEG2 = 4 - 1  */

/*******************************************************************************
 *                          Pin Selection (CAN0)                               *
 *  CAN0Rx = PE4, CAN0Tx = PE5, PMC encoding 8 (Table 17-1, p1049).            *
 *  DECISION 1 = Option B: these pins are muxed by the PORT driver via         *
 *  PORT_PBCFG.c (PORT_PE4_CAN0Rx_MODE / PORT_PE5_CAN0Tx_MODE). The CAN MCAL   *
 *  touches ZERO GPIO registers. The PE4/PE5/PMC values below are kept for     *
 *  documentation/traceability only.                                          *
 *******************************************************************************/
#define CAN_CFG_RX_PIN              (4U)     /* PE4 */
#define CAN_CFG_TX_PIN              (5U)     /* PE5 */
#define CAN_CFG_PIN_PMC             (8U)     /* alternate-function number */

/*******************************************************************************
 *                          Message Objects (hardware assignment)              *
 *  IF1 -> TX object 1, IF2 -> RX object 2. Object numbers are 1..32 (MNUM).    *
 *  Per-direction detail lives in the RX / TX sections below.                  *
 *******************************************************************************/
#define CAN_CFG_NUM_MSG_OBJECTS     (32U)    /* all objects invalidated at init (not RX/TX specific) */

/*******************************************************************************
 *                          WHAT WE RECEIVE (RX acceptance)                    *
 *  Inbound frames from the Jetson. The single RX object (object 2) accepts    *
 *  EXACTLY the id set below and nothing else (see the ID SURFACE map at the   *
 *  top of this file).                                                         *
 *                                                                             *
 *  Accepted-id set - self-documenting, one named constant per id. The         *
 *  literals are INTENTIONALLY duplicated from the DBC (comment cross-refs the  *
 *  ROBOT_* name); the MCAL is transport-only and must NOT #include robot.h.   *
 *******************************************************************************/
#define CAN_CFG_RX_ACCEPT_VELOCITY_CMD_ID  (0x100U)  /* = ROBOT_VELOCITY_COMMAND_FRAME_ID (DBC), from Jetson */
#define CAN_CFG_RX_ACCEPT_STEERING_CMD_ID  (0x120U)  /* = ROBOT_STEERING_COMMAND_FRAME_ID (DBC), from Jetson */

/* RX message object number (IF2 is used for RX exclusively). */
#define CAN_CFG_RX_MSG_OBJ          (2U)     /* RX uses message object 2 (TX uses object 1) */

/*-----------------------------------------------------------------------------
 *  SECOND RX OBJECT - the node liveness ping (0x7A0), added 2026-08-05.
 *
 *  ⚠️ WHY A DEDICATED OBJECT AND NOT A WIDER MASK. The command filter below is
 *  mask 0x7DF / base 0x100, which accepts EXACTLY {0x100, 0x120}. A single
 *  11-bit mask CANNOT be stretched to also admit 0x7A0: 0x7A0's high bits differ
 *  completely from 0x100/0x120 (0x7A0 & 0x7DF = 0x780, nowhere near 0x100), so
 *  any mask loose enough to catch all three would have to make most of the high
 *  bits don't-care and would drag in a wide swathe of unrelated ids. Widening it
 *  would trade a tight, provable command filter for a sloppy one.
 *
 *  So the ping gets its own message object with an EXACT-match filter. The
 *  TM4C123 CAN has 32 message objects and this driver uses three (1=TX, 2=RX
 *  commands, 3=RX ping), so the cost is nil. The two filters are provably
 *  disjoint - no frame can land in both:
 *      0x7A0 vs object 2: (0x7A0 & 0x7DF) = 0x780 != 0x100  -> rejected
 *      0x100 vs object 3: exact match against 0x7A0         -> rejected
 *      0x120 vs object 3: exact match against 0x7A0         -> rejected
 *  Both objects push into the SAME RX ring buffer, so Can_Receive stays the one
 *  drain point and the id in the frame tells the dispatch what arrived.
 *---------------------------------------------------------------------------*/
#define CAN_CFG_RX_PING_MSG_OBJ     (3U)     /* dedicated object for 0x7A0     */
#define CAN_CFG_RX_ACCEPT_PING_ID   (0x7A0U) /* = ROBOT_NODE_PING_REQUEST_FRAME_ID */
#define CAN_CFG_RX_PING_ID_MASK     (0x7FFU) /* 0x7FF = exact match, nothing else */

/*-----------------------------------------------------------------------------
 *  THIRD RX OBJECT - ResetCommand (0x140), added 2026-08-05.
 *
 *  ⚠️ 0x140 IS *NOT* ADMITTED BY THE COMMAND FILTER, despite being a Jetson
 *  command like 0x100/0x120: (0x140 & 0x7DF) = 0x140 != 0x100. That is why the
 *  frame has been defined in the DBC since the beginning yet never once reached
 *  software - it was rejected in hardware.
 *
 *  Widening the command mask WOULD have worked here (0x79F admits 0x100, 0x120,
 *  0x140 and 0x160), unlike the 0x7A0 case where no mask could. It was still
 *  rejected: it would loosen a filter that is currently provably exactly two
 *  ids into one that admits four, including 0x160 - one of OUR OWN transmit ids.
 *  An exact-match object costs one of 32 message objects and keeps every
 *  accepted id explicit and individually justified. Same reasoning as obj 3.
 *---------------------------------------------------------------------------*/
#define CAN_CFG_RX_RESET_MSG_OBJ    (4U)     /* dedicated object for 0x140     */
#define CAN_CFG_RX_ACCEPT_RESET_ID  (0x140U) /* = ROBOT_RESET_COMMAND_FRAME_ID */
#define CAN_CFG_RX_RESET_ID_MASK    (0x7FFU) /* 0x7FF = exact match            */

/* --- Filter mechanism (the values src/can.c Can_ConfigRxObject consumes) -----
 * Hardware acceptance filter, standard 11-bit:
 *   accept iff (rx_id & CAN_CFG_RX_ID_MASK) == (CAN_CFG_RX_ACCEPT_ID & CAN_CFG_RX_ID_MASK).
 * These two macro NAMES are what can.c reads - keep them defined. The base id
 * is sourced from the VelocityCommand accept id above (single source of truth).
 * ---------------------------------------------------------------------------*/
#define CAN_CFG_RX_ACCEPT_ID        (CAN_CFG_RX_ACCEPT_VELOCITY_CMD_ID)  /* base id -> 0x100 */
#define CAN_CFG_RX_ID_MASK          (0x7DFU)  /* mask 0x7DF => accepts exactly {0x100 VelocityCommand, 0x120 SteeringCommand}, nothing else (bit 5 don't-care; every other bit significant). 0x7FF = exact match; 0x000 = accept-all. */

/*******************************************************************************
 *                          WHAT WE SEND (TX)                                  *
 *  Outbound frames from the Tiva. The TX id itself is owned by the DBC        *
 *  (robot.h) and the cluster_comm layer - it is NOT redefined in this MCAL.   *
 *  Do NOT add a TX id constant here (keeps the transport free of message      *
 *  meaning).                                                                  *
 *                                                                             *
 *  TX id set (documentation only):                                            *
 *    0x200 VehicleStatus    - LIVE, sent by cluster_comm.c (ROBOT_VEHICLE_STATUS_FRAME_ID)
 *    0x110 VelocityFeedback - reserved, not yet transmitted                   *
 *    0x130 SteeringFeedback - reserved, not yet transmitted                   *
 *******************************************************************************/
#define CAN_CFG_TX_MSG_OBJ          (1U)     /* TX uses message object 1 (highest prio) */

/* C6-2: how many consecutive Can_Transmit calls may be refused while a frame is
 * still in flight before that frame is abandoned and the object overwritten.
 * main.c transmits at most one frame per millisecond, so 3 refusals means we
 * wait up to ~3 skipped TX slots (~15 ms given the 0/5-mod-10 slot spacing)
 * before giving up. A 500 kbps 8-byte frame needs ~250 us on an idle bus, so
 * this is ~an order of magnitude of patience; the escape exists only so a frame
 * that can NEVER complete (no ACK partner, or a missed TX interrupt) cannot
 * wedge transmission forever. */
#define CAN_CFG_TX_INFLIGHT_LIMIT   (3U)

/*******************************************************************************
 *                          TX completion (bring-up polling)                   *
 *  NOW UNUSED: the blocking TXRQST poll was removed when TX/RX became          *
 *  interrupt-driven; completion is signalled by the ISR. Kept for reference.  *
 *******************************************************************************/
#define CAN_CFG_TX_POLL_TIMEOUT     (1000000UL)

/*******************************************************************************
 *                          Interrupt configuration                            *
 *  CAN0 NVIC interrupt priority (0 = highest .. 7 = lowest; 3 implemented bits *
 *  placed in PRI9 INTD field bits 31:29, p152).                               *
 *******************************************************************************/
/*  ⚠️ CHANGED 1 -> 5 AT B6 (BUDGET-2). THIS IS A FreeRTOS CORRECTNESS           *
 *  REQUIREMENT, NOT A TUNING CHOICE.                                           *
 *                                                                              *
 *  The TM4C123 implements 3 NVIC priority bits (8 levels, 0 = most urgent), and *
 *  FreeRTOSConfig.h sets configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5. That *
 *  splits the interrupt space in two:                                          *
 *     0..4  never masked by the kernel, and therefore FORBIDDEN from calling    *
 *           ANY FreeRTOS "...FromISR" API;                                      *
 *     5..7  maskable, and permitted to call it.                                 *
 *                                                                              *
 *  From B6 the CAN0 ISR calls xSemaphoreGiveFromISR() on TX-complete (the       *
 *  signal tCanTx blocks on). At priority 1 that is illegal, and FreeRTOS's      *
 *  vPortValidateInterruptPriority() would configASSERT on the FIRST TRANSMITTED *
 *  FRAME. B8 will add xQueueSendFromISR on the RX path for the same reason.     *
 *                                                                              *
 *  5 is the most urgent legal value, so this keeps CAN as responsive as the     *
 *  kernel permits. COST: the ISR can now be delayed by a kernel critical        *
 *  section - worst case ~4 us, which is 1.8 % of one 222 us CAN frame at        *
 *  500 kbps. Negligible, and the controller has its own hardware FIFO plus the  *
 *  MSGLST overrun flag we already monitor (Can_RxHwLost).                       *
 *                                                                              *
 *  ⚠️ DO NOT RAISE THIS BACK toward 0 to "improve CAN latency" while any        *
 *  ...FromISR call remains in CAN0_Handler. The rule is absolute: an ISR may be *
 *  faster than the kernel OR talk to the kernel, never both.                    *
 *                                                                              *
 *  Non-RTOS builds are unaffected in behaviour - a lower NVIC priority only     *
 *  matters relative to other enabled interrupts, and in those images CAN0 is    *
 *  still the only application interrupt besides UART1 and SysTick.              *
 *******************************************************************************/
#define CAN_CFG_NVIC_PRIORITY       (5U)

#endif /* CAN_CFG_H_ */
