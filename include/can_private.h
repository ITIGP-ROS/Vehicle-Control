/******************************************************************************
 *
 * Module: CAN (MCAL)
 *
 * File Name: can_private.h
 *
 * Description: Private register addresses and bit/field masks for the CAN0
 *              controller. Included by can.c only. CAN0 peripheral register
 *              addresses live here (base 0x40040000 + offsets, per
 *              CAN_PERIPHERAL_NOTES.md Table 17-5, p1066). The SYSCTL clock/
 *              ready and GPIO Port E registers are reused from
 *              tm4c123gh6pm_registers.h (already defined there).
 *
 ******************************************************************************/

#ifndef CAN_PRIVATE_H_
#define CAN_PRIVATE_H_

#include "Platform_Types.h"

/*******************************************************************************
 *                  CAN0 Module Register Addresses                             *
 *  Base 0x4004.0000 (CAN_PERIPHERAL_NOTES.md section 3, p1066).               *
 *******************************************************************************/
#define CAN0_BASE                 (0x40040000UL)

#define CAN0_CTL_REG              (*((volatile uint32 *)(CAN0_BASE + 0x000)))  /* CANCTL    p1069 */
#define CAN0_STS_REG              (*((volatile uint32 *)(CAN0_BASE + 0x004)))  /* CANSTS    p1071 */
#define CAN0_ERR_REG              (*((volatile uint32 *)(CAN0_BASE + 0x008)))  /* CANERR RO p1074 */
#define CAN0_BIT_REG              (*((volatile uint32 *)(CAN0_BASE + 0x00C)))  /* CANBIT    p1075 */
#define CAN0_INT_REG              (*((volatile uint32 *)(CAN0_BASE + 0x010)))  /* CANINT RO p1076 */
#define CAN0_TST_REG              (*((volatile uint32 *)(CAN0_BASE + 0x014)))  /* CANTST    p1077 */
#define CAN0_BRPE_REG             (*((volatile uint32 *)(CAN0_BASE + 0x018)))  /* CANBRPE   p1079 */

/* --- Interface 1 (used for TX, see CAN_PERIPHERAL_NOTES.md gotchas) --- */
#define CAN0_IF1CRQ_REG           (*((volatile uint32 *)(CAN0_BASE + 0x020)))  /* p1080 */
#define CAN0_IF1CMSK_REG          (*((volatile uint32 *)(CAN0_BASE + 0x024)))  /* p1081 */
#define CAN0_IF1MSK1_REG          (*((volatile uint32 *)(CAN0_BASE + 0x028)))  /* p1084 */
#define CAN0_IF1MSK2_REG          (*((volatile uint32 *)(CAN0_BASE + 0x02C)))  /* p1085 */
#define CAN0_IF1ARB1_REG          (*((volatile uint32 *)(CAN0_BASE + 0x030)))  /* p1087 */
#define CAN0_IF1ARB2_REG          (*((volatile uint32 *)(CAN0_BASE + 0x034)))  /* p1088 */
#define CAN0_IF1MCTL_REG          (*((volatile uint32 *)(CAN0_BASE + 0x038)))  /* p1090 */
#define CAN0_IF1DA1_REG           (*((volatile uint32 *)(CAN0_BASE + 0x03C)))  /* bytes 0,1 p1093 */
#define CAN0_IF1DA2_REG           (*((volatile uint32 *)(CAN0_BASE + 0x040)))  /* bytes 2,3 p1093 */
#define CAN0_IF1DB1_REG           (*((volatile uint32 *)(CAN0_BASE + 0x044)))  /* bytes 4,5 p1093 */
#define CAN0_IF1DB2_REG           (*((volatile uint32 *)(CAN0_BASE + 0x048)))  /* bytes 6,7 p1093 */

/* --- Interface 2 (used for RX exclusively) --- */
#define CAN0_IF2CRQ_REG           (*((volatile uint32 *)(CAN0_BASE + 0x080)))  /* p1080 */
#define CAN0_IF2CMSK_REG          (*((volatile uint32 *)(CAN0_BASE + 0x084)))  /* p1081 */
#define CAN0_IF2MSK1_REG          (*((volatile uint32 *)(CAN0_BASE + 0x088)))  /* p1084 */
#define CAN0_IF2MSK2_REG          (*((volatile uint32 *)(CAN0_BASE + 0x08C)))  /* p1085 */
#define CAN0_IF2ARB1_REG          (*((volatile uint32 *)(CAN0_BASE + 0x090)))  /* p1087 */
#define CAN0_IF2ARB2_REG          (*((volatile uint32 *)(CAN0_BASE + 0x094)))  /* p1088 */
#define CAN0_IF2MCTL_REG          (*((volatile uint32 *)(CAN0_BASE + 0x098)))  /* p1090 */
#define CAN0_IF2DA1_REG           (*((volatile uint32 *)(CAN0_BASE + 0x09C)))  /* bytes 0,1 p1093 */
#define CAN0_IF2DA2_REG           (*((volatile uint32 *)(CAN0_BASE + 0x0A0)))  /* bytes 2,3 p1093 */
#define CAN0_IF2DB1_REG           (*((volatile uint32 *)(CAN0_BASE + 0x0A4)))  /* bytes 4,5 p1093 */
#define CAN0_IF2DB2_REG           (*((volatile uint32 *)(CAN0_BASE + 0x0A8)))  /* bytes 6,7 p1093 */

/* --- Status arrays (RO) --- */
#define CAN0_TXRQ1_REG            (*((volatile uint32 *)(CAN0_BASE + 0x100)))  /* CANTXRQ1 p1094 */
#define CAN0_NWDA1_REG            (*((volatile uint32 *)(CAN0_BASE + 0x120)))  /* CANNWDA1 p1095 (RX) */

/*******************************************************************************
 *                  CANCTL bits (offset 0x000, p1069)                          *
 *******************************************************************************/
#define CAN_PRIV_CTL_INIT_POS     (0U)   /* Initialization              */
#define CAN_PRIV_CTL_INIT_MASK    (1UL << CAN_PRIV_CTL_INIT_POS)
#define CAN_PRIV_CTL_IE_POS       (1U)   /* CAN Interrupt Enable        */
#define CAN_PRIV_CTL_IE_MASK      (1UL << CAN_PRIV_CTL_IE_POS)
#define CAN_PRIV_CTL_SIE_POS      (2U)   /* Status Interrupt Enable     */
#define CAN_PRIV_CTL_SIE_MASK     (1UL << CAN_PRIV_CTL_SIE_POS)
#define CAN_PRIV_CTL_EIE_POS      (3U)   /* Error Interrupt Enable      */
#define CAN_PRIV_CTL_EIE_MASK     (1UL << CAN_PRIV_CTL_EIE_POS)
#define CAN_PRIV_CTL_CCE_POS      (6U)   /* Configuration Change Enable */
#define CAN_PRIV_CTL_CCE_MASK     (1UL << CAN_PRIV_CTL_CCE_POS)

/*******************************************************************************
 *                  CANSTS bits (offset 0x004, p1071-1073)                     *
 *  LEC (2:0) = last error code; BOFF (7) = latched bus-off.                    *
 *  Field types (p1071): BOFF/EWARN/EPASS are RO (writes ignored); RXOK/TXOK/   *
 *  LEC are RW. The unused LEC code 0x7 ("No Event") may be written by the CPU  *
 *  to arm the field so a later read unambiguously reflects THIS attempt.       *
 *******************************************************************************/
#define CAN_PRIV_STS_LEC_MASK     (0x7UL)        /* 2:0 Last Error Code         */
#define CAN_PRIV_STS_LEC_NO_ERROR (0x0UL)        /* 0 no error (cleared on success) */
#define CAN_PRIV_STS_LEC_STUFF    (0x1UL)        /* 1 stuff error               */
#define CAN_PRIV_STS_LEC_FORM     (0x2UL)        /* 2 form/format error         */
#define CAN_PRIV_STS_LEC_ACK      (0x3UL)        /* 3 ACK error (no acknowledge)*/
#define CAN_PRIV_STS_LEC_BIT1     (0x4UL)        /* 4 bit-1 error               */
#define CAN_PRIV_STS_LEC_BIT0     (0x5UL)        /* 5 bit-0 error               */
#define CAN_PRIV_STS_LEC_CRC      (0x6UL)        /* 6 CRC error                 */
#define CAN_PRIV_STS_LEC_NO_EVENT (0x7UL)        /* 7 no event since write (arm)*/
#define CAN_PRIV_STS_TXOK_MASK    (1UL << 3)     /* transmitted OK (RW, sticky) */
#define CAN_PRIV_STS_RXOK_MASK    (1UL << 4)     /* received OK    (RW, sticky) */
#define CAN_PRIV_STS_EPASS_MASK   (1UL << 5)     /* error passive  (RO)         */
#define CAN_PRIV_STS_EWARN_MASK   (1UL << 6)     /* warning        (RO)         */
#define CAN_PRIV_STS_BOFF_POS     (7U)           /* bus-off        (RO)         */
#define CAN_PRIV_STS_BOFF_MASK    (1UL << CAN_PRIV_STS_BOFF_POS)

/*******************************************************************************
 *                  CANBIT fields (offset 0x00C, p1075)                        *
 *  NOTE: all four store (functional value - 1); see can_cfg.h.                *
 *******************************************************************************/
#define CAN_PRIV_BIT_BRP_POS      (0U)   /* 5:0  Baud Rate Prescaler        */
#define CAN_PRIV_BIT_SJW_POS      (6U)   /* 7:6  (Re)Sync Jump Width        */
#define CAN_PRIV_BIT_TSEG1_POS    (8U)   /* 11:8 Time Segment before sample */
#define CAN_PRIV_BIT_TSEG2_POS    (12U)  /* 14:12 Time Segment after sample */

/*******************************************************************************
 *                  CANIFnCRQ (offset 0x020, p1080)                            *
 *******************************************************************************/
#define CAN_PRIV_CRQ_MNUM_MASK    (0x3FUL)        /* 5:0 message number 1..32 */
#define CAN_PRIV_CRQ_BUSY_POS     (15U)           /* transfer in progress     */
#define CAN_PRIV_CRQ_BUSY_MASK    (1UL << CAN_PRIV_CRQ_BUSY_POS)

/*******************************************************************************
 *                  CANIFnCMSK (offset 0x024, p1081)                           *
 *******************************************************************************/
#define CAN_PRIV_CMSK_DATAB_MASK     (1UL << 0)
#define CAN_PRIV_CMSK_DATAA_MASK     (1UL << 1)
#define CAN_PRIV_CMSK_NEWDAT_MASK    (1UL << 2)   /* NEWDAT/TXRQST access     */
#define CAN_PRIV_CMSK_CLRINTPND_MASK (1UL << 3)
#define CAN_PRIV_CMSK_CONTROL_MASK   (1UL << 4)   /* transfer MCTL            */
#define CAN_PRIV_CMSK_ARB_MASK       (1UL << 5)   /* transfer ARB1/ARB2       */
#define CAN_PRIV_CMSK_MASK_MASK      (1UL << 6)   /* transfer MSK1/MSK2       */
#define CAN_PRIV_CMSK_WRNRD_MASK     (1UL << 7)   /* 1 = write IFn -> RAM     */

/* Composite for a TX write: WRNRD | ARB | CONTROL | DATAA | DATAB = 0xB3 */
#define CAN_PRIV_CMSK_TX_WRITE    (CAN_PRIV_CMSK_WRNRD_MASK | CAN_PRIV_CMSK_ARB_MASK | \
                                   CAN_PRIV_CMSK_CONTROL_MASK | CAN_PRIV_CMSK_DATAA_MASK | \
                                   CAN_PRIV_CMSK_DATAB_MASK)

/*******************************************************************************
 *                  CANIFnARB2 (offset 0x034, p1088)                           *
 *  Standard 11-bit: ID[10:0] occupies bits 12:2 of ARB2.                      *
 *******************************************************************************/
#define CAN_PRIV_ARB2_ID_STD_POS  (2U)            /* 11-bit ID left-justified */
#define CAN_PRIV_ARB2_DIR_POS     (13U)           /* 1 = transmit             */
#define CAN_PRIV_ARB2_DIR_MASK    (1UL << CAN_PRIV_ARB2_DIR_POS)
#define CAN_PRIV_ARB2_XTD_POS     (14U)           /* 0 = standard 11-bit      */
#define CAN_PRIV_ARB2_XTD_MASK    (1UL << CAN_PRIV_ARB2_XTD_POS)
#define CAN_PRIV_ARB2_MSGVAL_POS  (15U)           /* 1 = object valid         */
#define CAN_PRIV_ARB2_MSGVAL_MASK (1UL << CAN_PRIV_ARB2_MSGVAL_POS)

/*******************************************************************************
 *                  CANIFnMCTL (offset 0x038, p1090)                           *
 *******************************************************************************/
#define CAN_PRIV_MCTL_DLC_MASK    (0x0FUL)        /* 3:0 data length          */
#define CAN_PRIV_MCTL_EOB_MASK    (1UL << 7)      /* End Of Buffer (single)   */
#define CAN_PRIV_MCTL_TXRQST_MASK (1UL << 8)      /* Transmit Request         */
#define CAN_PRIV_MCTL_RXIE_MASK   (1UL << 10)     /* RX interrupt enable (ISR, later) */
#define CAN_PRIV_MCTL_TXIE_MASK   (1UL << 11)     /* TX interrupt enable (ISR, later) */
#define CAN_PRIV_MCTL_UMASK_MASK  (1UL << 12)     /* Use Acceptance Mask      */
#define CAN_PRIV_MCTL_MSGLST_MASK (1UL << 14)     /* Message Lost (overrun)   */

/* C6-3: iteration cap for the IFnCRQ BUSY waits, which run in CAN0_Handler.
 * BUSY reflects an IF<->message-RAM transfer that completes in a handful of
 * peripheral clocks (well under 1 us), so any real wait is a few iterations.
 * 10000 is ~3 orders of magnitude of headroom yet still bounds the worst case
 * to a few ms, so a stuck BUSY bit degrades ISR latency instead of wedging the
 * highest-priority context forever. Same spirit as TIMER0_INIT_SPIN_CAP. */
#define CAN_PRIV_IF_BUSY_SPIN_CAP (10000UL)
#define CAN_PRIV_MCTL_NEWDAT_MASK (1UL << 15)     /* New Data                 */

/*******************************************************************************
 *                  CANIFnMSK2 acceptance-filter bits (offset 0x02C/0x08C, p1085)*
 *  11-bit ID mask occupies MSK[12:2] - same left-justified position as ARB2.   *
 *******************************************************************************/
#define CAN_PRIV_MSK2_MSK_STD_POS (2U)            /* 11-bit ID mask left-justified */
#define CAN_PRIV_MSK2_MDIR_MASK   (1UL << 14)     /* 1 = filter on DIR             */
#define CAN_PRIV_MSK2_MXTD_MASK   (1UL << 15)     /* 1 = filter on XTD (std only)  */

/*******************************************************************************
 *                  CANIFnCMSK composites for RX (p1081)                       *
 *  CONFIG: write IF2 -> RAM, transfer MASK + ARB + CONTROL (no data).   = 0xF0 *
 *  READ  : read RAM -> IF2 (WRNRD=0), transfer MASK+ARB+CONTROL+CLRINTPND+     *
 *          NEWDAT+DATAA+DATAB; NEWDAT+CLRINTPND clear those bits in RAM. = 0x7F *
 *  LSTCLR: write IF2 -> RAM, CONTROL only (write MCTL back to clear MSGLST).=0x90*
 *******************************************************************************/
#define CAN_PRIV_CMSK_RX_CONFIG  (CAN_PRIV_CMSK_WRNRD_MASK | CAN_PRIV_CMSK_MASK_MASK | \
                                  CAN_PRIV_CMSK_ARB_MASK   | CAN_PRIV_CMSK_CONTROL_MASK)
#define CAN_PRIV_CMSK_RX_READ    (CAN_PRIV_CMSK_MASK_MASK  | CAN_PRIV_CMSK_ARB_MASK | \
                                  CAN_PRIV_CMSK_CONTROL_MASK | CAN_PRIV_CMSK_CLRINTPND_MASK | \
                                  CAN_PRIV_CMSK_NEWDAT_MASK | CAN_PRIV_CMSK_DATAA_MASK | \
                                  CAN_PRIV_CMSK_DATAB_MASK)
#define CAN_PRIV_CMSK_RX_LSTCLR  (CAN_PRIV_CMSK_WRNRD_MASK | CAN_PRIV_CMSK_CONTROL_MASK)

/*******************************************************************************
 *                  CANINT INTID encoding (offset 0x010, p1076)                *
 *  CAN_INTERRUPT_NOTES.md section 3. A message-object source reads back as the *
 *  object number itself (1..32), compared against the cfg object numbers.      *
 *******************************************************************************/
#define CAN_PRIV_INT_NONE         (0x0000UL)     /* no interrupt pending          */
#define CAN_PRIV_INT_STATUS       (0x8000UL)     /* status interrupt              */

/* CANIFnMCTL: INTPND (bit 13) - per-object interrupt-pending flag (p1090). */
#define CAN_PRIV_MCTL_INTPND_MASK (1UL << 13)

/* CMSK to acknowledge a message-object interrupt: a read (WRNRD=0) with
 * CLRINTPND set clears the object's INTPND in RAM (p1059/p1081). Used for the TX
 * object; the RX read-out (CAN_PRIV_CMSK_RX_READ) already includes CLRINTPND. */
#define CAN_PRIV_CMSK_TX_INTACK  (CAN_PRIV_CMSK_CLRINTPND_MASK)

/*******************************************************************************
 *                  NVIC registers for CAN0 = IRQ 39 (Cortex-M4 core)          *
 *  Defined here (not in tm4c123gh6pm_registers.h, which is off-limits and      *
 *  lacks EN1/PRI9), mirroring uart_private.h's local NVIC defs.                 *
 *  CAN0: Vector 55 / IRQ 39 (Table 2-9, p104). IRQ 39 -> EN1/PEND1 bit 7,       *
 *  PRI9 field INTD bits 31:29 (Reg 5 p142 / Reg 38 p152 / set-pending p157).    *
 *******************************************************************************/
#define CAN_PRIV_NVIC_EN1        (*((volatile uint32 *)0xE000E104))  /* set-enable  32-63 */
#define CAN_PRIV_NVIC_PEND1      (*((volatile uint32 *)0xE000E204))  /* set-pending 32-63 */
#define CAN_PRIV_NVIC_PRI9       (*((volatile uint32 *)0xE000E424))  /* priority 36-39    */
#define CAN_PRIV_NVIC_CAN0_BIT   (7U)            /* 39 - 32 -> bit 7 in EN1/PEND1   */
#define CAN_PRIV_NVIC_PRI9_INTD_POS (29U)        /* IRQ 39 = INTD field, bits 31:29 */

#endif /* CAN_PRIVATE_H_ */
