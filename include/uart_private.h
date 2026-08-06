/******************************************************************************
 *
 * Module: UART
 *
 * File Name: uart_private.h
 *
 * Description: Private definitions for TM4C123GH6PM UART driver
 *              Contains register addresses and bit definitions
 *
 ******************************************************************************/

#ifndef UART_PRIVATE_H_
#define UART_PRIVATE_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          UART Base Addresses                                *
 *******************************************************************************/

#define UART0_BASE_ADDR             (0x4000C000UL)
#define UART1_BASE_ADDR             (0x4000D000UL)
#define UART2_BASE_ADDR             (0x4000E000UL)
#define UART3_BASE_ADDR             (0x4000F000UL)
#define UART4_BASE_ADDR             (0x40010000UL)
#define UART5_BASE_ADDR             (0x40011000UL)
#define UART6_BASE_ADDR             (0x40012000UL)
#define UART7_BASE_ADDR             (0x40013000UL)

/*******************************************************************************
 *                          UART Register Offsets                              *
 *******************************************************************************/

#define UART_DR_OFFSET              (0x000)     /* Data Register */
#define UART_RSR_OFFSET             (0x004)     /* Receive Status Register */
#define UART_ECR_OFFSET             (0x004)     /* Error Clear Register */
#define UART_FR_OFFSET              (0x018)     /* Flag Register */
#define UART_ILPR_OFFSET            (0x020)     /* IrDA Low-Power Register */
#define UART_IBRD_OFFSET            (0x024)     /* Integer Baud-Rate Divisor */
#define UART_FBRD_OFFSET            (0x028)     /* Fractional Baud-Rate Divisor */
#define UART_LCRH_OFFSET            (0x02C)     /* Line Control Register */
#define UART_CTL_OFFSET             (0x030)     /* Control Register */
#define UART_IFLS_OFFSET            (0x034)     /* Interrupt FIFO Level Select */
#define UART_IM_OFFSET              (0x038)     /* Interrupt Mask Register */
#define UART_RIS_OFFSET             (0x03C)     /* Raw Interrupt Status */
#define UART_MIS_OFFSET             (0x040)     /* Masked Interrupt Status */
#define UART_ICR_OFFSET             (0x044)     /* Interrupt Clear Register */
#define UART_DMACTL_OFFSET          (0x048)     /* DMA Control Register */
#define UART_CC_OFFSET              (0xFC8)     /* Clock Configuration */

/*******************************************************************************
 *                          Register Access Macros                             *
 *******************************************************************************/

#define UART_REG(base, offset)      (*((volatile uint32 *)((base) + (offset))))

#define UART_DR(base)               UART_REG(base, UART_DR_OFFSET)
#define UART_RSR(base)              UART_REG(base, UART_RSR_OFFSET)
#define UART_ECR(base)              UART_REG(base, UART_ECR_OFFSET)
#define UART_FR(base)               UART_REG(base, UART_FR_OFFSET)
#define UART_IBRD(base)             UART_REG(base, UART_IBRD_OFFSET)
#define UART_FBRD(base)             UART_REG(base, UART_FBRD_OFFSET)
#define UART_LCRH(base)             UART_REG(base, UART_LCRH_OFFSET)
#define UART_CTL(base)              UART_REG(base, UART_CTL_OFFSET)
#define UART_IFLS(base)             UART_REG(base, UART_IFLS_OFFSET)
#define UART_IM(base)               UART_REG(base, UART_IM_OFFSET)
#define UART_RIS(base)              UART_REG(base, UART_RIS_OFFSET)
#define UART_MIS(base)              UART_REG(base, UART_MIS_OFFSET)
#define UART_ICR(base)              UART_REG(base, UART_ICR_OFFSET)
#define UART_DMACTL(base)           UART_REG(base, UART_DMACTL_OFFSET)
#define UART_CC(base)               UART_REG(base, UART_CC_OFFSET)

/*******************************************************************************
 *                          System Control Registers                           *
 *******************************************************************************/

#define SYSCTL_BASE_ADDR            (0x400FE000UL)
#define SYSCTL_RCGCUART             (*((volatile uint32 *)(SYSCTL_BASE_ADDR + 0x618)))
#define SYSCTL_PRUART               (*((volatile uint32 *)(SYSCTL_BASE_ADDR + 0xA18)))
#define SYSCTL_RCGCGPIO             (*((volatile uint32 *)(SYSCTL_BASE_ADDR + 0x608)))
#define SYSCTL_PRGPIO               (*((volatile uint32 *)(SYSCTL_BASE_ADDR + 0xA08)))

/*******************************************************************************
 *                          GPIO Base Addresses                                *
 *******************************************************************************/

#define GPIO_PORTA_BASE             (0x40004000UL)
#define GPIO_PORTB_BASE             (0x40005000UL)
#define GPIO_PORTC_BASE             (0x40006000UL)
#define GPIO_PORTD_BASE             (0x40007000UL)
#define GPIO_PORTE_BASE             (0x40024000UL)
#define GPIO_PORTF_BASE             (0x40025000UL)

/*******************************************************************************
 *                          GPIO Register Offsets                              *
 *******************************************************************************/

#define GPIO_AFSEL_OFFSET           (0x420)
#define GPIO_DEN_OFFSET             (0x51C)
#define GPIO_AMSEL_OFFSET           (0x528)
#define GPIO_PCTL_OFFSET            (0x52C)
#define GPIO_DIR_OFFSET             (0x400)
#define GPIO_LOCK_OFFSET            (0x520)
#define GPIO_CR_OFFSET              (0x524)

#define GPIO_REG(base, offset)      (*((volatile uint32 *)((base) + (offset))))

/*******************************************************************************
 *                          NVIC Registers                                     *
 *******************************************************************************/

#define NVIC_BASE_ADDR              (0xE000E000UL)
#define NVIC_EN0                    (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x100)))
#define NVIC_EN1                    (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x104)))
#define NVIC_PRI1                   (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x404)))
#define NVIC_PRI6                   (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x418)))
#define NVIC_PRI7                   (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x41C)))
#define NVIC_PRI8                   (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x420)))
#define NVIC_PRI12                  (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x430)))
#define NVIC_PRI13                  (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x434)))
#define NVIC_PRI14                  (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x438)))
#define NVIC_PRI15                  (*((volatile uint32 *)(NVIC_BASE_ADDR + 0x43C)))

/* UART Interrupt Numbers */
#define UART0_IRQ_NUM               (5)
#define UART1_IRQ_NUM               (6)
#define UART2_IRQ_NUM               (33)
#define UART3_IRQ_NUM               (59)
#define UART4_IRQ_NUM               (60)
#define UART5_IRQ_NUM               (61)
#define UART6_IRQ_NUM               (62)
#define UART7_IRQ_NUM               (63)

/*******************************************************************************
 *                          UART Flag Register Bits                            *
 *******************************************************************************/

#define UART_FR_BUSY                (1U << 3)   /* UART Busy */
#define UART_FR_RXFE                (1U << 4)   /* RX FIFO Empty */
#define UART_FR_TXFF                (1U << 5)   /* TX FIFO Full */
#define UART_FR_RXFF                (1U << 6)   /* RX FIFO Full */
#define UART_FR_TXFE                (1U << 7)   /* TX FIFO Empty */

/*******************************************************************************
 *                          UART Line Control Register Bits                    *
 *******************************************************************************/

#define UART_LCRH_BRK               (1U << 0)   /* Send Break */
#define UART_LCRH_PEN               (1U << 1)   /* Parity Enable */
#define UART_LCRH_EPS               (1U << 2)   /* Even Parity Select */
#define UART_LCRH_STP2              (1U << 3)   /* Two Stop Bits Select */
#define UART_LCRH_FEN               (1U << 4)   /* FIFO Enable */
#define UART_LCRH_WLEN_POS          (5U)        /* Word Length Position */
#define UART_LCRH_WLEN_MASK         (0x3U << 5) /* Word Length Mask */
#define UART_LCRH_SPS               (1U << 7)   /* Stick Parity Select */

/*******************************************************************************
 *                          UART Control Register Bits                         *
 *******************************************************************************/

#define UART_CTL_UARTEN             (1U << 0)   /* UART Enable */
#define UART_CTL_SIREN              (1U << 1)   /* SIR Enable */
#define UART_CTL_SIRLP              (1U << 2)   /* SIR Low-Power Mode */
#define UART_CTL_LBE                (1U << 7)   /* Loopback Enable */
#define UART_CTL_TXE                (1U << 8)   /* Transmit Enable */
#define UART_CTL_RXE                (1U << 9)   /* Receive Enable */
#define UART_CTL_RTS                (1U << 11)  /* Request to Send */
#define UART_CTL_EOT                (1U << 4)   /* End of Transmission */

/*******************************************************************************
 *                          UART Interrupt Bits                                *
 *******************************************************************************/

#define UART_INT_OE                 (1U << 10)  /* Overrun Error */
#define UART_INT_BE                 (1U << 9)   /* Break Error */
#define UART_INT_PE                 (1U << 8)   /* Parity Error */
#define UART_INT_FE                 (1U << 7)   /* Framing Error */
#define UART_INT_RT                 (1U << 6)   /* Receive Timeout */
#define UART_INT_TX                 (1U << 5)   /* Transmit */
#define UART_INT_RX                 (1U << 4)   /* Receive */

#define UART_INT_ALL_ERRORS         (UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE)

/*******************************************************************************
 *                          UART RSR/ECR Bits                                  *
 *******************************************************************************/

#define UART_RSR_FE                 (1U << 0)   /* Framing Error */
#define UART_RSR_PE                 (1U << 1)   /* Parity Error */
#define UART_RSR_BE                 (1U << 2)   /* Break Error */
#define UART_RSR_OE                 (1U << 3)   /* Overrun Error */

/*******************************************************************************
 *                          UART FIFO Level Select                             *
 *******************************************************************************/

#define UART_IFLS_TX_1_8            (0x0U << 0) /* TX FIFO <= 1/8 full */
#define UART_IFLS_TX_1_4            (0x1U << 0) /* TX FIFO <= 1/4 full */
#define UART_IFLS_TX_1_2            (0x2U << 0) /* TX FIFO <= 1/2 full */
#define UART_IFLS_TX_3_4            (0x3U << 0) /* TX FIFO <= 3/4 full */
#define UART_IFLS_TX_7_8            (0x4U << 0) /* TX FIFO <= 7/8 full */

#define UART_IFLS_RX_1_8            (0x0U << 3) /* RX FIFO >= 1/8 full */
#define UART_IFLS_RX_1_4            (0x1U << 3) /* RX FIFO >= 1/4 full */
#define UART_IFLS_RX_1_2            (0x2U << 3) /* RX FIFO >= 1/2 full */
#define UART_IFLS_RX_3_4            (0x3U << 3) /* RX FIFO >= 3/4 full */
#define UART_IFLS_RX_7_8            (0x4U << 3) /* RX FIFO >= 7/8 full */

/*******************************************************************************
 *                          GPIO PCTL Values for UART                          *
 *******************************************************************************/

#define GPIO_PCTL_UART              (0x1U)

/*******************************************************************************
 *                          UART Pin Mapping                                   *
 *******************************************************************************/

/*
 * UART0: PA0 (U0RX), PA1 (U0TX)
 * UART1: PB0 (U1RX), PB1 (U1TX) - Also PC4/PC5 alternate
 * UART2: PD6 (U2RX), PD7 (U2TX)
 * UART3: PC6 (U3RX), PC7 (U3TX)
 * UART4: PC4 (U4RX), PC5 (U4TX)
 * UART5: PE4 (U5RX), PE5 (U5TX)
 * UART6: PD4 (U6RX), PD5 (U6TX)
 * UART7: PE0 (U7RX), PE1 (U7TX)
 */

/* Pin masks */
#define UART0_RX_PIN                (1U << 0)   /* PA0 */
#define UART0_TX_PIN                (1U << 1)   /* PA1 */
#define UART1_RX_PIN                (1U << 0)   /* PB0 */
#define UART1_TX_PIN                (1U << 1)   /* PB1 */
#define UART2_RX_PIN                (1U << 6)   /* PD6 */
#define UART2_TX_PIN                (1U << 7)   /* PD7 */
#define UART3_RX_PIN                (1U << 6)   /* PC6 */
#define UART3_TX_PIN                (1U << 7)   /* PC7 */
#define UART4_RX_PIN                (1U << 4)   /* PC4 */
#define UART4_TX_PIN                (1U << 5)   /* PC5 */
#define UART5_RX_PIN                (1U << 4)   /* PE4 */
#define UART5_TX_PIN                (1U << 5)   /* PE5 */
#define UART6_RX_PIN                (1U << 4)   /* PD4 */
#define UART6_TX_PIN                (1U << 5)   /* PD5 */
#define UART7_RX_PIN                (1U << 0)   /* PE0 */
#define UART7_TX_PIN                (1U << 1)   /* PE1 */

/*******************************************************************************
 *                          Bounded-wait caps (U23-2)                          *
 *
 * Same bounded-spin pattern as TIMER0_INIT_SPIN_CAP and CAN_PRIV_IF_BUSY_SPIN_CAP:
 * no runtime spin in this driver may be unbounded, and none may approach the
 * 1 ms CAN-TX slot that src/main.c's super-loop depends on.
 *
 * UART_PRIV_TXFF_SPIN_CAP - cap on UART_SendBuffer's PER-BYTE back-pressure loop
 *   (both the software TX buffer AND the 16-deep hardware FIFO full at once).
 *   Sizing, at the real 16 MHz core clock (C6-4) and 115200 baud:
 *     - what the loop must outlast is ONE hardware FIFO slot freeing up, i.e.
 *       exactly one byte-time = ~86.8 us (NOT the ~44 ms it takes to drain a
 *       whole 512-byte buffer - the loop never has to wait for that);
 *     - one iteration is 25 instructions in the -O0 build (a failed
 *       CircularBuffer_Put: PRIMASK save/mask, count-vs-size compare, PRIMASK
 *       restore; plus one UART_FR read and the loop overhead), counted from the
 *       emitted disassembly of UART_SendBuffer - about 40 core cycles;
 *     - 128 iterations therefore bound the per-byte spin at ~0.32 ms, which is
 *       ~3.7 byte-times of drain tolerance (comfortable margin over the single
 *       byte-time actually needed) while staying ~3x inside main.c's 1 ms
 *       CAN-TX slot.
 *   MEASURED ON SILICON, not estimated (FIX 23 bench, 2026-08-03): with TX
 *   deliberately wedged (UART1 CTL.TXE cleared over the debug probe, so the FIFO
 *   can never drain) the drop counter advanced 4702 in 1500 ms and 4703 in the
 *   next 1500 ms = 319.0 us per cap expiry, both intervals agreeing to 0.02 %.
 *   So the real worst-case per-byte block is 319 us, and the CPU kept making
 *   progress throughout - the pre-fix loop hung forever in exactly this state.
 *   LIMITATION, recorded deliberately: this bounds the spin PER BYTE, not per
 *   UART_SendBuffer call. A caller that hands the driver more bytes than the
 *   115200 link can carry (e.g. staircase_test's 40 KB CSV dump, which is a
 *   tight unthrottled loop) still blocks for ~87 us per byte - that is the
 *   caller's data rate, not a driver defect, and capping the whole CALL would
 *   truncate integers mid-digit and destroy the very data the bench exists to
 *   collect. What the cap removes is the UNBOUNDED case: a wedged TX (TXE
 *   cleared, clock gated, line held) can no longer hang the super-loop forever.
 *
 * UART_PRIV_FR_BUSY_SPIN_CAP - cap on UART_SetBaudRate's FR_BUSY wait, which
 *   legitimately has to outlast the FIFO (16) plus the shift register draining
 *   at the OLD baud rate: ~17 byte-times ~= 1.5 ms at 115200, worse at lower
 *   baud rates. This is a reconfiguration call, not a per-loop path, so it gets
 *   a boot-class cap ~20x that budget rather than the 1 ms slot budget.
 *******************************************************************************/

#define UART_PRIV_TXFF_SPIN_CAP     (128UL)
#define UART_PRIV_FR_BUSY_SPIN_CAP  (100000UL)

/*******************************************************************************
 *                          Helper Macros                                      *
 *******************************************************************************/

#define UART_GET_BASE_ADDR(id)      (UART0_BASE_ADDR + ((id) * 0x1000))

/* Unlock key for GPIO */
#define GPIO_LOCK_KEY               (0x4C4F434B)

#endif /* UART_PRIVATE_H_ */
