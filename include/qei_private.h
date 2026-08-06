/******************************************************************************
 *
 * Module: QEI (Quadrature Encoder Interface)
 *
 * File Name: qei_private.h
 *
 * Description: Private definitions for TM4C123GH6PM QEI driver
 *              Contains register addresses and bit definitions
 *
 ******************************************************************************/

#ifndef QEI_PRIVATE_H_
#define QEI_PRIVATE_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          QEI Module Base Addresses                          *
 *******************************************************************************/

#define QEI0_BASE_ADDR                  (0x4002C000UL)
#define QEI1_BASE_ADDR                  (0x4002D000UL)

/*******************************************************************************
 *                          QEI Register Offsets                               *
 *******************************************************************************/

#define QEI_CTL_OFFSET                  (0x000)     /* Control */
#define QEI_STAT_OFFSET                 (0x004)     /* Status */
#define QEI_POS_OFFSET                  (0x008)     /* Position */
#define QEI_MAXPOS_OFFSET               (0x00C)     /* Maximum Position */
#define QEI_LOAD_OFFSET                 (0x010)     /* Velocity Timer Load */
#define QEI_TIME_OFFSET                 (0x014)     /* Velocity Timer */
#define QEI_COUNT_OFFSET                (0x018)     /* Velocity Counter */
#define QEI_SPEED_OFFSET                (0x01C)     /* Velocity Capture */
#define QEI_INTEN_OFFSET                (0x020)     /* Interrupt Enable */
#define QEI_RIS_OFFSET                  (0x024)     /* Raw Interrupt Status */
#define QEI_ISC_OFFSET                  (0x028)     /* Interrupt Status and Clear */

/*******************************************************************************
 *                          Register Access Macros                             *
 *******************************************************************************/

#define QEI_REG(base, offset)           (*((volatile uint32 *)((base) + (offset))))

#define QEI_CTL(base)                   QEI_REG(base, QEI_CTL_OFFSET)
#define QEI_STAT(base)                  QEI_REG(base, QEI_STAT_OFFSET)
#define QEI_POS(base)                   QEI_REG(base, QEI_POS_OFFSET)
#define QEI_MAXPOS(base)                QEI_REG(base, QEI_MAXPOS_OFFSET)
#define QEI_LOAD(base)                  QEI_REG(base, QEI_LOAD_OFFSET)
#define QEI_TIME(base)                  QEI_REG(base, QEI_TIME_OFFSET)
#define QEI_COUNT(base)                 QEI_REG(base, QEI_COUNT_OFFSET)
#define QEI_SPEED(base)                 QEI_REG(base, QEI_SPEED_OFFSET)
#define QEI_INTEN(base)                 QEI_REG(base, QEI_INTEN_OFFSET)
#define QEI_RIS(base)                   QEI_REG(base, QEI_RIS_OFFSET)
#define QEI_ISC(base)                   QEI_REG(base, QEI_ISC_OFFSET)

/*******************************************************************************
 *                          System Control Registers                           *
 *******************************************************************************/

#define SYSCTL_BASE_ADDR                (0x400FE000UL)
#define SYSCTL_RCGCQEI                  (*((volatile uint32 *)(SYSCTL_BASE_ADDR + 0x644)))
#define SYSCTL_PRQEI                    (*((volatile uint32 *)(SYSCTL_BASE_ADDR + 0xA44)))

/*******************************************************************************
 *                          QEI Control Register (QEICTL) Bits                 *
 *******************************************************************************/

/* Bit 0: ENABLE - Enable QEI */
#define QEI_CTL_ENABLE_POS              (0U)
#define QEI_CTL_ENABLE_MASK             (1UL << QEI_CTL_ENABLE_POS)

/* Bit 1: SWAP - Swap Signals PhA and PhB */
#define QEI_CTL_SWAP_POS                (1U)
#define QEI_CTL_SWAP_MASK               (1UL << QEI_CTL_SWAP_POS)

/* Bit 2: SIGMODE - Signal Mode (0=Quadrature, 1=Clock/Direction) */
#define QEI_CTL_SIGMODE_POS             (2U)
#define QEI_CTL_SIGMODE_MASK            (1UL << QEI_CTL_SIGMODE_POS)
#define QEI_CTL_SIGMODE_QUADRATURE      (0UL << QEI_CTL_SIGMODE_POS)
#define QEI_CTL_SIGMODE_CLOCKDIR        (1UL << QEI_CTL_SIGMODE_POS)

/*
 * Bit 3: CAPMODE - Capture Mode. Controls DECODE RESOLUTION, i.e. which
 * phase edges increment/decrement QEIPOS - NOT velocity capture (that is
 * the separate VELEN bit below). Previously named/commented as if this
 * were a velocity-related setting ("0=Edges only, 1=Edges with
 * velocity"); it is not - it is the x2-vs-x4 decode resolution selector:
 *   0 = count edges on PhA only        (x2: both edges of one channel)
 *   1 = count edges on both PhA and PhB (x4: both edges of both channels)
 * This driver sets it to 1 (BOTH_PHASES) for full x4 quadrature decode,
 * matching QEI_COUNTS_PER_REV = 2464 in qei_cfg.h.
 */
#define QEI_CTL_CAPMODE_POS             (3U)
#define QEI_CTL_CAPMODE_MASK            (1UL << QEI_CTL_CAPMODE_POS)
#define QEI_CTL_CAPMODE_PHA_ONLY        (0UL << QEI_CTL_CAPMODE_POS)
#define QEI_CTL_CAPMODE_BOTH_PHASES     (1UL << QEI_CTL_CAPMODE_POS)

/* Bit 4: RESMODE - Reset Mode (0=Index, 1=Maximum position) */
#define QEI_CTL_RESMODE_POS             (4U)
#define QEI_CTL_RESMODE_MASK            (1UL << QEI_CTL_RESMODE_POS)
#define QEI_CTL_RESMODE_INDEX           (0UL << QEI_CTL_RESMODE_POS)
#define QEI_CTL_RESMODE_MAXPOS          (1UL << QEI_CTL_RESMODE_POS)

/* Bit 5: VELEN - Velocity Capture Enable */
#define QEI_CTL_VELEN_POS               (5U)
#define QEI_CTL_VELEN_MASK              (1UL << QEI_CTL_VELEN_POS)

/* Bits 7:6: VELDIV - Velocity Predivider (0=/1, 1=/2, 2=/4, 3=/8, etc.) */
#define QEI_CTL_VELDIV_POS              (6U)
#define QEI_CTL_VELDIV_MASK             (0x7UL << QEI_CTL_VELDIV_POS)
#define QEI_CTL_VELDIV_1                (0x0UL << QEI_CTL_VELDIV_POS)
#define QEI_CTL_VELDIV_2                (0x1UL << QEI_CTL_VELDIV_POS)
#define QEI_CTL_VELDIV_4                (0x2UL << QEI_CTL_VELDIV_POS)
#define QEI_CTL_VELDIV_8                (0x3UL << QEI_CTL_VELDIV_POS)
#define QEI_CTL_VELDIV_16               (0x4UL << QEI_CTL_VELDIV_POS)
#define QEI_CTL_VELDIV_32               (0x5UL << QEI_CTL_VELDIV_POS)
#define QEI_CTL_VELDIV_64               (0x6UL << QEI_CTL_VELDIV_POS)
#define QEI_CTL_VELDIV_128              (0x7UL << QEI_CTL_VELDIV_POS)

/* Bit 9: INVA - Invert PhA */
#define QEI_CTL_INVA_POS                (9U)
#define QEI_CTL_INVA_MASK               (1UL << QEI_CTL_INVA_POS)

/* Bit 10: INVB - Invert PhB */
#define QEI_CTL_INVB_POS                (10U)
#define QEI_CTL_INVB_MASK               (1UL << QEI_CTL_INVB_POS)

/* Bit 11: INVI - Invert Index */
#define QEI_CTL_INVI_POS                (11U)
#define QEI_CTL_INVI_MASK               (1UL << QEI_CTL_INVI_POS)

/* Bit 12: STALLEN - Stall QEI during debug */
#define QEI_CTL_STALLEN_POS             (12U)
#define QEI_CTL_STALLEN_MASK            (1UL << QEI_CTL_STALLEN_POS)

/* Bit 13: FILTEN - Enable Input Filter */
#define QEI_CTL_FILTEN_POS              (13U)
#define QEI_CTL_FILTEN_MASK             (1UL << QEI_CTL_FILTEN_POS)

/* Bits 19:16: FILTCNT - Filter Count (0-15) */
#define QEI_CTL_FILTCNT_POS             (16U)
#define QEI_CTL_FILTCNT_MASK            (0xFUL << QEI_CTL_FILTCNT_POS)

/*******************************************************************************
 *                          QEI Status Register (QEISTAT) Bits                 *
 *******************************************************************************/

/* Bit 0: ERROR - Error detected */
#define QEI_STAT_ERROR_POS              (0U)
#define QEI_STAT_ERROR_MASK             (1UL << QEI_STAT_ERROR_POS)

/* Bit 1: DIRECTION - Current direction (0=Forward, 1=Reverse) */
#define QEI_STAT_DIRECTION_POS          (1U)
#define QEI_STAT_DIRECTION_MASK         (1UL << QEI_STAT_DIRECTION_POS)

/*******************************************************************************
 *                          QEI Interrupt Bits                                 *
 *******************************************************************************/

/* Bit 0: INTINDEX - Index Pulse Interrupt */
#define QEI_INT_INDEX_POS               (0U)
#define QEI_INT_INDEX_MASK              (1UL << QEI_INT_INDEX_POS)

/* Bit 1: INTTIMER - Velocity Timer Expired Interrupt */
#define QEI_INT_TIMER_POS               (1U)
#define QEI_INT_TIMER_MASK              (1UL << QEI_INT_TIMER_POS)

/* Bit 2: INTDIR - Direction Change Interrupt */
#define QEI_INT_DIR_POS                 (2U)
#define QEI_INT_DIR_MASK                (1UL << QEI_INT_DIR_POS)

/* Bit 3: INTERROR - Phase Error Interrupt */
#define QEI_INT_ERROR_POS               (3U)
#define QEI_INT_ERROR_MASK              (1UL << QEI_INT_ERROR_POS)

#define QEI_INT_ALL_MASK                (QEI_INT_INDEX_MASK | QEI_INT_TIMER_MASK | \
                                         QEI_INT_DIR_MASK | QEI_INT_ERROR_MASK)

/*******************************************************************************
 *                          Helper Macros                                      *
 *******************************************************************************/

/* Get QEI module base address */
#define QEI_GET_BASE(channel)           ((channel) == 0 ? QEI0_BASE_ADDR : QEI1_BASE_ADDR)

/* Math constants */
#define QEI_PI                          (3.14159265359f)
#define QEI_2PI                         (6.28318530718f)
#define QEI_DEG_PER_RAD                 (57.2957795131f)

#endif /* QEI_PRIVATE_H_ */
