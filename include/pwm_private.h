/******************************************************************************
 *
 * Module: PWM
 *
 * File Name: pwm_private.h
 *
 * Description: Private definitions for TM4C123GH6PM PWM driver
 *              Bit-position/mask constants and per-channel register
 *              selection. Included by PWM.c only.
 *
 ******************************************************************************/

#ifndef PWM_PRIVATE_H_
#define PWM_PRIVATE_H_

#include "Platform_Types.h"

/*******************************************************************************
 *                  SYSCTL Bits Touched By This Driver                         *
 * (register addresses themselves live in tm4c123gh6pm_registers.h, since      *
 *  they are shared SoC-level registers, not private to PWM)                   *
 *******************************************************************************/

/* SYSCTL_RCGCPWM - bit 0 enables PWM module 0's clock */
#define PWM_PRIV_RCGCPWM_PWM0_POS        (0U)
#define PWM_PRIV_RCGCPWM_PWM0_MASK       (1UL << PWM_PRIV_RCGCPWM_PWM0_POS)

/* SYSCTL_PRPWM - bit 0 reads back PWM module 0's clock-ready status.
 * Polling this (instead of a heuristic busy-loop) closes the missing-
 * PRPWM-wait gap flagged in PWM_AUDIT.md Section 1, mirroring the PORT
 * driver's existing RCGCGPIO/PRGPIO ready-wait. */
#define PWM_PRIV_PRPWM_PWM0_POS          (0U)
#define PWM_PRIV_PRPWM_PWM0_MASK         (1UL << PWM_PRIV_PRPWM_PWM0_POS)

/* SYSCTL_RCC - bit 20 (USEPWMDIV) selects whether the PWM clock divider
 * chain is applied. Cleared so PWM clock = system clock, undivided. */
#define PWM_PRIV_RCC_USEPWMDIV_POS       (20U)
#define PWM_PRIV_RCC_USEPWMDIV_MASK      (1UL << PWM_PRIV_RCC_USEPWMDIV_POS)

/*******************************************************************************
 *                  Per-Generator PWMnCTL Bits                                 *
 *******************************************************************************/

#define PWM_PRIV_GEN_CTL_ENABLE_POS      (0U)   /* Generator counter enable */
#define PWM_PRIV_GEN_CTL_ENABLE_MASK     (1UL << PWM_PRIV_GEN_CTL_ENABLE_POS)

#define PWM_PRIV_GEN_CTL_MODE_POS        (1U)   /* 0=count-down, 1=count-up/down */
#define PWM_PRIV_GEN_CTL_MODE_MASK       (1UL << PWM_PRIV_GEN_CTL_MODE_POS)

#define PWM_PRIV_GEN_CTL_DEBUG_POS       (2U)   /* 1=keep counting while CPU halted in debug */
#define PWM_PRIV_GEN_CTL_DEBUG_MASK      (1UL << PWM_PRIV_GEN_CTL_DEBUG_POS)

/*******************************************************************************
 *                  Per-Generator PWMnGENA Action Encoding                     *
 * Bits 7:6 = action on Comparator-A match while counting UP                   *
 * Bits 5:4 = action on Comparator-A match while counting DOWN                 *
 *******************************************************************************/

#define PWM_PRIV_GENA_ACTION_DRIVE_LOW   (0x2UL)  /* drive output Low on match */
#define PWM_PRIV_GENA_ACTION_DRIVE_HIGH  (0x3UL)  /* drive output High on match */

#define PWM_PRIV_GENA_ACTCMPAU_POS       (6U)   /* action-on-up-match field position */
#define PWM_PRIV_GENA_ACTCMPAD_POS       (4U)   /* action-on-down-match field position */

/* Verified-on-hardware Output-A action configuration (PWM_AUDIT.md Section 3
 * follow-up): drive Low at the up-counting CMPA match, drive High at the
 * down-counting CMPA match. Combined with CMPA = LOAD*(1-percent/100), this
 * physically produces 0%=stopped, 100%=full speed. Do not change without
 * re-verifying on hardware. */
#define PWM_PRIV_GENA_CONFIG             ((PWM_PRIV_GENA_ACTION_DRIVE_LOW  << PWM_PRIV_GENA_ACTCMPAU_POS) | \
                                           (PWM_PRIV_GENA_ACTION_DRIVE_HIGH << PWM_PRIV_GENA_ACTCMPAD_POS))

/*******************************************************************************
 *                  PWM0ENABLE - Per-Output Enable Bits                        *
 *******************************************************************************/

#define PWM_PRIV_OUTPUT_ENABLE_M0PWM0_POS  (0U)  /* M0PWM0 = PB6 = RIGHT motor */
#define PWM_PRIV_OUTPUT_ENABLE_M0PWM2_POS  (2U)  /* M0PWM2 = PB4 = LEFT motor */

/*******************************************************************************
 *                  Module-Level PWM0_CTL (Global Sync) Bits                   *
 *******************************************************************************/

#define PWM_PRIV_MODULE_CTL_GLOBALSYNC0_POS  (0U)
#define PWM_PRIV_MODULE_CTL_GLOBALSYNC1_POS  (1U)

/*******************************************************************************
 *                  Per-Channel Register Selection                             *
 *******************************************************************************/

/* One generator's full register set, selected per-channel so PWM.c never
 * needs an if/else on the channel past this single lookup. */
typedef struct
{
    volatile uint32 *ctl;
    volatile uint32 *load;
    volatile uint32 *cmpa;
    volatile uint32 *gena;
    uint8             outputEnableBitPos;
} PWM_GeneratorRegsType;

#endif /* PWM_PRIVATE_H_ */
