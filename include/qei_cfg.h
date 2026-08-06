/******************************************************************************
 *
 * Module: QEI (Quadrature Encoder Interface)
 *
 * File Name: qei_cfg.h
 *
 * Description: Configuration header for TM4C123GH6PM QEI driver
 *              Configured for dual DC motor encoders
 *
 ******************************************************************************/

#ifndef QEI_CFG_H_
#define QEI_CFG_H_

/*******************************************************************************
 *                          System Configuration                               *
 *******************************************************************************/

/* System clock frequency in Hz */
#define QEI_SYSTEM_CLOCK_HZ             (16000000UL)

/*******************************************************************************
 *                          QEI Module Enable                                  *
 *******************************************************************************/

#define QEI0_ENABLED                    (1U)    /* Motor A encoder */
#define QEI1_ENABLED                    (1U)    /* Motor B encoder */

/*******************************************************************************
 *                          Encoder Hardware Parameters                        *
 *******************************************************************************/

/*
 * Encoder specifications (per motor) - hardware-verified, see QEI_AUDIT.md
 * and QEI_REFACTOR.md:
 * - Motor: JGB37-520 gearmotor, quadrature hall encoder on the MOTOR shaft
 * - Base encoder resolution: 11 PPR (pulses per revolution), MOTOR shaft,
 *   BEFORE the gearbox
 * - Gearbox reduction: 56:1 (output shaft turns once per 56 motor-shaft turns)
 * - Quadrature decode: x4 (QEICTL.CAPMODE=1, both edges of both PhA/PhB -
 *   see QEI_CTL_CAPMODE_BOTH_PHASES in qei_private.h)
 * - Counts per OUTPUT-shaft revolution = 11 * 56 * 4 = 2464
 *   (hardware-confirmed: one measured hand-turn gave ~2460-2511 counts)
 *
 * Each factor below is its own named constant so the gear ratio can never
 * again be silently absorbed into a mislabeled "PPR" constant - that is
 * exactly what happened before this refactor (QEI_ENCODER_PPR was 616,
 * i.e. 11*56 already multiplied together, mislabeled as "base encoder
 * resolution" with a comment that wrongly claimed a 1:1 gear ratio).
 */

/* Raw encoder pulses per revolution, on the MOTOR shaft (pre-gearbox) */
#define QEI_ENCODER_PPR_BASE            (11UL)

/* Gearbox reduction ratio, output-shaft-revolutions : motor-shaft-revolutions */
#define QEI_GEAR_RATIO                  (56UL)

/* Quadrature multiplication factor (1, 2, or 4) */
/* 4 = count all edges of both A and B (highest resolution) */
#define QEI_QUADRATURE_MODE             (4UL)

/* Total counts per OUTPUT-shaft revolution: PPR * gear ratio * quadrature */
#define QEI_COUNTS_PER_REV              (QEI_ENCODER_PPR_BASE * QEI_GEAR_RATIO * QEI_QUADRATURE_MODE)

/* Compile-time guard: if any factor above is ever changed by mistake,
 * fail the build loudly instead of silently scaling every position/
 * velocity/distance reading this driver produces. 2464 is the
 * hardware-verified value for this robot's JGB37-520 + 11 PPR + 56:1 gear. */
#if (QEI_COUNTS_PER_REV != 2464UL)
#error "QEI_COUNTS_PER_REV must be 2464 for JGB37-520 (11 PPR x 56 gear x 4) - check the factors"
#endif

/* MAXPOS register value (counts per rev - 1) */
//#define QEI_MAXPOS_VALUE                (QEI_COUNTS_PER_REV - 1UL)
#define QEI_MAXPOS_VALUE  (0xFFFFFFFFUL)

/*******************************************************************************
 *                          Wheel/Robot Parameters                             *
 *******************************************************************************/

/* Wheel diameter in millimeters */
#define QEI_WHEEL_DIAMETER_MM           (65.0f)

/* Wheel circumference in mm */
#define QEI_WHEEL_CIRCUMFERENCE_MM      (QEI_WHEEL_DIAMETER_MM * 3.14159265359f)

/* Wheel circumference in cm */
#define QEI_WHEEL_CIRCUMFERENCE_CM      (QEI_WHEEL_CIRCUMFERENCE_MM / 10.0f)

/* Wheel circumference in meters */
#define QEI_WHEEL_CIRCUMFERENCE_M       (QEI_WHEEL_CIRCUMFERENCE_MM / 1000.0f)

/* Distance per encoder count in mm */
#define QEI_MM_PER_COUNT                (QEI_WHEEL_CIRCUMFERENCE_MM / (float32)QEI_COUNTS_PER_REV)

/* Distance per encoder count in cm */
#define QEI_CM_PER_COUNT                (QEI_WHEEL_CIRCUMFERENCE_CM / (float32)QEI_COUNTS_PER_REV)

/* Distance per encoder count in meters */
#define QEI_M_PER_COUNT                 (QEI_WHEEL_CIRCUMFERENCE_M / (float32)QEI_COUNTS_PER_REV)

/* Counts per cm (for integer math) */
#define QEI_COUNTS_PER_CM               ((float32)QEI_COUNTS_PER_REV / QEI_WHEEL_CIRCUMFERENCE_CM)

/* Wheel base (distance between wheels) in mm - for differential drive odometry */
#define QEI_WHEEL_BASE_MM               (150.0f)

/*******************************************************************************
 *                          Velocity Measurement Configuration                 *
 *******************************************************************************/

/*
 * Velocity timer configuration:
 *
 * The QEI module measures velocity by counting pulses over a fixed time period.
 * LOAD register defines the measurement period. This is a true HARDWARE
 * sample-and-hold: QEISPEED only updates once per period, autonomously,
 * regardless of how often software reads it.
 *
 * velocity_period = (LOAD + 1) / SYS_CLOCK
 *
 * Trade-off:
 * - Longer period = smoother velocity, slower update rate
 * - Shorter period = faster update, more noise
 *
 * For 20ms update period at 16MHz: LOAD = 16000000 * 0.02 - 1 = 319999
 */

/* Velocity measurement period in milliseconds. MUST equal the PID
 * velocity-loop period (currently 20ms) - QEISPEED (read by
 * QEI_GetVelocityRPM) only refreshes once per this period, so polling it
 * faster than this value reads the SAME stale sample 2+ times per real
 * hardware update, which breaks closed-loop feedback regardless of gains
 * (see QEI_WINDOW_AUDIT.md). Polling slower than this value just wastes
 * the extra resolution, which is harmless but pointless. */
#define QEI_VELOCITY_PERIOD_MS          (20UL)

/* LOAD register value for velocity timer */
#define QEI_VELOCITY_LOAD_VALUE         ((QEI_SYSTEM_CLOCK_HZ * QEI_VELOCITY_PERIOD_MS / 1000UL) - 1UL)

/* Velocity measurement frequency in Hz */
#define QEI_VELOCITY_UPDATE_HZ          (1000UL / QEI_VELOCITY_PERIOD_MS)

/*******************************************************************************
 *                          Filter Configuration                               *
 *******************************************************************************/

/* Enable input filter (recommended for noisy environments) */
#define QEI_FILTER_ENABLE               (1U)

/* Filter count (0-15): Number of consecutive samples before accepting change */
/* Higher = more noise rejection, but slower response */
#define QEI_FILTER_COUNT                (3U)

/*******************************************************************************
 *                          Interrupt Configuration                            *
 *******************************************************************************/

/* Enable QEI interrupts - currently unused: the ISRs, NVIC setup, and
 * callback-registration API this would have driven were all removed as
 * dead code (see QEI_REFACTOR.md). Velocity/position are polled instead.
 * Kept at 0 as a marker that interrupt-driven QEI is not wired up. */
#define QEI_INTERRUPT_ENABLE            (0U)

/*******************************************************************************
 *                          Direction Configuration                            *
 *******************************************************************************/

/*
 * Swap A/B inputs to reverse direction sense (QEICTL.SWAP, hardware).
 *
 * The two motors are mirror-mounted, so their encoders see the same
 * physical wheel rotation with opposite PhA/PhB phase relationships.
 * QEI_CHANNEL_0 (right motor, PD6/PD7) needs its hardware SWAP bit set so
 * that forward wheel rotation still produces an INCREASING position count
 * and DIR=FORWARD, matching QEI_CHANNEL_1 (left motor, PC5/PC6), which
 * does not need the swap.
 *
 * Hardware-verified (see QEI_AUDIT.md / QEI_REFACTOR.md hand-spin test):
 * with these settings, spinning either wheel in its own chassis-forward
 * sense gives an increasing count and DIR=FORWARD on that wheel's
 * channel. Do NOT change either value without re-running that hardware
 * test - getting this wrong silently inverts every position/velocity
 * sign for one motor.
 */
#define QEI0_SWAP_SIGNALS               (1U) /* right motor (QEI_CHANNEL_0) - mirror-mount compensation */
#define QEI1_SWAP_SIGNALS               (0U) /* left motor (QEI_CHANNEL_1) - no compensation needed */

/* Invert direction interpretation */
#define QEI0_INVERT_DIRECTION           (0U)
#define QEI1_INVERT_DIRECTION           (0U)

/*******************************************************************************
 *                          Index Signal Configuration                         *
 *******************************************************************************/

/* Enable index signal (if encoder has index/Z channel) */
#define QEI_INDEX_ENABLE                (0U)

/* Reset position on index pulse */
#define QEI_INDEX_RESET_ENABLE          (0U)

#endif /* QEI_CFG_H_ */
