/******************************************************************************
 *
 * Module: ServoFb (steering-pot feedback)
 *
 * File Name: servo_feedback_cfg.h
 *
 * Description: Configuration for the steering-pot feedback layer. For now this
 *              is just the light-filter depth; the angle calibration constants
 *              (center/scale in ADC counts) are DEFERRED until they are
 *              measured from this tool's raw output.
 *
 ******************************************************************************/

#ifndef SERVO_FEEDBACK_CFG_H_
#define SERVO_FEEDBACK_CFG_H_

/* Moving-average depth for ServoFb_ReadRawFiltered(). The pot showed ~+/-2-3
 * counts of noise at rest (expect worse with motors running); averaging 8
 * samples knocks that down without much lag. Must be >= 1. */
#define SERVO_FB_FILTER_SAMPLES         (8U)

/*******************************************************************************
 *      Steering-Angle Calibration - RE-MEASURED 2026-07-30 (linkage rework)   *
 *  Linear pot: a least-squares fit over the 600..2400us region gives          *
 *      POT = 1.7418 * pulse_us - 550.6   (max residual 28.5 counts)           *
 *  i.e. ~1.74 counts/us, straight enough that a single center + scale (no LUT) *
 *  maps count -> angle (REP-103).                                             *
 *                                                                            *
 *  Captured via the bringup jog+pot stream (TM4C 12-bit ADC, 8-sample avg),   *
 *  every landmark approached FROM CENTER so backlash loads the same way:      *
 *      max LEFT    600us -> POT  557 -> +0.2421 rad  (achieved, see below)    *
 *      CENTER     1450us -> POT 1991 ->  0.0000 rad  (wheels straight)        *
 *      max RIGHT  2500us -> POT 3790 -> -0.3037 rad                           *
 *  Sign: higher POT = RIGHT = NEGATIVE (REP-103: +left / -right).            *
 *                                                                            *
 *  Usable pot span 557..3790 of 4095 - ~400 counts of headroom below and ~300 *
 *  above, so the linkage does NOT outrun the pot at either end (no railing).  *
 *                                                                            *
 *  DESIGN vs ACHIEVED: the CAD steering angle is 17.4 deg (0.3037 rad) PER    *
 *  SIDE, symmetric. The RIGHT endpoint hits it exactly (by construction - the *
 *  scale below is derived from it). The LEFT endpoint only achieves           *
 *  +0.2421 rad (13.9 deg), because the servo runs out of authority before the *
 *  linkage does. Mapped honestly here, NOT "corrected" in code - see the      *
 *  AT_TARGET consequence noted in servo_cfg.h and REVIEW_02.                  *
 *                                                                            *
 *  Backlash: revisiting CENTER from either side spans ~40 counts (2005 from   *
 *  the right, 1965 from the left) ~= 0.007 rad. Unchanged from the previous   *
 *  linkage. Always approach a calibration point from the SAME direction.      *
 *******************************************************************************/

/* ADC counts at wheels-straight (angle == 0). Mean of three CENTER visits
 * (2001.5 / 1965.1 / 2005.8) - the spread is the backlash band above. */
#define SERVO_FB_POT_CENTER             (1991)

/* Radians per ADC count. Derived from the RIGHT endpoint (largest count span =
 * best resolution), which also makes the right side exact:
 *      scale = -0.3037 rad / (3790 - 1991 counts) = -0.3037 / 1799 = -0.00016882
 * Check LEFT:  ( 557 - 1991) * -0.00016882 = -1434 * -0.00016882 = +0.2421 rad (13.9 deg)
 * Check RIGHT: (3790 - 1991) * -0.00016882 = +1799 * -0.00016882 = -0.3037 rad (17.4 deg)
 * The LEFT check is BELOW the CAD 17.4 deg on purpose - that is the achieved
 * left travel, not an error in this scale (see DESIGN vs ACHIEVED above).
 * Negative because higher counts = RIGHT = negative angle. */
#define SERVO_FB_SCALE_RAD_PER_COUNT    (-0.00016882f)

/* OPTIONAL coarse pot-fault window (NOT travel limits, NOT used to clamp the
 * angle). Normal travel is now 557..3790, so the previous [900, 4000] window
 * would have false-triggered POT_FAULT across the whole left half of travel.
 * Widened to sit outside real travel while still catching a disconnected/
 * shorted input, which rails toward 0 or 4095 ->
 * ServoFb_IsPotFaulted() reports it as a separate flag. */
#define SERVO_FB_POT_MIN_VALID          (250U)
#define SERVO_FB_POT_MAX_VALID          (3950U)

#endif /* SERVO_FEEDBACK_CFG_H_ */
