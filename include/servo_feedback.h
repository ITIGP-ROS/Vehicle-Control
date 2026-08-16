/******************************************************************************
 *
 * Module: ServoFb (steering-pot feedback)
 *
 * File Name: servo_feedback.h
 *
 * Description: Thin feedback layer ABOVE the ADC MCAL. Turns the steering pot
 *              into an angle (rad, REP-103) using the Phase-2 measured
 *              center/scale calibration, and also still exposes the raw /
 *              lightly-filtered ADC counts (useful for debug / recalibration).
 *
 *              Layering: angle  ->  ServoFb_ReadRawFiltered (filter)  ->  Adc
 *              (MCAL raw). The averaging filter lives here, NOT in the MCAL.
 *              Calibration constants live in servo_feedback_cfg.h.
 *
 ******************************************************************************/

#ifndef SERVO_FEEDBACK_H_
#define SERVO_FEEDBACK_H_

#include "Platform_Types.h"
#include "servo_feedback_cfg.h"

/**
 * @brief  Initialize the steering-pot feedback path (brings up the ADC MCAL).
 * @note   Call once at boot, after Port_Init() (which sets PE3 to analog).
 */
void ServoFb_Init(void);

/**
 * @brief  Read the steering pot as a lightly-filtered RAW ADC count.
 *         Averages SERVO_FB_FILTER_SAMPLES consecutive Adc_ReadRaw() samples.
 * @return Filtered raw count, 0..4095. NO units, NO angle - calibration is a
 *         later task.
 */
uint16 ServoFb_ReadRawFiltered(void);

/**
 * @brief  Pure conversion: filtered ADC count -> steering angle (rad, REP-103).
 *         angle = (count - SERVO_FB_POT_CENTER) * SERVO_FB_SCALE_{LEFT,RIGHT}_RAD_PER_COUNT,
 *         the scale chosen by which side of centre the count falls (the linkage is
 *         asymmetric; see servo_feedback_cfg.h, re-calibrated 2026-08-16).
 *         Exposed so a caller can convert a count it already has (e.g. the
 *         bring-up stream) without a second ADC read.
 * @param  raw_count: a filtered ADC count (0..4095).
 * @return Steering angle in radians (+left / -right). NOT clamped.
 */
float32 ServoFb_CountToAngleRad(uint16 raw_count);

/**
 * @brief  Read the steering angle in radians (REP-103: +left / -right).
 *         = ServoFb_CountToAngleRad(ServoFb_ReadRawFiltered()).
 * @return Measured angle (rad). The TRUE measured value - deliberately NOT
 *         clamped to any travel limit; the steering loop decides limits.
 */
float32 ServoFb_ReadAngleRad(void);

/**
 * @brief  Pure plausibility test on a count the caller ALREADY has - the fault
 *         twin of ServoFb_CountToAngleRad(), and for the same reason: it lets a
 *         caller derive BOTH the angle and the fault verdict from ONE
 *         acquisition, so the two describe the same instant instead of two
 *         independent samples of a moving quantity. Performs NO ADC read.
 * @param  raw_count: a filtered ADC count (0..4095), or the ADC_READ_INVALID
 *         sentinel - which is outside the window and so reports faulted.
 * @return TRUE if the count is OUTSIDE
 *         [SERVO_FB_POT_MIN_VALID, SERVO_FB_POT_MAX_VALID], else FALSE.
 */
boolean ServoFb_IsCountFaulted(uint16 raw_count);

/**
 * @brief  Separate (non-clamping) sanity flag for a disconnected/shorted pot.
 *         Reads the filtered count and reports whether it is OUTSIDE the
 *         plausible window [SERVO_FB_POT_MIN_VALID, SERVO_FB_POT_MAX_VALID].
 *         This does NOT affect ServoFb_ReadAngleRad() - it is an independent
 *         fault indicator for a higher layer to act on.
 * @note   Convenience wrapper = ServoFb_IsCountFaulted(ServoFb_ReadRawFiltered()),
 *         i.e. it COSTS AN ACQUISITION. If you already hold a count, or you also
 *         need the angle from the same instant, call ServoFb_IsCountFaulted()
 *         instead.
 * @return TRUE if the pot reading is implausible (likely fault), else FALSE.
 */
boolean ServoFb_IsPotFaulted(void);

#endif /* SERVO_FEEDBACK_H_ */
