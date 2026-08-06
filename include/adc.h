/******************************************************************************
 *
 * Module: ADC (MCAL)
 *
 * File Name: adc.h
 *
 * Description: Public API for the generic ADC0 single-channel reader.
 *
 *  Reads one software-triggered channel (AIN0/PE3) on Sample Sequencer 3 and
 *  returns the RAW 12-bit count (0-4095). Blocking, bring-up-simple.
 *
 *  This is the MCAL: it knows only "ADC channel -> raw count". It has no
 *  concept of "pot", "servo", "angle", or filtering - those live above it
 *  (servo_feedback.*), mirroring how Encoder.c sits above QEI.c.
 *
 ******************************************************************************/

#ifndef ADC_H_
#define ADC_H_

#include "Platform_Types.h"
#include "adc_cfg.h"

/*******************************************************************************
 *                          Failure Sentinel                                   *
 *******************************************************************************/

/* Returned by Adc_ReadRaw() when no conversion result could be obtained (read
 * before Adc_Init(), or the conversion-complete flag never asserted within the
 * bounded wait). 0xFFFF is deliberately OUTSIDE the 12-bit result range
 * (0..4095), so it can never be confused with a real count.
 *
 * Layering note: this MCAL only reports the failure - it does not decide what
 * it means. The plausibility window above it (SERVO_FB_POT_MIN/MAX_VALID = 900
 * /4000 in servo_feedback_cfg.h) already rejects it: even ONE sentinel inside
 * the 8-sample block average of ServoFb_ReadRawFiltered() drags the average
 * above 4000, so ServoFb_IsPotFaulted() reports the fault. The averaged value
 * itself is meaningless in that case - it is flagged, not corrected. No change
 * to servo_feedback was made or is required. */
#define ADC_READ_INVALID                (0xFFFFU)

/**
 * @brief  One-time ADC0 bring-up: enable the ADC0 clock, wait for ready, and
 *         configure Sample Sequencer 3 for a single software-triggered
 *         conversion on the configured channel (ADC_CFG_CHANNEL_AIN).
 * @note   The analog pin (PE3/AIN0) must already be in analog mode via the
 *         PORT driver (PORT_ANALOG in PORT_PBCFG.c). Call after Port_Init().
 */
void Adc_Init(void);

/**
 * @brief  Trigger one conversion, wait (bounded) for it to complete, and
 *         return the raw 12-bit result.
 * @return Raw ADC count, 0..4095 on success. ADC_READ_INVALID (0xFFFF) if the
 *         driver has not been initialised, or if the conversion did not
 *         complete within ADC_PRIV_CONV_SPIN_CAP polls. (No scaling, no units -
 *         calibration happens in a layer above this MCAL.)
 * @note   NEVER spins unbounded: a dead/unclocked ADC fails fast instead of
 *         hanging the caller. Mirrors TimerPWM_SetPulseUs's
 *         TIMER_PWM_ERROR_NOT_INITIALIZED guard, expressed through the return
 *         value because this API returns a count rather than a status.
 */
uint16 Adc_ReadRaw(void);

#endif /* ADC_H_ */
