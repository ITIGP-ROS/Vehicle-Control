/******************************************************************************
 * Servo Feedback Layer - steering-pot raw/filtered counts (ABOVE the ADC MCAL)
 *
 * Provides a lightly-filtered raw ADC count for the steering pot AND the
 * Phase-2 linear count->angle conversion (rad, REP-103). The filter (a small
 * block average) lives HERE, not in the ADC MCAL, keeping the MCAL generic.
 * Calibration center/scale live in servo_feedback_cfg.h.
 ******************************************************************************/

#include "Platform_Types.h"
#include "servo_feedback.h"
#include "adc.h"            /* the MCAL - the only way this layer reaches the ADC */

void ServoFb_Init(void)
{
    Adc_Init();
}

uint16 ServoFb_ReadRawFiltered(void)
{
    uint32 sum = 0U;
    uint8  i;

    for (i = 0U; i < (uint8)SERVO_FB_FILTER_SAMPLES; i++)
    {
        sum += (uint32)Adc_ReadRaw();
    }

    /* Block average. SERVO_FB_FILTER_SAMPLES is >= 1 (no divide-by-zero); the
     * sum of N 12-bit samples fits a uint32 for any realistic N. */
    return (uint16)(sum / (uint32)SERVO_FB_FILTER_SAMPLES);
}

float32 ServoFb_CountToAngleRad(uint16 raw_count)
{
    /* angle = (count - center) * scale ; signed difference so left (< center)
     * gives + and right (> center) gives - (REP-103). NOT clamped. */
    return (float32)((sint32)raw_count - SERVO_FB_POT_CENTER)
           * SERVO_FB_SCALE_RAD_PER_COUNT;
}

float32 ServoFb_ReadAngleRad(void)
{
    return ServoFb_CountToAngleRad(ServoFb_ReadRawFiltered());
}

boolean ServoFb_IsCountFaulted(uint16 raw_count)
{
    /* Pure - no ADC read. Separate fault flag; does NOT clamp the angle.
     * Implausible counts (outside the coarse valid window) indicate a
     * disconnected/shorted pot, and the ADC_READ_INVALID sentinel (0xFFFF) is
     * far above the window so it reports faulted here too. */
    return ((raw_count < (uint16)SERVO_FB_POT_MIN_VALID) ||
            (raw_count > (uint16)SERVO_FB_POT_MAX_VALID)) ? TRUE : FALSE;
}

boolean ServoFb_IsPotFaulted(void)
{
    /* Convenience wrapper: costs one acquisition. Callers that also need the
     * angle from the SAME sample should take the raw count once themselves and
     * use ServoFb_CountToAngleRad + ServoFb_IsCountFaulted (R3-2). */
    return ServoFb_IsCountFaulted(ServoFb_ReadRawFiltered());
}
