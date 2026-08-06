/******************************************************************************
 *
 * Module: ADC (MCAL)
 *
 * File Name: adc_private.h
 *
 * Description: Private register bit/field definitions for the ADC0 SS3
 *              single-channel reader. Included by adc.c only. Register
 *              addresses live in tm4c123gh6pm_registers.h (ADC0_* block).
 *
 ******************************************************************************/

#ifndef ADC_PRIVATE_H_
#define ADC_PRIVATE_H_

#include "Platform_Types.h"

/*******************************************************************************
 *                  SYSCTL Bits Touched By This Driver                         *
 *******************************************************************************/

/* SYSCTL_RCGCADC bit 0 enables the ADC0 module clock. */
#define ADC_PRIV_RCGCADC_ADC0_POS       (0U)
#define ADC_PRIV_RCGCADC_ADC0_MASK      (1UL << ADC_PRIV_RCGCADC_ADC0_POS)

/* SYSCTL_PRADC bit 0 reads back ADC0's clock-ready status (RCGC/PR ready-wait,
 * mirroring the PWM/PORT/TimerPWM drivers). */
#define ADC_PRIV_PRADC_ADC0_POS         (0U)
#define ADC_PRIV_PRADC_ADC0_MASK        (1UL << ADC_PRIV_PRADC_ADC0_POS)

/*******************************************************************************
 *                  Sample Sequencer 3 (SS3) Bit Masks                         *
 *  All of these target sequencer 3 specifically (bit 3 in the per-SS fields). *
 *******************************************************************************/

/* ADCACTSS - Active Sample Sequencer: ASEN3 (bit 3) enables SS3. */
#define ADC_PRIV_ACTSS_SS3_POS          (3U)
#define ADC_PRIV_ACTSS_SS3_MASK         (1UL << ADC_PRIV_ACTSS_SS3_POS)

/* ADCEMUX - Event Mux: EM3 field (bits 15:12). 0x0 = processor (software)
 * trigger. Mask used to CLEAR the field down to software-trigger. */
#define ADC_PRIV_EMUX_EM3_MASK          (0xFUL << 12U)
#define ADC_PRIV_EMUX_EM3_SOFTWARE      (0x0UL << 12U)

/* ADCSSCTL3 - SS3 control for the (single) first sample:
 *   IE0  (bit 2) = 1 -> raw interrupt asserted when this sample completes
 *   END0 (bit 1) = 1 -> this is the end of the sequence
 *   (TS0/D0 left 0: not temp sensor, not differential) */
#define ADC_PRIV_SSCTL3_IE0_POS         (2U)
#define ADC_PRIV_SSCTL3_END0_POS        (1U)
#define ADC_PRIV_SSCTL3_CONFIG          ((1UL << ADC_PRIV_SSCTL3_IE0_POS) | \
                                         (1UL << ADC_PRIV_SSCTL3_END0_POS))

/* ADCPSSI - Processor Sample Sequencer Initiate: SS3 (bit 3) software-starts. */
#define ADC_PRIV_PSSI_SS3_POS           (3U)
#define ADC_PRIV_PSSI_SS3_MASK          (1UL << ADC_PRIV_PSSI_SS3_POS)

/* ADCRIS - Raw Interrupt Status: INR3 (bit 3) set when SS3 conversion done. */
#define ADC_PRIV_RIS_SS3_POS            (3U)
#define ADC_PRIV_RIS_SS3_MASK           (1UL << ADC_PRIV_RIS_SS3_POS)

/* ADCISC - Interrupt Status and Clear: IN3 (bit 3) write-1-to-clear. */
#define ADC_PRIV_ISC_SS3_POS            (3U)
#define ADC_PRIV_ISC_SS3_MASK           (1UL << ADC_PRIV_ISC_SS3_POS)

/* ADCSSMUX3 - input channel field for the first sample (bits 3:0). */
#define ADC_PRIV_SSMUX3_MUX0_MASK       (0xFUL)

/* ADCSSFIFO3 - 12-bit result occupies bits 11:0. */
#define ADC_PRIV_FIFO_RESULT_MASK       (0x0FFFUL)

/*******************************************************************************
 *                  Bounded-Wait Caps                                          *
 *  Every wait in this driver is bounded so a dead/unclocked peripheral fails   *
 *  fast instead of hanging the caller forever. Same rationale and style as     *
 *  TIMER0_INIT_SPIN_CAP (timer0_private.h:98).                                 *
 *******************************************************************************/

/*  ⚠️ REPLACED AT BUDGET-1 (2026-08-06). The old cap was an ITERATION COUNT:
 *
 *      #define ADC_PRIV_CONV_SPIN_CAP  (100000UL)
 *
 *  and its comment claimed it would terminate "in well under a millisecond on a
 *  dead module". THAT CLAIM WAS WRONG BY ~TWO ORDERS OF MAGNITUDE, and the
 *  reassurance is why the defect survived review.
 *
 *  Counted from the actual emitted code (arm-none-eabi-objdump of adc.o), the
 *  spin body is 6 instructions - subs / ldr(peripheral) / tst / bne / cmp / bne
 *  - i.e. ~8-12 cycles per iteration once the APB read and the taken-branch
 *  refill are included. At 16 MHz:
 *
 *      100,000 iterations x ~10 cycles / 16 MHz  =  ~62 ms   per Adc_ReadRaw
 *      x 8 (SERVO_FB_FILTER_SAMPLES)             =  ~500 ms  per pot read
 *
 *  Half a second, inside what B7 makes a 10 ms task. The count cap also made
 *  the duration -O dependent, the very hazard i2c.c calls out and rejects.
 *
 *  Replaced with a TIME bound measured on the free-running TIMER0, exactly as
 *  every wait in i2c.c already is. See ADC_CONV_TIMEOUT_US in adc.c.
 */

#endif /* ADC_PRIVATE_H_ */
