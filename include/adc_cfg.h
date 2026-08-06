/******************************************************************************
 *
 * Module: ADC (MCAL)
 *
 * File Name: adc_cfg.h
 *
 * Description: Compile-time configuration for the generic ADC0 single-channel
 *              reader. Hardware facts only: which ADC module, which input
 *              channel (AINx), and which sample sequencer. This file is
 *              application-agnostic - it does NOT mention "pot", "servo", or
 *              "angle"; those concepts live above the MCAL (servo_feedback.*).
 *
 *              Pin facts: the analog input is AIN0 = PE3, muxed to analog mode
 *              by the PORT driver (PORT_ANALOG in PORT_PBCFG.c). See ADC_POT_REPORT.md.
 *
 ******************************************************************************/

#ifndef ADC_CFG_H_
#define ADC_CFG_H_

/* ADC input channel: AIN0 (= PE3). Written into ADCSSMUX3. */
#define ADC_CFG_CHANNEL_AIN             (0U)

/* NOTE: the sample sequencer is NOT configurable here. This driver is hardwired
 * to Sample Sequencer 3 (depth-1 FIFO - exactly right for one channel) through
 * the ADC0_SSMUX3/SSCTL3/SSFIFO3 registers and the ADC_PRIV_*_SS3_MASK bit
 * definitions in adc_private.h. A former ADC_CFG_SEQUENCER macro lived here and
 * was referenced by nothing, so editing it changed no behaviour while looking
 * like it would; it has been removed rather than left as a trap. Moving to
 * another sequencer means editing adc.c and adc_private.h together. */

#endif /* ADC_CFG_H_ */
