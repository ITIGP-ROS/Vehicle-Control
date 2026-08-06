#ifndef INA226_CFG_H_
#define INA226_CFG_H_

/******************************************************************************
 *
 * Module: INA226 (MCAL)
 *
 * File Name: ina226_cfg.h
 *
 * Description: Compile-time configuration for the INA226 driver - bus, address,
 *              device config word, and the ONE calibration constant the whole
 *              current chain depends on.
 *
 *              Everything here is a value a re-wiring or a re-calibration might
 *              legitimately change. The conversion mathematics live in ina226.c.
 *
 ******************************************************************************/

#include "Platform_Types.h"
#include "i2c_types.h"      /* I2C_ID_0 */

/*******************************************************************************
 *                          Bus / device                                       *
 *******************************************************************************/

/* INA226 is on I2C0 (PB2=SCL, PB3=SDA) at 400 kHz - a DIFFERENT bus from the
 * MPU6050's I2C1/PA6-PA7. See PINOUT.md. */
#define INA226_I2C_ID               I2C_ID_0

/* 7-bit address; the I2C driver applies the <<1 and the R/S bit itself. */
#define INA226_ADDR_7BIT            (0x40U)

/* Boot liveness anchor (register 0xFE). */
#define INA226_MANUF_ID_EXPECTED    (0x5449U)

/*******************************************************************************
 *                          Device configuration word                          *
 *******************************************************************************/

/*----------------------------------------------------------------------------
 * CONFIG = 0x46DF - the SERVICE config.
 *   bit[15]    RST      = 0    no reset. A 1 here makes the part self-reset
 *                              continuously and never converge; a previously
 *                              recorded "production" value had this bit set.
 *   bit[14:12] reserved = 100  fixed per datasheet
 *   bit[11:9]  AVG      = 011  -> 64 samples
 *   bit[8:6]   VBUSCT   = 011  -> 588 us bus   conversion time
 *   bit[5:3]   VSHCT    = 011  -> 588 us shunt conversion time
 *   bit[2:0]   MODE     = 111  -> shunt + bus, continuous
 *   => 0 100 011 011 011 111 = 0100 0110 1101 1111 = 0x46DF
 *   Conversion cycle = (588 + 588) us x 64 = 75.3 ms, which fits inside a
 *   100 ms service tick with margin.
 *
 * THE DECODE TABLE IS PART OF THE CONSTANT. A bare hex word is exactly how the
 * wrong value survived unnoticed for so long. Related words, for reference:
 *   0x4527 = AVG 16,  35.2 ms   - the bench word; safe fallback if 588 us
 *                                 conversions prove noisy in service
 *   0x4D27 = AVG 512, 1126 ms   - MUST NEVER SHIP: 11x too slow for the tick
 *
 * The R_shunt calibration (2026-08-04) was performed under THIS word. AVG is a
 * hardware boxcar average, so it sets how much PWM ripple each sample already
 * rejects - changing it invalidates the calibration conditions.
 *--------------------------------------------------------------------------*/
#define INA226_CONFIG               (0x46DFU)

/*----------------------------------------------------------------------------
 * CALIBRATION register word (0x05).
 *
 * WRITTEN AT INIT, BUT NOT USED BY THIS DRIVER. The service path computes
 * current in software from SHUNT_V (see ina226.h), so the CAL register affects
 * only the chip's own CURRENT (0x04) and POWER (0x03) registers, which this
 * driver never reads. It is programmed anyway so that a debugger, a bench, or
 * any future consumer peeking at those registers sees a sane value rather than
 * whatever survived the last reset.
 *
 * CAL = trunc(0.00512 / (Current_LSB x R_shunt))
 *     = trunc(0.00512 / (0.001 A x 0.00216 Ohm)) = trunc(2370.4) = 2370
 * using the MEASURED R_shunt below and a 1 mA Current_LSB (=> 32.77 A
 * full-scale on the chip's own register).
 *--------------------------------------------------------------------------*/
#define INA226_CAL_WORD             (2370U)

/*******************************************************************************
 *                          Calibration                                        *
 *******************************************************************************/

/*----------------------------------------------------------------------------
 * R_shunt - THE ONLY CALIBRATION TERM IN THE ENTIRE CURRENT/POWER/SoC CHAIN.
 *
 * MEASURED 2026-08-04, 4-point run against a PSU current display:
 *     duty  shunt_uV  I_psu     R
 *      50%     1490   0.675 A   2.2074 mOhm
 *      80%     1600   0.745 A   2.1477 mOhm
 *      90%     1588   0.755 A   2.1033 mOhm
 *     100%     1490   0.685 A   2.1752 mOhm
 *   mean 2.1584, current-weighted 2.1566, spread +/-2.41%.
 *
 * ACCURACY +/-~5-7%, dominated by the PSU's ABSOLUTE accuracy (~+/-3.4%, a
 * systematic term that does NOT average down), not by the spread.
 *
 * CONDITIONS (record with the value - the parasitic is thermally sensitive):
 *   both motors open-loop and UNLOADED (in air), ~0.7 A, bus 11.38-11.44 V,
 *   CONFIG 0x46DF. ALL POINTS ARE LOW-CURRENT: accuracy at 10-20 A is an
 *   extrapolation, not a measurement.
 *
 * WHY IT IS BELIEVED: the +/-2.41% spread proves REPEATABILITY, not
 * correctness - all four points sit inside a 12% lever arm in current, which
 * cannot test the zero-intercept assumption in R = V/I. What earns the trust is
 * the independent physical prior: the part is an R002 (2 mOhm) shunt and this
 * is +7.9% over nominal, the right scale for a non-Kelvin trace/solder
 * parasitic. The superseded R_eff = 3.48 mOhm was +74% over nominal, which was
 * never plausible; it was fit at 0.09-0.15 A where one PSU count (0.01 A) is
 * ~10% of the reading.
 *
 * Full record: docs/ina226/CALIB_r_shunt_result.md
 *--------------------------------------------------------------------------*/
#define INA226_R_SHUNT_MOHM         (2.16f)

/* Set to STD_ON to mark every sample INA226_FLAG_UNCAL - i.e. "publish voltage,
 * do not trust current". OFF because R_shunt above is measured, not assumed.
 * Turn it back ON if the shunt hardware is changed/reworked, or if a future
 * loaded high-current point disagrees with the low-current characterisation. */
#define INA226_R_SHUNT_PROVISIONAL  STD_OFF

/*******************************************************************************
 *                          Fixed device scaling                               *
 *                                                                             *
 * Datasheet constants, NOT board configuration - here (rather than in the .c)  *
 * only because the API contract quotes them. Do not "tune" these.             *
 *******************************************************************************/

#define INA226_SHUNT_LSB_UV         (2.5f)      /* 2.5 uV  per SHUNT_V LSB */
#define INA226_BUS_LSB_V            (0.00125f)  /* 1.25 mV per BUS_V   LSB */

/* SHUNT ADC full scale is +/-81.92 mV = +/-0x7FFF. Past ~99.8% of that the
 * reading is pinned at the rail and UNDER-reports; flagged as SHUNT_SAT.
 * At R_shunt = 2.16 mOhm this rail corresponds to ~37.9 A. */
#define INA226_SHUNT_SAT_RAW        (32700)

/*******************************************************************************
 *                          Transport timeout                                  *
 *******************************************************************************/

/* Per-command TIMER0-tick cap handed to every I2C call (see i2c.h). The 200 us
 * default was VALIDATED by measurement on 2026-08-04: a healthy command on this
 * bus is ~60 us, so the cap is ~3.3x headroom (i2c.h's comment still says
 * "~45 us", which is the stale figure - the cap itself is correct). */
#define INA226_I2C_CAP_TICKS        I2C_CAP_DEFAULT_TICKS

#endif /* INA226_CFG_H_ */
