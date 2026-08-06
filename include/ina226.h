#ifndef INA226_H_
#define INA226_H_

/******************************************************************************
 *
 * Module: INA226 (MCAL - current/voltage sense over I2C)
 *
 * File Name: ina226.h
 *
 * Description: Driver for the TI INA226 bus-voltage / shunt-voltage monitor on
 *              I2C0 (PB2/PB3, address 0x40). Supplies the raw electrical truth
 *              that battery_service turns into SoC / power / range for CAN
 *              0x210. This layer does NO filtering, NO state-of-charge, NO
 *              unit-policy - it acquires and converts, nothing more.
 *
 * -----------------------------------------------------------------------------
 *  CURRENT IS COMPUTED IN SOFTWARE - THE CURRENT REGISTER IS NEVER READ.
 *
 *      current_mA = shunt_uV / R_shunt_mOhm
 *
 *  SHUNT_V (0x01) and BUS_V (0x02) are raw ADC results, completely independent
 *  of the CALIBRATION register. So BOTH numbers this driver publishes are
 *  CAL-independent, and the CAL word - which was derived from weak low-current
 *  points and marked "DIAGNOSTIC ONLY" - leaves the trust path entirely. That
 *  is REVIEW 12's blocking finding I12-2, dissolved at the root rather than
 *  worked around. The single remaining calibration term is R_shunt, measured
 *  2026-08-04 (docs/ina226/CALIB_r_shunt_result.md).
 *
 *  The raw shunt_uV is returned ALONGSIDE current_mA precisely so R_shunt can
 *  be re-characterised later without a change to this arithmetic.
 * -----------------------------------------------------------------------------
 *
 *  ONE SNAPSHOT, NO PER-QUANTITY GETTERS - AND THAT IS DELIBERATE.
 *
 *  There is intentionally no Ina226_GetBusVoltage() / Ina226_GetCurrent().
 *  Separate getters make a CONSISTENT read impossible: 0x210's power field is
 *  V x I, so two getters would multiply two values sampled at different
 *  instants and publish a product that never existed. This is finding S10-1 /
 *  V9-R1, which the Tier-2 services hit twice and had to retrofit; this module
 *  is built with the answer already in place. Ina226_ReadAll() hands back one
 *  coherent Ina226_SampleType, and the caller reads it as a unit.
 *
 * -----------------------------------------------------------------------------
 *  PREREQUISITES, IN THIS ORDER, BEFORE Ina226_Init():
 *
 *      Port_Init()  ->  Timer0_FreeRunning_Init()  ->  I2C_Init()
 *
 *  Ina226_Init() itself is agnostic about TIMER0 - it never touches it. But
 *  every I2C command is capped in TIMER0 ticks, and with TIMER0 stopped GPTMTAR
 *  is static, the cap never elapses, and the FIRST read hangs forever. This is
 *  a caller/startup obligation (a main.c wiring requirement, restated here
 *  because that is where it gets forgotten), not something this driver can
 *  check without reaching outside its layer.
 * -----------------------------------------------------------------------------
 *
 *  COST AND CADENCE. Ina226_ReadAll() performs exactly TWO register reads
 *  (BUS_V then SHUNT_V) = 6 capped I2C commands. Measured on this bus
 *  (2026-08-04): ~298 us typical, ~393 us worst observed; the absolute ceiling
 *  if every command timed out is 6 x 200 us = 1.2 ms. It contains NO delay and
 *  does exactly one acquisition per call - the caller owns the cadence, at
 *  ~10 Hz against the 75.3 ms conversion cycle.
 *
 *  IT NEVER RE-INITIALISES ON FAILURE (R22-3). Ina226_Init() costs 6 capped
 *  commands of its own; running that inline from a failed read is the bench
 *  pattern REVIEW 22 explicitly banned from anything entering the super-loop,
 *  because it would blow the 1 ms slot budget. Recovery policy - retry, rate
 *  limit, re-Init, declare the sensor dead - belongs to the CALLER.
 *
 ******************************************************************************/

#include "Std_Types.h"
#include "ina226_types.h"
#include "ina226_cfg.h"

/*******************************************************************************
 *                          Function Prototypes                                *
 *******************************************************************************/

/**
 * @brief  Bring up the INA226: identity check, then CONFIG and CALIBRATION
 *         written and verified by read-back.
 *
 * @details Sequence (6 capped I2C commands):
 *            1. MANUF_ID (0xFE) must read INA226_MANUF_ID_EXPECTED
 *            2. write CONFIG, read back: RST bit must be CLEAR and the word
 *               must match exactly
 *            3. write CALIBRATION, read back exact match
 *            4. read MASK_EN once - this CLEARS any stale CVRF so the first
 *               ReadAll's duplicate detection starts from a known state
 *          A failed CAL write reads back as 0, which is why the read-back is
 *          not optional.
 *
 * @return E_OK only if every step succeeded. On E_NOT_OK call
 *         Ina226_GetLastI2cError() for the transport cause; note that a
 *         MISMATCH (device answered with the wrong value) leaves the last I2C
 *         error at I2C_OK, because nothing about the transport failed.
 *
 * @note   Re-runnable. Safe to call again after a bus recovery - but NOT from
 *         inside a failed ReadAll (see the R22-3 note above).
 */
Std_ReturnType Ina226_Init(void);

/**
 * @brief  One coherent acquisition: bus voltage, shunt voltage, and the
 *         software-derived current, all from the same instant.
 *
 * @param  out  destination snapshot. Must not be NULL.
 *
 * @return E_OK if BOTH register reads succeeded.
 *         E_NOT_OK on a NULL pointer, if Init has not run, or on any I2C
 *         failure.
 *
 * @note   ON FAILURE THE SNAPSHOT IS THE LAST GOOD ONE, NEVER GARBAGE. The
 *         driver keeps its own last-good sample and copies it into *out, with
 *         INA226_FLAG_I2C_FAIL set and `seq` NOT advanced. (Before the first
 *         successful read that last-good sample is all-zero, which is also why
 *         `seq` matters: a caller that has never seen seq change is holding a
 *         value the driver never measured.) So a caller can safely use *out
 *         unconditionally, and use `seq`/flags to decide how much to believe.
 *
 * @note   CVRF IS NOT POLLED. Reading MASK_EN to check conversion-ready would
 *         add a third read (+~50%) and, because that read CLEARS CVRF, would
 *         also destroy the flag for anyone else. Against a 75.3 ms conversion
 *         and a ~10 Hz caller, a duplicate sample costs less than the read used
 *         to detect it. Staleness is instead surfaced two ways that are free:
 *         `seq` (did the driver acquire?) and INA226_FLAG_CVRF_STALE (were the
 *         raw registers byte-identical to last time?).
 */
Std_ReturnType Ina226_ReadAll(Ina226_SampleType *out);

/**
 * @brief  Transport cause behind the most recent INA226_FLAG_I2C_FAIL.
 * @return Cached I2C_StatusType; I2C_OK if no transport error has occurred.
 * @note   STICKY: set only on failure, never cleared by a later success. Read
 *         it immediately after a call reports failure. Distinguishing
 *         I2C_ERROR_NO_ACK (wiring, address, power) from I2C_ERROR_TIMEOUT /
 *         BUS_STUCK (pull-ups, timing, a slave holding SDA) is the single most
 *         useful diagnostic bit when a board comes up quiet.
 */
Ina226_I2cErrorType Ina226_GetLastI2cError(void);

#endif /* INA226_H_ */
