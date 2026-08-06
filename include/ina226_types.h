#ifndef INA226_TYPES_H_
#define INA226_TYPES_H_

/******************************************************************************
 *
 * Module: INA226 (MCAL - current/voltage sense over I2C)
 *
 * File Name: ina226_types.h
 *
 * Description: Public types for the INA226 driver: the acquisition snapshot,
 *              its flag bitset, and the transport-error alias.
 *
 ******************************************************************************/

#include "Platform_Types.h"
#include "i2c_types.h"      /* I2C_StatusType - see Ina226_I2cErrorType below */

/*******************************************************************************
 *                          Sample flags                                       *
 *******************************************************************************/

/**
 * @brief Per-sample condition bits, OR-ed into Ina226_SampleType.flags.
 * @note  Stored in a uint8 field, not in an enum-typed field: the C90 enum type
 *        is implementation-defined in width, and this bitset is copied into a
 *        struct a CAN frame will eventually be packed from.
 */
typedef enum
{
    INA226_FLAG_NONE       = 0x00U,

    /* The two raw registers came back byte-identical to the previous successful
     * read, so this MAY be the same conversion seen twice. Costs NO extra I2C:
     * it is inferred in software, NOT read from MASK_EN/CVRF (see ina226.h for
     * why CVRF is not polled in the hot path).
     * HONEST LIMIT: on a genuinely steady rail identical raws are the CORRECT
     * reading, not an error - hence "possibly duplicate". Use `seq` to tell
     * "the driver acquired again" from "the value changed". */
    INA226_FLAG_CVRF_STALE = 0x01U,

    /* RESERVED - never set by Ina226_ReadAll(). OVF (MASK_EN bit 2) describes a
     * math overflow of the on-chip CURRENT/POWER registers, and this driver
     * computes current in SOFTWARE from SHUNT_V and never reads those registers
     * (DESIGN decision 1 / I12-2), so the condition cannot affect any value
     * published here. The bit position is retained rather than reused so an
     * old decode of these flags never silently means something new. */
    INA226_FLAG_OVF        = 0x02U,

    /* A register read failed during this ReadAll. The sample carries the LAST
     * GOOD values (never uninitialized data) and `seq` is NOT advanced. Call
     * Ina226_GetLastI2cError() for the transport cause. */
    INA226_FLAG_I2C_FAIL   = 0x04U,

    /* R_shunt is provisional, so current_mA is not trustworthy (bus_mV still
     * is - it is independent of R_shunt). Driven by INA226_R_SHUNT_PROVISIONAL
     * in ina226_cfg.h. Currently OFF: R_shunt was measured 2026-08-04. */
    INA226_FLAG_UNCAL      = 0x08U,

    /* The shunt ADC is at/near its +/-81.92 mV rail, so the reading is CLIPPED
     * and UNDER-reports the true current. Not in the original API sketch;
     * added because it is the one failure mode that lies in the DANGEROUS
     * direction - it reports a LOW current exactly when the real current is
     * highest. Anything acting on current (protection, power, coulomb
     * counting) must treat a set bit as "at least this much, direction
     * unknown", never as a measurement. */
    INA226_FLAG_SHUNT_SAT  = 0x10U
} Ina226_FlagType;

/*******************************************************************************
 *                          Acquisition snapshot                               *
 *******************************************************************************/

/**
 * @brief One coherent acquisition: every field comes from the SAME ReadAll.
 *
 * @details Integer milli-units, not float: these values are packed into CAN
 *          signals with integer scaling, and keeping the whole chain in
 *          integers removes a float round-trip and its rounding from the
 *          transport path. Conversion to float is the consumer's business.
 *
 *          THE POINT OF THIS STRUCT IS THAT IT IS READ AS ONE UNIT. There are
 *          deliberately no per-quantity getters - see ina226.h.
 */
typedef struct
{
    sint32 bus_mV;      /* bus voltage, mV.  BUS_V(0x02) x 1.25 mV, exact (x5/4) */
    sint32 shunt_uV;    /* shunt voltage, uV. SHUNT_V(0x01) x 2.5 uV, SIGNED     */
    sint32 current_mA;  /* SOFTWARE-derived: shunt_uV / R_shunt_mOhm, SIGNED     */
    uint8  flags;       /* Ina226_FlagType bitset                                */
    uint8  seq;         /* ++ on each SUCCESSFUL ReadAll; wraps at 255. Unchanged
                         * across a failed read, which is how a caller detects
                         * that it is holding a stale sample.                    */
} Ina226_SampleType;

/*******************************************************************************
 *                          Transport error                                    *
 *******************************************************************************/

/**
 * @brief Transport-layer cause behind INA226_FLAG_I2C_FAIL.
 * @note  Deliberately an ALIAS of I2C_StatusType rather than a parallel enum.
 *        A private taxonomy would need a mapping function, and every mapping is
 *        a place where NO_ACK (wiring/address/power) and TIMEOUT/BUS_STUCK
 *        (pull-ups, timing, a slave holding SDA) get flattened into one useless
 *        "failed". Same reasoning as MPU6050_GetLastI2cError().
 */
typedef I2C_StatusType Ina226_I2cErrorType;

#endif /* INA226_TYPES_H_ */
