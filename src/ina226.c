/******************************************************************************
 *
 * Module: INA226 (MCAL)
 *
 * File Name: ina226.c
 *
 * Description: Implementation of the INA226 current/voltage driver. Contract,
 *              rationale and the caller's obligations are in ina226.h - read
 *              that first; this file holds the mechanics.
 *
 ******************************************************************************/

#include "ina226.h"
#include "ina226_private.h"
#include "i2c.h"

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

static boolean             g_Ina226_Initialized  = FALSE;

/* Sticky transport error behind the most recent INA226_FLAG_I2C_FAIL. See
 * Ina226_GetLastI2cError(): the point is to preserve WHICH failure it was,
 * because NO_ACK and BUS_STUCK point at completely different faults. */
static Ina226_I2cErrorType g_Ina226_LastI2cError = I2C_OK;

/* Last SUCCESSFUL acquisition. Two jobs:
 *   1. it is what a failed ReadAll hands back, so *out is never garbage;
 *   2. its raw registers feed the duplicate-sample (CVRF_STALE) inference.
 * All-zero until the first success - and `seq` staying 0 is precisely how a
 * caller can tell it is holding a value the driver never measured. */
static Ina226_SampleType   g_Ina226_LastGood     = { 0, 0, 0, INA226_FLAG_NONE, 0U };

/* Raw registers behind g_Ina226_LastGood, kept for the byte-identical
 * comparison. Held separately rather than reconstructed from the converted
 * values: the conversions are lossy (see the truncation note in
 * Ina226_ShuntRawToMicrovolts), so comparing converted values would miss
 * changes smaller than one output unit. */
static uint16              g_Ina226_LastBusRaw   = 0U;
static uint16              g_Ina226_LastShuntRaw = 0U;
static boolean             g_Ina226_HaveLastRaw  = FALSE;

/*******************************************************************************
 *                          Register access                                    *
 *******************************************************************************/

Ina226_I2cErrorType Ina226_WriteReg16(uint8 reg, uint16 val)
{
    uint8 buf[2];
    Ina226_I2cErrorType s;

    buf[0] = (uint8)((val >> 8) & 0xFFU);   /* MSB first - big-endian wire */
    buf[1] = (uint8)(val & 0xFFU);

    s = I2C_Write(INA226_I2C_ID, INA226_ADDR_7BIT, reg, buf, 2U, INA226_I2C_CAP_TICKS);
    if (s != I2C_OK)
    {
        g_Ina226_LastI2cError = s;
    }
    return s;
}

Ina226_I2cErrorType Ina226_ReadReg16(uint8 reg, uint16 *out)
{
    uint8 buf[2] = { 0U, 0U };
    Ina226_I2cErrorType s;

    if (out == NULL_PTR)
    {
        return I2C_ERROR_NULL_PTR;
    }

    s = I2C_Read(INA226_I2C_ID, INA226_ADDR_7BIT, reg, buf, 2U, INA226_I2C_CAP_TICKS);
    if (s == I2C_OK)
    {
        *out = ((uint16)buf[0] << 8) | (uint16)buf[1];   /* MSB first */
    }
    else
    {
        g_Ina226_LastI2cError = s;   /* *out deliberately untouched */
    }
    return s;
}

/*******************************************************************************
 *                          Conversions                                        *
 *                                                                             *
 * Kept in INTEGER arithmetic wherever the scaling is an exact ratio. Both      *
 * device LSBs happen to be exact binary-friendly fractions, so the only place  *
 * a float is genuinely needed is the division by R_shunt.                     *
 *******************************************************************************/

/**
 * @brief BUS_V raw -> millivolts. UNSIGNED register.
 * @note  1.25 mV/LSB = 5/4 exactly, so this is exact with no float and no
 *        rounding. Max raw 0xFFFF -> 81,918 mV, nowhere near sint32 overflow.
 */
static sint32 Ina226_BusRawToMillivolts(uint16 raw)
{
    return ((sint32)raw * 5) / 4;
}

/**
 * @brief SHUNT_V raw -> microvolts. SIGNED register - the cast is the whole
 *        point: a discharging/charging sign error here would invert current.
 * @note  2.5 uV/LSB = 5/2 exactly. Max |raw| 32768 -> 81,920 uV, no overflow.
 *        TRUNCATION: for ODD raw values the /2 truncates toward zero, losing up
 *        to 0.5 uV (= 0.23 mA at R_shunt = 2.16 mOhm). That is ~30x smaller
 *        than one ADC count and ~4 orders below the +/-5-7% calibration
 *        uncertainty, so it is left as truncation rather than carrying a
 *        rounding term that would only add noise to reason about.
 */
static sint32 Ina226_ShuntRawToMicrovolts(uint16 raw)
{
    sint16 signedRaw = (sint16)raw;         /* reinterpret: SHUNT_V is signed */
    return ((sint32)signedRaw * 5) / 2;
}

/**
 * @brief shunt microvolts -> milliamps, via the single calibration constant.
 * @note  uV / mOhm == mA EXACTLY - no scale factor hides in this line.
 *        Rounds half away from zero so a positive and a negative current of the
 *        same magnitude quantise symmetrically; truncation would bias every
 *        reading toward zero and, on a signed quantity, that bias would flip
 *        sign at zero crossing.
 */
static sint32 Ina226_MicrovoltsToMilliamps(sint32 shunt_uV)
{
    float32 mA = (float32)shunt_uV / INA226_R_SHUNT_MOHM;

    return (mA >= 0.0f) ? (sint32)(mA + 0.5f) : (sint32)(mA - 0.5f);
}

/*******************************************************************************
 *                          Public API                                         *
 *******************************************************************************/

Std_ReturnType Ina226_Init(void)
{
    uint16 rb = 0U;

    g_Ina226_Initialized = FALSE;

    /* --- 1. Identity: is the expected part even answering? --- */
    if (Ina226_ReadReg16(INA226_REG_MANUF_ID, &rb) != I2C_OK)
    {
        return E_NOT_OK;
    }
    if (rb != INA226_MANUF_ID_EXPECTED)
    {
        return E_NOT_OK;   /* answered, but it is not an INA226 */
    }

    /* --- 2. CONFIG, write then verify --- */
    if (Ina226_WriteReg16(INA226_REG_CONFIG, INA226_CONFIG) != I2C_OK)
    {
        return E_NOT_OK;
    }
    if (Ina226_ReadReg16(INA226_REG_CONFIG, &rb) != I2C_OK)
    {
        return E_NOT_OK;
    }
    if ((rb & INA226_CONFIG_RST_BIT) != 0U)
    {
        return E_NOT_OK;   /* RST set -> the part self-resets and never converges */
    }
    if (rb != INA226_CONFIG)
    {
        return E_NOT_OK;
    }

    /* --- 3. CALIBRATION, write then verify.
     * Not used by this driver's maths (see ina226.h) - programmed so the chip's
     * own CURRENT/POWER registers are sane for anyone who peeks. Verified
     * because a failed write reads back as 0, silently. --- */
    if (Ina226_WriteReg16(INA226_REG_CALIB, INA226_CAL_WORD) != I2C_OK)
    {
        return E_NOT_OK;
    }
    if (Ina226_ReadReg16(INA226_REG_CALIB, &rb) != I2C_OK)
    {
        return E_NOT_OK;
    }
    if (rb != INA226_CAL_WORD)
    {
        return E_NOT_OK;
    }

    /* --- 4. One MASK_EN read: clears any stale CVRF so the duplicate-sample
     * inference starts from a known state. The ONLY MASK_EN read in the whole
     * driver - it is deliberately absent from the hot path. --- */
    (void)Ina226_ReadReg16(INA226_REG_MASK_EN, &rb);

    /* A fresh Init invalidates the previous device state's raw history. */
    g_Ina226_HaveLastRaw = FALSE;

    g_Ina226_Initialized = TRUE;
    return E_OK;
}

Std_ReturnType Ina226_ReadAll(Ina226_SampleType *out)
{
    uint16  busRaw   = 0U;
    uint16  shuntRaw = 0U;
    uint8   flags    = INA226_FLAG_NONE;
    sint16  shuntSigned;

    if (out == NULL_PTR)
    {
        return E_NOT_OK;
    }

#if (INA226_R_SHUNT_PROVISIONAL == STD_ON)
    flags |= (uint8)INA226_FLAG_UNCAL;
#endif

    if (g_Ina226_Initialized == FALSE)
    {
        *out = g_Ina226_LastGood;                    /* all-zero, seq 0 */
        out->flags = (uint8)(flags | (uint8)INA226_FLAG_I2C_FAIL);
        return E_NOT_OK;
    }

    /* --- The two reads. Exactly two: 6 capped commands, ~298 us measured. --- */
    if (Ina226_ReadReg16(INA226_REG_BUS_V, &busRaw) != I2C_OK)
    {
        *out = g_Ina226_LastGood;                    /* last good, never garbage */
        out->flags = (uint8)(flags | (uint8)INA226_FLAG_I2C_FAIL);
        return E_NOT_OK;                             /* NO re-Init here - R22-3 */
    }
    if (Ina226_ReadReg16(INA226_REG_SHUNT_V, &shuntRaw) != I2C_OK)
    {
        *out = g_Ina226_LastGood;
        out->flags = (uint8)(flags | (uint8)INA226_FLAG_I2C_FAIL);
        return E_NOT_OK;
    }

    /* --- Duplicate-sample inference, free: identical raws MAY be one
     * conversion seen twice. On a steady rail they are also simply the correct
     * answer, hence "possibly" - see INA226_FLAG_CVRF_STALE. --- */
    if ((g_Ina226_HaveLastRaw != FALSE) &&
        (busRaw == g_Ina226_LastBusRaw) && (shuntRaw == g_Ina226_LastShuntRaw))
    {
        flags |= (uint8)INA226_FLAG_CVRF_STALE;
    }

    /* --- Shunt ADC at the rail: the reading is clipped and UNDER-reports, so
     * it lies in the dangerous direction (low current exactly when the real
     * current is highest). --- */
    shuntSigned = (sint16)shuntRaw;
    if ((shuntSigned >= INA226_SHUNT_SAT_RAW) || (shuntSigned <= -INA226_SHUNT_SAT_RAW))
    {
        flags |= (uint8)INA226_FLAG_SHUNT_SAT;
    }

    /* --- Convert. One coherent snapshot: every field below comes from the two
     * reads above, so V and I describe the same instant. --- */
    out->bus_mV     = Ina226_BusRawToMillivolts(busRaw);
    out->shunt_uV   = Ina226_ShuntRawToMicrovolts(shuntRaw);
    out->current_mA = Ina226_MicrovoltsToMilliamps(out->shunt_uV);
    out->flags      = flags;
    out->seq        = (uint8)(g_Ina226_LastGood.seq + 1U);   /* wraps at 255 */

    g_Ina226_LastGood     = *out;
    g_Ina226_LastBusRaw   = busRaw;
    g_Ina226_LastShuntRaw = shuntRaw;
    g_Ina226_HaveLastRaw  = TRUE;

    return E_OK;
}

Ina226_I2cErrorType Ina226_GetLastI2cError(void)
{
    return g_Ina226_LastI2cError;
}
