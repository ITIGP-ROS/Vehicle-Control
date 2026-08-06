/******************************************************************************
 *
 * Module: BatteryService (application / service layer)
 *
 * File Name: battery_service.c
 *
 * Description: Hybrid coulomb-counting / voltage-re-anchoring fuel gauge.
 *              The contract and the reasoning behind the hybrid are in
 *              battery_service.h - read that first; this file is the mechanics.
 *
 ******************************************************************************/

#include "battery_service.h"
#include "ina226.h"

/*----------------------------------------------------------------------------
 * B4 (2026-08-06): the publish/read boundary became CROSS-TASK.
 *
 * Until B4 this whole module ran in one context - the super-loop called
 * Update() in slot 43 and GetStatus() in slot 47, so nothing could interleave.
 * B4 peels Update() into tBattery (prio 1) while the 0x210 transmit stays in
 * tSuperLoop (prio 2), so the reader now PREEMPTS the writer and the 24-byte
 * BatteryStatusType is exposed to tearing.
 *
 * These macros mark the two places that must commit/read the struct whole.
 * They are NO-OPS in every non-RTOS build (the 18 bench envs and the
 * battery_service_test harness), which therefore keep byte-identical behaviour.
 *
 * ⚠️ Deliberately NOT a mutex. tBattery is the LOWEST-priority task in the
 * system; a mutex would let it block the loop that drives the wheels. A brief
 * BASEPRI raise cannot be held across a context switch, cannot block, and
 * cannot invert priorities.
 *--------------------------------------------------------------------------*/
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#define BATT_PUBLISH_ENTER()   taskENTER_CRITICAL()
#define BATT_PUBLISH_EXIT()    taskEXIT_CRITICAL()
#else
#define BATT_PUBLISH_ENTER()   do { } while (0)
#define BATT_PUBLISH_EXIT()    do { } while (0)
#endif

/*******************************************************************************
 *                          The charge accumulator's unit                      *
 *******************************************************************************/

/*----------------------------------------------------------------------------
 * Charge is accumulated in mA-TICKS: one Update at current I contributes
 * exactly I. No multiply, no divide, no rounding - the Ts factor is folded once
 * into the capacity constant instead of being applied on every one of the
 * ~720 million integration steps of a full discharge.
 *
 * That is the answer to the roundoff problem (P5). The obvious formulation,
 * `charge_mAs -= current_mA * 0.1f`, truncates or accumulates float error on
 * EVERY tick; at 10 Hz that is 864,000 opportunities per day to lose charge in
 * one direction. Here each step is an exact integer subtraction.
 *
 * SIZING - deliberately sint32, not sint64:
 *     20000 mAh x 3600 s/h x 10 ticks/s = 720,000,000  <  2,147,483,647
 * so a full pack fits with ~3x headroom. This matters beyond elegance: 64-bit
 * DIVISION on this toolchain pulls in __aeabi_uldivmod, whose .ARM.exidx
 * section has no home in linker/tm4c123.ld and overlapped .data the last time
 * it was linked in (see the FIX 24 note in docs/MEMORY.md). Staying in 32 bits
 * sidesteps a known linker gap rather than re-opening it.
 *
 * The static assert below is what keeps that reasoning honest if someone fits
 * a bigger pack: BATTERY_CAPACITY_MAH beyond ~59 Ah overflows sint32 here, and
 * this build must fail loudly rather than wrap silently.
 *--------------------------------------------------------------------------*/
#define BATTERY_CAPACITY_MA_TICKS \
    ((sint32)(BATTERY_CAPACITY_MAH * 3600L * BATTERY_TICKS_PER_SECOND))

/* Negative-array-size assert: integer constant expression, so it works on this
 * C90/GCC 4.8.4 toolchain (no _Static_assert). A too-large capacity makes the
 * dimension negative -> "size of array is negative" at compile time. */
typedef char BatteryService_AssertCapacityFitsSint32[
    ((BATTERY_CAPACITY_MAH > 0L) &&
     (BATTERY_CAPACITY_MAH <= (2147483647L / (3600L * BATTERY_TICKS_PER_SECOND)))) ? 1 : -1];

/*******************************************************************************
 *                          Private State                                      *
 *******************************************************************************/

static BatteryStatusType g_Batt_Status;          /* the published snapshot     */
static boolean           g_Batt_Initialized  = FALSE;
static boolean           g_Batt_Anchored     = FALSE;
static boolean           g_Batt_LastReadOk   = FALSE;

/* Coulomb accumulator, mA-ticks. Meaningful only once g_Batt_Anchored. */
static sint32            g_Batt_ChargeMaTicks = 0;

/* Voltage filter: median-3 window over IR-compensated voltage, then EMA. */
static sint32            g_Batt_VocWindow[3]  = { 0, 0, 0 };
static uint8             g_Batt_VocCount      = 0U;   /* saturates at 3        */
static float32           g_Batt_VocEma        = 0.0f;
static boolean           g_Batt_EmaSeeded     = FALSE;

/* Rest detection. */
static sint32            g_Batt_RestTicks     = 0;

/* Range inputs, smoothed - raw power/speed are far too jittery for 1 km/LSB. */
static float32           g_Batt_PowerEma      = 0.0f;
static float32           g_Batt_SpeedEma      = 0.0f;
static boolean           g_Batt_RangeSeeded   = FALSE;

/* The LUT (ascending in voltage - the interpolation depends on it). */
static const sint32  g_Batt_LutMv[BATTERY_SOC_LUT_POINTS]  = BATTERY_SOC_LUT_MV;
static const float32 g_Batt_LutPct[BATTERY_SOC_LUT_POINTS] = BATTERY_SOC_LUT_PCT;

/*******************************************************************************
 *                          Private Helpers                                    *
 *******************************************************************************/

static float32 BatteryService_AbsF(float32 v)
{
    return (v < 0.0f) ? -v : v;
}

static sint32 BatteryService_AbsI(sint32 v)
{
    return (v < 0) ? -v : v;
}

static float32 BatteryService_ClampPct(float32 pct)
{
    if (pct < 0.0f)   { return 0.0f;   }
    if (pct > 100.0f) { return 100.0f; }
    return pct;
}

/**
 * @brief Open-circuit voltage estimate: undo the IR drop the load causes.
 * @note  current_mA x R_mOhm is microvolts, /1000 gives millivolts. Discharge
 *        is POSITIVE, and a discharging pack sags, so the correction ADDS -
 *        get this sign wrong and the compensation doubles the error instead of
 *        removing it.
 */
static sint32 BatteryService_VocMillivolts(sint32 bus_mV, sint32 current_mA)
{
    float32 dropMv = ((float32)current_mA * BATTERY_R_SOURCE_MOHM) / 1000.0f;
    float32 voc    = (float32)bus_mV + dropMv;

    return (voc >= 0.0f) ? (sint32)(voc + 0.5f) : (sint32)(voc - 0.5f);
}

/**
 * @brief Median of the 3-sample window - spike rejection BEFORE the EMA.
 * @note  A median REJECTS an outlier; an EMA would SMEAR it across the
 *        following seconds. Order matters, not just presence.
 */
static sint32 BatteryService_Median3(void)
{
    sint32 a = g_Batt_VocWindow[0];
    sint32 b = g_Batt_VocWindow[1];
    sint32 c = g_Batt_VocWindow[2];

    if (g_Batt_VocCount < 3U)
    {
        return g_Batt_VocWindow[0];   /* newest; window not full yet */
    }

    if (((a <= b) && (b <= c)) || ((c <= b) && (b <= a))) { return b; }
    if (((b <= a) && (a <= c)) || ((c <= a) && (a <= b))) { return a; }
    return c;
}

/**
 * @brief Voltage -> SoC %, linear interpolation between LUT breakpoints,
 *        clamped outside the table.
 */
static float32 BatteryService_SocFromVoltage(sint32 mv)
{
    uint8 i;

    if (mv <= g_Batt_LutMv[0])
    {
        return g_Batt_LutPct[0];
    }
    if (mv >= g_Batt_LutMv[BATTERY_SOC_LUT_POINTS - 1U])
    {
        return g_Batt_LutPct[BATTERY_SOC_LUT_POINTS - 1U];
    }

    for (i = 1U; i < BATTERY_SOC_LUT_POINTS; i++)
    {
        if (mv <= g_Batt_LutMv[i])
        {
            sint32  spanMv = g_Batt_LutMv[i] - g_Batt_LutMv[i - 1U];
            float32 frac   = (spanMv != 0)
                             ? ((float32)(mv - g_Batt_LutMv[i - 1U]) / (float32)spanMv)
                             : 0.0f;
            return g_Batt_LutPct[i - 1U] +
                   (frac * (g_Batt_LutPct[i] - g_Batt_LutPct[i - 1U]));
        }
    }

    return g_Batt_LutPct[BATTERY_SOC_LUT_POINTS - 1U];   /* unreachable */
}

/* SoC % <-> accumulator, the two directions of the same conversion. */
static float32 BatteryService_SocFromCharge(sint32 chargeMaTicks)
{
    return BatteryService_ClampPct(
        ((float32)chargeMaTicks / (float32)BATTERY_CAPACITY_MA_TICKS) * 100.0f);
}

static sint32 BatteryService_ChargeFromSoc(float32 pct)
{
    float32 q = (BatteryService_ClampPct(pct) / 100.0f) *
                (float32)BATTERY_CAPACITY_MA_TICKS;
    return (sint32)(q + 0.5f);
}

/**
 * @brief Push one IR-compensated sample through median-3 -> EMA.
 * @return the filtered voltage in mV.
 */
static float32 BatteryService_FilterVoltage(sint32 vocMv)
{
    sint32 med;

    g_Batt_VocWindow[2] = g_Batt_VocWindow[1];
    g_Batt_VocWindow[1] = g_Batt_VocWindow[0];
    g_Batt_VocWindow[0] = vocMv;
    if (g_Batt_VocCount < 3U) { g_Batt_VocCount++; }

    med = BatteryService_Median3();

    if (g_Batt_EmaSeeded == FALSE)
    {
        /* Seed with the first sample rather than 0 - otherwise SoC would ramp
         * up from "empty" over the first ~15 s of every boot, which looks like
         * a fault and would poison the boot anchor. */
        g_Batt_VocEma    = (float32)med;
        g_Batt_EmaSeeded = TRUE;
    }
    else
    {
        g_Batt_VocEma += BATTERY_VOLT_EMA_ALPHA * ((float32)med - g_Batt_VocEma);
    }

    return g_Batt_VocEma;
}

/**
 * @brief Range estimate from remaining energy and recent consumption.
 * @return METRES, 0 when not estimable. (Was km until 2026-08-05; the
 *         estimation math is unchanged, only the output unit - the cluster
 *         renders metres because this robot's range is fractional in km.)
 */
static float32 BatteryService_EstimateRange(float32 socPct, float32 speedMps,
                                            float32 powerW)
{
    float32 remainingWh;
    float32 speedKmh;
    float32 km;
    float32 metres;

    /* ⚠️ `powerW` USED TO BE READ AS g_Batt_Status.power_W (B4, 2026-08-06).
     * It is now an explicit parameter, and that is a REQUIREMENT, not tidying.
     *
     * Update() previously assigned g_Batt_Status.power_W and then called this
     * function, which read the value back out of the published struct. B4 makes
     * the publish a SINGLE deferred struct commit (see Update), so the global
     * no longer holds this pass's power at the moment this runs - reading it
     * here would silently use the PREVIOUS pass's power in the EMA.
     *
     * Passing it in is exactly value-equivalent to the old read (the caller
     * hands over the very number it used to store first), so the EMA, the
     * seeding branch and every downstream metre are bit-identical. */
    if (g_Batt_RangeSeeded == FALSE)
    {
        g_Batt_PowerEma   = powerW;
        g_Batt_SpeedEma   = speedMps;
        g_Batt_RangeSeeded = TRUE;
    }
    else
    {
        g_Batt_PowerEma += BATTERY_RANGE_EMA_ALPHA * (powerW - g_Batt_PowerEma);
        g_Batt_SpeedEma += BATTERY_RANGE_EMA_ALPHA * (speedMps - g_Batt_SpeedEma);
    }

    /* Standing still, or charging: consumption per km is undefined, not zero.
     * Publishing a huge number would be worse than publishing none. */
    if ((g_Batt_SpeedEma < BATTERY_RANGE_MIN_SPEED_MPS) ||
        (g_Batt_PowerEma < BATTERY_RANGE_MIN_POWER_W))
    {
        return 0.0f;
    }

    /* Wh = (SoC/100) x Ah x V_nominal */
    remainingWh = (socPct / 100.0f) *
                  ((float32)BATTERY_CAPACITY_MAH / 1000.0f) *
                  ((float32)BATTERY_NOMINAL_MV / 1000.0f);

    speedKmh = g_Batt_SpeedEma * 3.6f;

    /* km = Wh / (W per km) = Wh / (W / km_per_h) = Wh x km_per_h / W */
    km = (remainingWh * speedKmh) / g_Batt_PowerEma;

    /* km -> m as the LAST step, so the energy arithmetic above stays in the
     * units its derivation is written in and only the published value changes. */
    metres = km * 1000.0f;

    if (metres < 0.0f)                   { metres = 0.0f; }
    if (metres > BATTERY_RANGE_MAX_M)    { metres = BATTERY_RANGE_MAX_M; }
    return metres;
}

/*******************************************************************************
 *                          Public API                                         *
 *******************************************************************************/

void BatteryService_Init(void)
{
    g_Batt_Status.voltage_mV = 0;
    g_Batt_Status.current_mA = 0;
    g_Batt_Status.soc_pct    = 0.0f;
    g_Batt_Status.power_W    = 0.0f;
    g_Batt_Status.range_m    = 0.0f;
    g_Batt_Status.flags      = (uint8)BATTERY_FLAG_NOT_ANCHORED;
    g_Batt_Status.seq        = 0U;

    g_Batt_ChargeMaTicks = 0;
    g_Batt_VocWindow[0]  = 0;
    g_Batt_VocWindow[1]  = 0;
    g_Batt_VocWindow[2]  = 0;
    g_Batt_VocCount      = 0U;
    g_Batt_VocEma        = 0.0f;
    g_Batt_EmaSeeded     = FALSE;
    g_Batt_RestTicks     = 0;
    g_Batt_PowerEma      = 0.0f;
    g_Batt_SpeedEma      = 0.0f;
    g_Batt_RangeSeeded   = FALSE;
    g_Batt_Anchored      = FALSE;
    g_Batt_LastReadOk    = FALSE;

    g_Batt_Initialized = (Ina226_Init() == E_OK) ? TRUE : FALSE;

    /* The anchor itself is taken by the first successful Update(): it needs a
     * sample, and Update() is the one place that acquires. Doing it here would
     * duplicate the whole acquire/compensate/filter path for no gain. */
}

void BatteryService_Update(float32 speed_mps)
{
    Ina226_SampleType sample;
    uint8             flags = (uint8)BATTERY_FLAG_NONE;
    float32           vFiltered;
    float32           socVolt;
    float32           socCoulomb;

#if (BATTERY_R_SOURCE_PROVISIONAL == STD_ON)
    flags |= (uint8)BATTERY_FLAG_UNCAL;
#endif

    /* ---- Acquire: exactly ONE ReadAll, never a re-Init (R22-3) ---- */
    if ((g_Batt_Initialized == FALSE) || (Ina226_ReadAll(&sample) != E_OK))
    {
        g_Batt_LastReadOk = FALSE;

        /* Hold the previous status. seq is NOT advanced, and - the part that
         * actually matters - the coulomb accumulator is NOT advanced. A bad
         * published sample is transient; a bad INTEGRATED sample is permanent. */
        g_Batt_Status.flags = (uint8)(flags | (uint8)BATTERY_FLAG_I2C_FAIL |
                                      ((g_Batt_Anchored == FALSE)
                                          ? (uint8)BATTERY_FLAG_NOT_ANCHORED
                                          : (uint8)0U));
        return;
    }
    g_Batt_LastReadOk = TRUE;

    if ((sample.flags & (uint8)INA226_FLAG_SHUNT_SAT) != 0U)
    {
        flags |= (uint8)BATTERY_FLAG_SHUNT_SAT;
    }

    /* ---- Voltage: IR-compensate, then median-3 -> EMA, then look up ----
     * Filter the VOLTAGE and look up SoC afterwards, never the reverse: the
     * LUT is non-linear, so filtering SoC would weight the knees differently
     * from the flat middle. */
    vFiltered = BatteryService_FilterVoltage(
                    BatteryService_VocMillivolts(sample.bus_mV, sample.current_mA));
    socVolt   = BatteryService_SocFromVoltage((sint32)(vFiltered + 0.5f));

    /* ---- Anchor on the first good sample ---- */
    if (g_Batt_Anchored == FALSE)
    {
        g_Batt_ChargeMaTicks = BatteryService_ChargeFromSoc(socVolt);
        g_Batt_Anchored      = TRUE;
    }
    else
    {
        /* ---- Coulomb integration. One exact integer step, no scaling.
         * Discharge is POSITIVE per the DBC, and discharging removes charge. */
        g_Batt_ChargeMaTicks -= sample.current_mA;

        if (g_Batt_ChargeMaTicks < 0)                        { g_Batt_ChargeMaTicks = 0; }
        if (g_Batt_ChargeMaTicks > BATTERY_CAPACITY_MA_TICKS)
        {
            g_Batt_ChargeMaTicks = BATTERY_CAPACITY_MA_TICKS;
        }
    }

    /* ---- Rest detection and re-anchor: the drift corrector ---- */
    if (BatteryService_AbsI(sample.current_mA) < BATTERY_REST_CURRENT_MA)
    {
        if (g_Batt_RestTicks < BATTERY_REST_TICKS) { g_Batt_RestTicks++; }
    }
    else
    {
        g_Batt_RestTicks = 0;   /* any load restarts the settling window */
    }

    socCoulomb = BatteryService_SocFromCharge(g_Batt_ChargeMaTicks);

    if (g_Batt_RestTicks >= BATTERY_REST_TICKS)
    {
        float32 diff = socVolt - socCoulomb;

        if (BatteryService_AbsF(diff) >= BATTERY_REANCHOR_SNAP_PCT)
        {
            /* Too far apart to be integrator drift - a missed charge session, a
             * pack swap, a reset mid-discharge. Blending would take minutes to
             * admit an error this size, so snap (rest is already confirmed). */
            g_Batt_ChargeMaTicks = BatteryService_ChargeFromSoc(socVolt);
        }
        else
        {
            /* Ordinary drift: blend, so one noisy voltage sample can never
             * rewrite the charge state. */
            g_Batt_ChargeMaTicks = BatteryService_ChargeFromSoc(
                                       socCoulomb + (BATTERY_REANCHOR_ALPHA * diff));
        }
        socCoulomb = BatteryService_SocFromCharge(g_Batt_ChargeMaTicks);
    }

    if (BatteryService_AbsF(socVolt - socCoulomb) >= BATTERY_DIVERGED_PCT)
    {
        flags |= (uint8)BATTERY_FLAG_DIVERGED;
    }

    /* ---- Publish: every field below comes from THIS pass ----
     *
     * ⚠️ BUILT INTO A LOCAL, THEN COMMITTED IN ONE ASSIGNMENT (B4, 2026-08-06).
     * This used to write the seven fields straight into g_Batt_Status one at a
     * time. That was harmless while a single super-loop context did both the
     * writing and the reading - and it stopped being harmless the moment B4
     * peeled Update() into its own task:
     *
     *   tBattery (prio 1) writes  ->  tSuperLoop (prio 2) reads via GetStatus
     *
     * The reader has the HIGHER priority, so it can preempt the writer at any
     * instruction. Field-by-field, a reader landing mid-publish would get this
     * pass's voltage and current stapled to the previous pass's soc, range and
     * flags - a snapshot that never existed, with no flag to say so. Exactly
     * the S10-1 defect, one service over.
     *
     * NO MUTEX, deliberately: a mutex would let the lowest-priority task in the
     * system block the loop that drives the wheels. The publish is ~24 bytes at
     * 10 Hz, so the right primitive is a coherent commit, not a lock. */
    {
        BatteryStatusType published;

        published.voltage_mV = sample.bus_mV;   /* what the sensor saw, not V_oc */
        published.current_mA = sample.current_mA;

        /* W = mV x mA / 1e6. Sign follows current: + discharge, - charge. NEVER
         * inferred from commanded duty - current is non-monotonic in duty (back-EMF,
         * measured 2026-08-04), so a command-derived power would be wrong exactly
         * where the command is largest. */
        published.power_W  = ((float32)sample.bus_mV * (float32)sample.current_mA)
                             / 1000000.0f;
        published.soc_pct  = socCoulomb;

        /* power is PASSED now, not read back out of g_Batt_Status - the global
         * does not hold this pass's value until the commit below. See the note
         * in BatteryService_EstimateRange. */
        published.range_m  = BatteryService_EstimateRange(socCoulomb, speed_mps,
                                                          published.power_W);
        published.flags    = flags;
        published.seq      = (uint8)(g_Batt_Status.seq + 1U);

        /* ⚠️ The critical section is NOT decoration. `g_Batt_Status = published`
         * is a 24-byte struct copy, which on Cortex-M4 is a multi-instruction
         * sequence (LDM/STM or memcpy) - NOT atomic. Without the guard the
         * commit is just as tearable as the seven separate stores were.
         *
         * A CRITICAL SECTION IS NOT A MUTEX: it briefly raises BASEPRI, cannot
         * be held across a context switch, cannot block a task, and cannot
         * invert priorities. Cost is ~20-30 cycles = under 2 us at 16 MHz,
         * against a 100 ms period. */
        BATT_PUBLISH_ENTER();
        g_Batt_Status = published;
        BATT_PUBLISH_EXIT();
    }
}

Std_ReturnType BatteryService_GetStatus(BatteryStatusType *out)
{
    if (out == NULL_PTR)
    {
        return E_NOT_OK;
    }

    /* Guarded for the same reason the publish is: a 24-byte struct copy is not
     * atomic on Cortex-M4.
     *
     * ℹ️ Strictly, TODAY the reader (tSuperLoop, prio 2) outranks the writer
     * (tBattery, prio 1), so the writer can never preempt a read and this side
     * alone would be safe. It is guarded anyway because that safety is an
     * accident of the current priority table, not a property of this module -
     * and the port is about to move several tasks around. Symmetric guarding
     * makes the contract hold whatever the priorities become. Cost: <2 us. */
    BATT_PUBLISH_ENTER();
    *out = g_Batt_Status;
    BATT_PUBLISH_EXIT();

    return E_OK;
}

boolean BatteryService_IsHealthy(void)
{
    return ((g_Batt_Initialized != FALSE) &&
            (g_Batt_LastReadOk  != FALSE) &&
            (g_Batt_Anchored    != FALSE)) ? TRUE : FALSE;
}
