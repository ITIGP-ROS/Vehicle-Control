#ifndef BATTERY_SERVICE_TYPES_H_
#define BATTERY_SERVICE_TYPES_H_

/******************************************************************************
 *
 * Module: BatteryService (application / service layer)
 *
 * File Name: battery_service_types.h
 *
 * Description: Public types for the battery service: the published status
 *              snapshot and its flag bitset.
 *
 ******************************************************************************/

#include "Platform_Types.h"

/*******************************************************************************
 *                          Status flags                                       *
 *******************************************************************************/

typedef enum
{
    BATTERY_FLAG_NONE          = 0x00U,

    /* The INA226 read failed this Update. The status is the LAST GOOD one,
     * `seq` did NOT advance, and - critically - the coulomb integrator was NOT
     * advanced either (integrating a garbage sample corrupts charge state
     * permanently, unlike a single bad published value). */
    BATTERY_FLAG_I2C_FAIL      = 0x01U,

    /* R_source is provisional, so the IR compensation - and therefore the
     * voltage anchor, and therefore SoC - is an ESTIMATE. Driven by
     * BATTERY_R_SOURCE_PROVISIONAL. Note this does NOT mean current is
     * uncalibrated: R_shunt was measured 2026-08-04. */
    BATTERY_FLAG_UNCAL         = 0x02U,

    /* Propagated from the driver: the shunt ADC is at its rail, so current is
     * CLIPPED and UNDER-reported. Both the published current and the coulomb
     * integration understate reality while this is set. */
    BATTERY_FLAG_SHUNT_SAT     = 0x04U,

    /* The coulomb-counted SoC and the voltage-LUT SoC disagree by more than
     * BATTERY_DIVERGED_PCT. Informational, not an error: it is the expected
     * state under heavy load (where the LUT is unreliable) and the signal that
     * a re-anchor is overdue if it persists AT REST. */
    BATTERY_FLAG_DIVERGED      = 0x08U,

    /* No successful sample has been taken yet, so the coulomb accumulator has
     * never been anchored to a physical measurement. SoC is meaningless until
     * this clears. Distinct from I2C_FAIL: that one means "we had a value and
     * lost it", this one means "we never had one". */
    BATTERY_FLAG_NOT_ANCHORED  = 0x10U
} BatteryService_FlagType;

/*******************************************************************************
 *                          Published status                                   *
 *******************************************************************************/

/**
 * @brief One coherent battery status: every field computed in the SAME Update.
 *
 * @details READ AS ONE UNIT. There are deliberately no per-quantity getters -
 *          `power_W` is voltage x current, so a torn read of V and I would
 *          publish a product of two different instants (S10-1 / V9-R1). The
 *          driver below this makes the same guarantee (Ina226_ReadAll).
 */
typedef struct
{
    sint32  voltage_mV;   /* measured bus voltage, mV (NOT IR-compensated -
                           * this is what the sensor saw, which is what the
                           * cluster should display; V_oc is internal to SoC) */
    sint32  current_mA;   /* signed; + = discharge, - = charge (DBC convention) */
    float32 soc_pct;      /* 0..100, HYBRID: coulomb-counted, voltage-re-anchored */
    float32 power_W;      /* bus_V x current_A; sign follows current            */
    float32 range_m;      /* METRES. 0 when not estimable (stationary/charging)  */
    uint8   flags;        /* BatteryService_FlagType bitset                      */
    uint8   seq;          /* ++ per SUCCESSFUL Update; frozen on a failed read   */
} BatteryStatusType;

#endif /* BATTERY_SERVICE_TYPES_H_ */
