#ifndef BATTERY_SERVICE_H_
#define BATTERY_SERVICE_H_

/******************************************************************************
 *
 * Module: BatteryService (application / service layer)
 *
 * File Name: battery_service.h
 *
 * Description: Fuel gauge over the INA226 driver. Turns raw electrical truth
 *              (bus voltage, shunt current) into state of charge, power and
 *              range for CAN 0x210.
 *
 *              Layer: above the MCAL. Includes ONLY ina226.h. It NEVER touches
 *              i2c.c, registers, CAN, or UART - packing and transmitting 0x210
 *              is cluster_comm's job, following the same build -> pack ->
 *              transmit pattern as VehicleStatus (0x200).
 *
 *              Shape mirrors imu_service / velocity_control / steering_control:
 *              Init() / Update() / Get*(), so a scheduler slot drops it in.
 *
 * -----------------------------------------------------------------------------
 *  SoC METHOD: HYBRID - COULOMB COUNTING, RE-ANCHORED BY VOLTAGE AT REST.
 *
 *  Neither method alone is adequate on this vehicle, for reasons that are
 *  measured, not assumed:
 *
 *   - VOLTAGE-ONLY is wrong under load. The supply path has ~0.9 Ohm of series
 *     resistance, so at 0.7 A the bus sags 0.63 V - and the LUT's mid-region
 *     spans only 0.6 V for 40 % of the pack. That is ~42 SoC points of error
 *     from load alone. IR compensation helps but inherits R_source's own
 *     uncertainty.
 *   - COULOMB-ONLY drifts. The current gain is good to +/-5-7 %, and that error
 *     INTEGRATES: about 6 SoC points over a full discharge. Nothing inside the
 *     integrator can see it happening.
 *
 *  So: the coulomb count carries short-term dynamics (it responds instantly and
 *  correctly to load, which is where voltage fails), and the IR-compensated
 *  voltage LUT re-anchors it whenever the pack is at rest (which is where the
 *  integrator's drift is correctable). Each covers precisely the other's blind
 *  spot. Full quantitative review: docs/ina226/REVIEW_coulomb_counting.md.
 *
 *  This is practical on THIS pack specifically because 20 Ah is a known spec
 *  anchor (not a guess), the 0.40C max rate means hours of runtime and ample
 *  rest opportunities, and the 3S BMS backstops both extremes so the gauge does
 *  not have to be right at 0 % / 100 %.
 * -----------------------------------------------------------------------------
 *
 *  CADENCE LIVES AT THE CALL SITE, AND IT IS LOAD-BEARING HERE.
 *  Update() contains no delay and performs exactly ONE Ina226_ReadAll
 *  (~312 us, 6 capped I2C commands). It must be called at
 *  BATTERY_TICKS_PER_SECOND (10 Hz). Unlike a filter, where a wrong rate only
 *  changes the smoothing, the coulomb integral SCALES DIRECTLY with the tick
 *  rate: call it at 20 Hz and the gauge drains at half speed, permanently and
 *  silently. Same class of invariant as velocity_control's Ts.
 *
 *  PREREQUISITES, in this order, before BatteryService_Init():
 *      Port_Init()  ->  Timer0_FreeRunning_Init()  ->  I2C_Init()
 *  Timer0 MUST precede I2C: every I2C command is TIMER0-tick capped, and with
 *  TIMER0 stopped the cap never elapses and the first read hangs forever.
 *
 *  R22-3: Update() NEVER re-initialises the sensor on a failed read.
 *
 ******************************************************************************/

#include "Std_Types.h"
#include "battery_service_types.h"
#include "battery_service_cfg.h"

/*******************************************************************************
 *                          Function Prototypes                                *
 *******************************************************************************/

/**
 * @brief  Bring up the gauge: init the INA226, clear filter state, and anchor
 *         the coulomb accumulator from the first good sample.
 *
 * @details The boot anchor comes from the IR-compensated voltage LUT, on the
 *          assumption that a vehicle being powered on is at or near rest. If
 *          the first read fails, the service starts with
 *          BATTERY_FLAG_NOT_ANCHORED set and anchors on the first successful
 *          Update() instead - it never publishes an SoC it has not measured.
 *
 * @note    Boot is the ONE anchor that cannot wait for the rest-detection
 *          window, so it is also the least certain. If the vehicle is powered
 *          up under load, the IR compensation carries the whole correction. The
 *          first genuine rest period fixes it.
 */
void BatteryService_Init(void);

/**
 * @brief  One tick: acquire, integrate, filter, re-anchor, publish.
 *
 * @param  speed_mps  vehicle speed, m/s, for range estimation.
 *
 * @note   SPEED IS INJECTED DELIBERATELY. range_m needs distance-per-energy,
 *         hence speed, which this service has no business owning. Including
 *         encoder.h here would break the layering symmetry with imu_service,
 *         and pushing range to the caller would split one coherent status
 *         across two owners. Pass 0.0f if range is not wanted - the service
 *         then publishes range_m = 0.
 *
 * @note   ON A FAILED READ the previous status is held, BATTERY_FLAG_I2C_FAIL
 *         is set, `seq` does NOT advance, and THE COULOMB INTEGRATOR IS NOT
 *         ADVANCED. That last point is the important one: a bad published
 *         sample is transient, but integrating a bad sample corrupts the charge
 *         state permanently.
 */
void BatteryService_Update(float32 speed_mps);

/**
 * @brief  Copy out the latest status as ONE coherent snapshot.
 * @param  out  destination. Must not be NULL.
 * @return E_OK on success; E_NOT_OK on a NULL pointer.
 * @note   There are deliberately no per-quantity getters. power_W is
 *         voltage x current, so separate getters would let a caller multiply
 *         two different instants and publish a product that never existed
 *         (S10-1 / V9-R1). Read the struct as a unit; use `seq` to detect a
 *         fresh update and `flags` to decide how much to believe it.
 */
Std_ReturnType BatteryService_GetStatus(BatteryStatusType *out);

/**
 * @brief  TRUE when the last Update() acquired a real sample AND the gauge has
 *         been anchored at least once.
 * @note   Deliberately conservative: an un-anchored gauge is reported unhealthy
 *         even though the bus may be fine, because its SoC is not yet a
 *         measurement of anything.
 */
boolean BatteryService_IsHealthy(void);

#endif /* BATTERY_SERVICE_H_ */
