#ifndef BATTERY_SERVICE_CFG_H_
#define BATTERY_SERVICE_CFG_H_

/******************************************************************************
 *
 * Module: BatteryService
 *
 * File Name: battery_service_cfg.h
 *
 * Description: Compile-time configuration - everything a pack swap, a re-wiring
 *              or a re-calibration might legitimately change. The estimator
 *              logic lives in battery_service.c.
 *
 ******************************************************************************/

#include "Platform_Types.h"
#include "Std_Types.h"

/*******************************************************************************
 *                          The pack                                           *
 *                                                                             *
 * PRODUCTION BATTERY: 12.6 V 3S Li-ion, 20 Ah, 8 A max discharge, built-in 3S *
 * BMS. Nominal 11.1 V, empty ~9.0 V. C-rate at max = 8 A / 20 Ah = 0.40C.     *
 *******************************************************************************/

/* 20 Ah is a SPEC anchor, not a guess - which is what makes coulomb counting
 * practical here at all. Re-measure after a full discharge cycle if capacity
 * fade is suspected; this is the one number a tired pack invalidates. */
#define BATTERY_CAPACITY_MAH        (20000L)

/* Nominal pack voltage, used ONLY for the energy figure behind range_m. */
#define BATTERY_NOMINAL_MV          (11100L)

/* Pack ceiling, for reference. The shunt rails around +/-39 A but THE PACK
 * LIMIT IS 8 A - the electrical ceiling is not the safe one. Not enforced by
 * this service (it measures, it does not protect; the 3S BMS protects). */
#define BATTERY_MAX_DISCHARGE_MA    (8000L)

/*******************************************************************************
 *                          Cadence                                            *
 *******************************************************************************/

/*----------------------------------------------------------------------------
 * ⚠️ MUST MATCH THE CALL SITE. The coulomb integral is charge = SUM(I x Ts), so
 * a Ts that disagrees with the actual Update() rate scales accumulated charge
 * DIRECTLY and linearly - call at 20 Hz with this set to 10 and the gauge
 * drains at half rate, silently and forever. Same class of invariant as
 * velocity_control's Ts.
 *
 * The accumulator's unit is mA-TICKS rather than mA-seconds precisely so this
 * constant appears in exactly ONE place (the capacity conversion below) instead
 * of multiplying every integration step - see battery_service.c.
 *--------------------------------------------------------------------------*/
#define BATTERY_TICKS_PER_SECOND    (10L)       /* 10 Hz => Ts = 0.1 s */

/*******************************************************************************
 *                          IR compensation                                    *
 *******************************************************************************/

/*----------------------------------------------------------------------------
 * Series resistance between the pack EMF and the INA226's bus-sense point.
 *
 * MEASURED 176 mOhm (2026-08-05) ON THE REAL 12.6 V PACK through the PRODUCTION
 * HARNESS. See docs/ina226/BATTERY_real_pack_result.md §3.
 *
 * ⚠️ THIS SUPERSEDES THE EARLIER ~0.9 Ohm, WHICH WAS MEASURED OFF PSU LEADS and
 * was 5.1x TOO LARGE. That is not a small error: it over-compensated the IR
 * term, so the gauge CLIMBED from 75.7 % to 79.4 % WHILE THE PACK DISCHARGED.
 * The old comment below already predicted exactly this failure - "a property of
 * the SUPPLY WIRING, not the board" - and it was right.
 *
 * WHY 176 mOhm IS TRUSTED, and not merely "a different number": the correct
 * R_source is DEFINED by making SoC independent of load. Across a 3.5x current
 * range (160 -> 562 mA) the SoC spread collapses to 0.6 points at 176 mOhm but
 * fans out to 11.6 points at 900 mOhm - a 19x reduction, which no fitted
 * constant achieves by coincidence. It also lands SoC at 68.2-68.8 %, matching
 * the LUT's ~68 % for this pack's ~11.8 V resting voltage. Three independent
 * methods agree: four-point least squares 0.176, pairwise 0.181/0.188, and
 * DMM-at-the-terminals vs INA226 ~0.19.
 *
 * ⚠️ STILL A PROPERTY OF THE SUPPLY WIRING, NOT THE BOARD - leads, connectors,
 * jumpers and pack internals. RE-MEASURE IF THE HARNESS OR PACK CHANGES: two
 * points at clearly different currents, R = dV_bus/dI.
 *
 * WHY IT MATTERS: without compensation the LUT reads the sag, not the charge -
 * and with the WRONG compensation it reads a sag that is not there. The LUT's
 * mid-region 11.25-11.85 V covers 30-70 % SoC, so a 0.1 V error here is ~7 SoC
 * points. Erring HIGH is the dangerous direction: the vehicle claims more charge
 * than it has.
 *--------------------------------------------------------------------------*/
#define BATTERY_R_SOURCE_MOHM       (176.0f)

/* STD_OFF since 2026-08-05: R_source now has its own multi-point calibration on
 * the FINAL pack + harness (see above), so BATTERY_FLAG_UNCAL is no longer set
 * and SoC is reported as calibrated rather than as an uncalibrated estimate.
 * Set back to STD_ON if the harness changes and R_source becomes a guess again.
 * (Current was always calibrated independently - R_shunt was measured; this
 * flag only ever described the IR term.) */
#define BATTERY_R_SOURCE_PROVISIONAL  STD_OFF

/*******************************************************************************
 *                          Voltage filter                                     *
 *******************************************************************************/

/* EMA on the IR-compensated voltage: alpha = Ts/tau = 0.1/5 = 0.02 (tau ~ 5 s).
 * SoC physically cannot move fast, so filter hard. A median-3 runs FIRST to
 * reject single-sample spikes (a PWM commutation transient landing inside the
 * conversion window) - an EMA would smear a spike instead of rejecting it. */
#define BATTERY_VOLT_EMA_ALPHA      (0.02f)

/* Same smoothing for the power and speed that feed range_m, which is otherwise
 * far too jittery to publish at 1 km/LSB. */
#define BATTERY_RANGE_EMA_ALPHA     (0.02f)

/*******************************************************************************
 *                          Rest detection + re-anchor                         *
 *******************************************************************************/

/*----------------------------------------------------------------------------
 * The drift corrector. Coulomb counting integrates the current GAIN error
 * (+/-5-7 %) into an ever-growing SoC error - roughly 6 SoC points over a full
 * discharge. Nothing inside the integrator can detect that. What fixes it is a
 * second, independent measurement: at rest the IR term vanishes, so
 * V_oc ~ bus_mV and the voltage LUT is honest. Every rest period therefore
 * resets the integrator to a physical truth.
 *--------------------------------------------------------------------------*/

/* "At rest" = |current| below this. Above it the pack is loaded and the LUT is
 * not trustworthy. With the logic/servo idle load the vehicle is never truly at
 * open circuit while powered, so this threshold defines rest as "nothing beyond
 * a small housekeeping draw", and the IR compensation covers the remainder.
 *
 * ⚠️ OPEN FINDING BP-2 - DEFERRED, NOT SETTLED. 400 mA is TOO HIGH: on the real
 * pack (2026-08-05) the vehicle DRIVING at 6 rad/s drew 360 mA and post-stop
 * idle was 259 mA, so BOTH are classified "at rest" and the LUT re-anchor fires
 * while the pack is LOADED - exactly when the LUT is least trustworthy. That is
 * what made BP-1's error STICKY (it got blended into the coulomb accumulator)
 * rather than transient.
 *
 * WHY IT IS NOT FIXED HERE: those currents were measured WHEELS-UP / FREE-SPIN.
 * On the floor, under real rolling load, drive current will be materially higher
 * and may clear 400 mA unaided. Choosing a threshold from free-spin data risks
 * being wrong once loaded. NEEDS AN ON-FLOOR CURRENT CHARACTERISATION FIRST.
 * With BP-1 now corrected the residual harm is small - the inflated value BP-2
 * used to blend is gone - so this is a refinement, not a live gauge error.
 *
 * (Historical note: this comment previously cited "~360 mA idle draw" as the
 * justification for 400. Measured idle is 161 mA, so the stated basis for this
 * value was itself a ~2x overstatement. See BATTERY_real_pack_result.md §4.) */
#define BATTERY_REST_CURRENT_MA     (400L)

/*----------------------------------------------------------------------------
 * How long that must hold before the reading is trusted. MUST be >= ~2x the
 * voltage EMA time constant (2 x 5 s), or the re-anchor samples a filter still
 * relaxing toward the rest value and pulls SoC toward the LOADED voltage - the
 * exact error the compensation exists to avoid.
 * 100 ticks at 10 Hz = 10 s = 2 tau (~86 % settled).
 * ⚠️ PHYSICAL CAVEAT: a real cell also relaxes chemically for tens of seconds
 * to minutes after a load. 10 s is a pragmatic floor, not full relaxation, so
 * a re-anchor taken right after a hard drive still reads slightly low. The
 * blend below is what keeps that from being harmful.
 *--------------------------------------------------------------------------*/
#define BATTERY_REST_TICKS          (100L)

/* Re-anchor is a BLEND, not a snap: each rest tick moves the accumulator this
 * fraction of the way toward the LUT value. 0.01/tick at 10 Hz gives a ~10 s
 * time constant, so a sustained rest converges while a brief one barely moves
 * it. A hard snap would let one noisy voltage sample rewrite the charge state. */
#define BATTERY_REANCHOR_ALPHA      (0.01f)

/* ...except when the disagreement is too large to be drift. Past this, the
 * accumulator is not slightly off, it is WRONG (a missed charge session, a pack
 * swap, a reset mid-discharge), and blending would take minutes to admit it.
 * Snap, but only with rest confirmed. */
#define BATTERY_REANCHOR_SNAP_PCT   (25.0f)

/* Coulomb-vs-voltage disagreement that raises BATTERY_FLAG_DIVERGED. Purely
 * informational - under load this is EXPECTED, because the LUT is the
 * unreliable one there. */
#define BATTERY_DIVERGED_PCT        (15.0f)

/*******************************************************************************
 *                          Range                                              *
 *******************************************************************************/

/* Below this speed range_m is not estimable (dividing energy by ~zero
 * consumption per km explodes), so the service publishes 0. */
#define BATTERY_RANGE_MIN_SPEED_MPS (0.10f)

/* Likewise when filtered power is not positive (charging, or standing still). */
#define BATTERY_RANGE_MIN_POWER_W   (0.50f)

/* DBC range is uint8, 1 km/LSB, 0..255. */
/* Range clamp, in METRES (was 255.0f km before the 2026-08-05 cluster unit
 * change). ⚠️ THIS NUMBER IS TIED TO THE WIRE FORMAT: the DBC `range` signal on
 * 0x210 is uint16 at 1 m/LSB, so it saturates at 65535 m. A larger estimate
 * would WRAP on encode - 90 km would arrive as 24464 m, a plausible-looking but
 * completely wrong number - which is far worse than clamping. 65000 leaves a
 * little margin below the ceiling. If the DBC scale ever changes, change this
 * with it. */
#define BATTERY_RANGE_MAX_M         (65000.0f)

/*******************************************************************************
 *                          Voltage -> SoC LUT (3S Li-ion, 9.0-12.6 V)         *
 *                                                                             *
 * ASCENDING in voltage - the interpolation in battery_service.c depends on it. *
 * Dense in the knees, sparse in the middle, which is the honest shape of the   *
 * chemistry AND its limitation: 11.25-11.85 V covers 30-70 % SoC, i.e. 0.6 V   *
 * for 40 % of the pack, so a 0.1 V error there is ~7 % SoC. That is exactly    *
 * why this LUT is the ANCHOR and not the primary estimator.                    *
 *******************************************************************************/

#define BATTERY_SOC_LUT_POINTS      (12U)

#define BATTERY_SOC_LUT_MV \
    { 9000L, 10200L, 10800L, 11100L, 11250L, 11370L, 11460L, 11610L, 11850L, 12000L, 12300L, 12600L }

#define BATTERY_SOC_LUT_PCT \
    { 0.0f,   5.0f,  10.0f,  20.0f,  30.0f,  40.0f,  50.0f,  60.0f,  70.0f,  80.0f,  90.0f, 100.0f }

#endif /* BATTERY_SERVICE_CFG_H_ */
