/******************************************************************************
 *
 * Module: DistanceControl
 *
 * File Name: distance_control.c
 *
 * Description: Outer distance loop. distance-error (cm) -> PID -> velocity
 *              setpoint (RPM) -> velocity_control inner loop. Straight line.
 *
 ******************************************************************************/

#include "distance_control.h"
#include "pid.h"               /* outer PID handle (own gains)            */
#include "encoder.h"           /* Encoder_GetAverageDistanceCm (additive)  */
#include "velocity_control.h"  /* inner loop: SetSetpoint/Stop/getters     */

/* ============================================================================
 *  OUTER distance-loop tuning — distance-error (cm) -> velocity-setpoint (RPM).
 *  This is a DIFFERENT transfer than the INNER velocity loop's gains and MUST
 *  NOT be set from those.
 *
 *  D11-7: this comment used to quote the velocity gains as "Kp=1.875,
 *  Ki=10.65". Those were SUPERSEDED (RETUNE 13; corrected in velocity_control.c
 *  by FIX 26 / V9-1) and no longer exist anywhere in the build. The numbers are
 *  deliberately NOT re-quoted here — a second copy in a second file is exactly
 *  how the first one went stale. For the live inner-loop gains and their
 *  authoritative source, see the constants block in src/velocity_control.c
 *  (sourced from motor_modeling/RETUNE_16MHZ_REPORT.md).
 *
 *  DC_KI MUST be non-zero here. The shared PID engine (pid.c) always runs
 *  back-calculation anti-windup. On the first tick of any move large enough
 *  to saturate the output (Kp*error > DC_VEL_MAX_RPM), the engine feeds the
 *  saturation excess back into the integrator. With DC_KI = 0 the integral
 *  term has NO dynamics to ever bleed that excess off, so it freezes as a
 *  permanent output offset (= -clampExcess) and the loop settles at
 *  DC_VEL_MAX_RPM/DC_KP cm (~12 cm) for EVERY target >= that value.
 *  A small, real DC_KI gives the integrator proper unwind dynamics so the
 *  anti-windup correction is absorbed instead of stuck, letting the loop
 *  reach any commanded distance. This is REQUIRED, not optional tuning.
 * ==========================================================================*/
#define DC_KP              (5.0f)    /* TUNABLE: RPM per cm of error            */
#define DC_KI              (0.5f)    /* TUNABLE: REQUIRED non-zero — see note above */
#define DC_KD              (0.0f)    /* TUNABLE: start 0                        */
#define DC_TS_SECONDS      (0.05f)   /* 50 ms outer cadence (MUST match caller) */

/* Velocity-setpoint output clamp (RPM). TUNABLE — sane straight-line approach
 * speed; the inner loop was verified up to ~100 RPM. */
#define DC_VEL_MAX_RPM     (60.0f)   /* TUNABLE */

/* Minimum-move floor (RPM). TUNABLE — the smallest setpoint magnitude that
 * actually makes the car move. As the target nears, the PID output (RPM)
 * shrinks toward zero; below this floor the wheels creep/stall short of the
 * target (worse on the ground, where friction needs a real minimum speed).
 * OUTSIDE the arrival deadband the |setpoint| is floored up to this value,
 * sign preserved (direction toward the target); INSIDE the deadband the floor
 * is NOT applied, so the arrival/hold logic can bring the car to a full stop.
 * Effective travelling speed is thus clamped to [DC_MIN_MOVE_RPM,
 * DC_VEL_MAX_RPM] in magnitude. Tune on the ground. */
#define DC_MIN_MOVE_RPM    (8.0f)    /* TUNABLE */

/* ---------------------------------------------------------------------------
 *  TWO DISTINCT ARRIVAL BANDS (D11-4). They are NOT the same thing:
 *
 *  DC_ARRIVAL_DEADBAND_CM  = FLOOR-SUPPRESSION band. Inside it the
 *      DC_MIN_MOVE_RPM floor is released so the loop can decelerate below the
 *      creep threshold instead of being held at 8 RPM all the way in.
 *  DC_ARRIVAL_TOLERANCE_CM = STOP tolerance, i.e. the REST ACCURACY of the
 *      instrument. The car is declared arrived (and latched stopped) only
 *      within this, AND only when it is actually slow.
 *
 *  ORDERING IS LOAD-BEARING: TOLERANCE must be < DEADBAND, so the floor
 *  releases BEFORE the settle point and the last (DEADBAND - TOLERANCE) = 0.8 cm
 *  is driven by un-floored PID output decaying toward the target. If they were
 *  equal, the car would still be commanded at the 8 RPM floor at the instant it
 *  is declared arrived - i.e. stopping from full creep speed, which is the
 *  truncation this fix exists to remove.
 * -------------------------------------------------------------------------*/
#define DC_ARRIVAL_DEADBAND_CM  (1.0f)   /* TUNABLE: floor-suppression band  */
#define DC_ARRIVAL_TOLERANCE_CM (0.2f)   /* TUNABLE: REST ACCURACY (D11-4)   */
#define DC_ARRIVAL_SPEED_RPM    (3.0f)   /* TUNABLE: "actually slow" gate    */

/* Rejection bound for an implausible target (D11-5). Mirrors the guards its two
 * sibling services already have (velocity R8-1, steering R3-1) - this module was
 * the only one of the three with NO input validation at all. 100 m is far beyond
 * any floor test, so ordinary over-long commands still take the normal path;
 * only non-finite or absurd values are refused. Without this a fat-fingered
 * entry (the harness parser accepts up to ~15 digits) saturates the outer PID
 * and drives the car until it is stopped by hand - on the floor. */
#define DC_IMPLAUSIBLE_CM       (10000.0f)

/* ---------------------------------------------------------------------------
 *  Encapsulated state (file-static).
 *
 *  ⚠️ D11-3 SINGLE-WRITER INVARIANT: this module writes the velocity setpoint
 *  directly (VelocityControl_SetSetpoint at the arrival brake, the normal
 *  command, and Stop; plus VelocityControl_Stop). It is the SOLE writer only
 *  because the closed_loop_distance harness never runs src/main.c's super-loop
 *  - the two owners are mutually exclusive BY CONSTRUCTION, not by any check.
 *  Never run both. Full rationale (and velocity's V9-R3) in distance_control.h.
 * -------------------------------------------------------------------------*/
static PID_HandleType dc_pid;
static float32 dc_targetCm       = 0.0f;
static float32 dc_originCm       = 0.0f;   /* avg distance captured at SetTarget */
static float32 dc_velSetpointRpm = 0.0f;   /* last outer output (observability)  */

/* D11-4/D11-5 arrival state.
 *   dc_inBand  - edge flag: TRUE once inside DC_ARRIVAL_DEADBAND_CM, so the
 *                integrator is cleared exactly ONCE on entry (see Update).
 *                Clearing it every in-band tick would kill the integral action
 *                that closes the last millimetres.
 *   dc_stopped - latched once the settle test passes; the module then holds
 *                zero until re-armed by SetTarget (or Stop). An instrument
 *                should come to rest and STAY there, not chase drift. */
static boolean dc_inBand         = FALSE;
static boolean dc_stopped        = FALSE;

static uint32  dc_rejectedTargets = 0U;    /* D11-5: implausible targets refused */

/* ===========================================================================*/

static float32 DistanceControl_MeasuredCm(void)
{
    return Encoder_GetAverageDistanceCm() - dc_originCm;
}

static float32 DistanceControl_AbsAvgSpeedRpm(void)
{
    float32 spd = (VelocityControl_GetLeftMeasuredRpm()
                 + VelocityControl_GetRightMeasuredRpm()) * 0.5f;
    return (spd < 0.0f) ? -spd : spd;
}

/* THE settle test - ONE definition, used by both DistanceControl_Update() (which
 * passes its own already-computed absErr, so it does not re-measure) and
 * DistanceControl_IsAtTarget() (which measures live). Keeping a single copy is
 * deliberate: a duplicated arrival condition that drifts apart would make the
 * CSV's at_target column disagree with when the car actually stopped. */
static boolean DistanceControl_Settled(float32 absErr)
{
    return ((absErr <= DC_ARRIVAL_TOLERANCE_CM)
         && (DistanceControl_AbsAvgSpeedRpm() <= DC_ARRIVAL_SPEED_RPM)) ? TRUE : FALSE;
}

/* D11-5: TRUE only for a finite, plausible target. Same hand-rolled form as
 * VelocityControl_IsPlausible / SteeringControl_IsPlausible - this is a
 * -ffreestanding -fno-builtin build with no libm, so <math.h> classification is
 * not dependable. NaN is caught by (v != v); +/-Inf by the bounded compare. */
static boolean DistanceControl_IsPlausible(float32 v)
{
    if (v != v)
    {
        return FALSE;                       /* NaN */
    }

    if ((v > DC_IMPLAUSIBLE_CM) || (v < -DC_IMPLAUSIBLE_CM))
    {
        return FALSE;                       /* +/-Inf (and absurd finite values) */
    }

    return TRUE;
}

void DistanceControl_Init(void)
{
    PID_ConfigType cfg;

    cfg.gains.Kp   = DC_KP;
    cfg.gains.Ki   = DC_KI;
    cfg.gains.Kd   = DC_KD;
    cfg.Ts         = DC_TS_SECONDS;
    cfg.N          = PID_DERIVATIVE_FILTER_N;
    cfg.limits.min = -DC_VEL_MAX_RPM;     /* allow reverse correction */
    cfg.limits.max =  DC_VEL_MAX_RPM;

    (void)PID_Init(&dc_pid, &cfg);

    dc_targetCm       = 0.0f;
    dc_originCm       = Encoder_GetAverageDistanceCm();   /* zero here */
    dc_velSetpointRpm = 0.0f;
    dc_inBand         = FALSE;
    dc_stopped        = FALSE;
}

void DistanceControl_SetTarget(float32 distance_cm)
{
    /* D11-5: REJECT a non-finite / absurd target instead of arming a move on it.
     * This module was the only one of the three services with no input guard,
     * yet it is the one that drives the car across a floor. Policy matches the
     * siblings: refuse, hold whatever we were doing, and COUNT it so the refusal
     * is visible rather than silent. The origin is deliberately NOT re-captured
     * on the reject path - a bad command must not move the reference. */
    if (DistanceControl_IsPlausible(distance_cm) == FALSE)
    {
        if (dc_rejectedTargets < 0xFFFFFFFFUL)
        {
            dc_rejectedTargets++;
        }
        return;                          /* target/origin/arm-state UNCHANGED */
    }

    dc_originCm       = Encoder_GetAverageDistanceCm();   /* capture origin NOW */
    dc_targetCm       = distance_cm;
    dc_velSetpointRpm = 0.0f;
    (void)PID_Reset(&dc_pid);            /* fresh integrator for the new move */

    /* D11-4: re-arm the arrival state so the next move gets its own band-entry
     * integrator reset and is not held stopped by the previous one. */
    dc_inBand         = FALSE;
    dc_stopped        = FALSE;
}

uint32 DistanceControl_GetRejectedTargetCount(void)
{
    return dc_rejectedTargets;
}

boolean DistanceControl_IsAtTarget(void)
{
    float32 err    = dc_targetCm - DistanceControl_MeasuredCm();
    float32 absErr = (err < 0.0f) ? -err : err;

    /* D11-4: now keyed on the STOP TOLERANCE (0.2 cm), not the floor-suppression
     * deadband (1.0 cm). Before this fix at_target could read TRUE up to a full
     * centimetre short, which is precisely what made the instrument report
     * "arrived" at ~49.3 of a commanded 50.0. Reported LIVE (re-measured), so if
     * the car is nudged after settling this goes FALSE while the module keeps
     * holding zero - that divergence is honest, not a bug. */
    return DistanceControl_Settled(absErr);
}

void DistanceControl_Update(void)
{
    float32 measured = DistanceControl_MeasuredCm();
    float32 err      = dc_targetCm - measured;
    float32 absErr   = (err < 0.0f) ? -err : err;
    float32 vel;

    /* ---- ARRIVAL (D11-4, rewritten; the old text here was made false) ------
     * WAS: the instant |err| <= DC_ARRIVAL_DEADBAND_CM this commanded 0 and
     * returned. That is bang-bang truncation - it stops the car wherever it
     * FIRST entered the band, i.e. up to a full centimetre short, every time,
     * in the same direction. Measured as a systematic ~0.6-0.8 cm undershoot
     * (~1.2-1.6 % on a 50 cm move) while at_target still read TRUE, which is
     * exactly the reading that gets mis-attributed to velocity tuning.
     *
     * WHY IT WAS WRITTEN THAT WAY, and why simply deleting it is wrong: the
     * shared PID engine's back-calculation anti-windup leaves a large, usually
     * negative integral near the target, so the raw output is strongly
     * wrong-signed at <1 cm error. Without the hard stop that produced a
     * reverse/forward LIMIT CYCLE at the band edge (observed jitter ~1 cm
     * short). The premature stop was a WORKAROUND FOR STALE INTEGRATOR STATE.
     *
     * FIX: remove the cause instead of the symptom. Clear the integrator ONCE
     * on band entry (D11-5), which removes the wrong-signed windup dump, and
     * then let the now-clean loop actually close the error. The stop is
     * declared only by the settle test - |err| <= TOLERANCE *AND* actually slow
     * - not by band entry. Rest accuracy goes 1.0 cm -> 0.2 cm (4x tighter)
     * with a bounded settle rather than open-ended creep.
     *
     * The reset is edge-triggered: doing it every in-band tick would zero the
     * integral action that closes the last millimetres. */
    if (dc_stopped != FALSE)
    {
        /* Latched: hold zero until re-armed by SetTarget/Stop. An instrument
         * should come to rest and STAY there rather than chase drift. */
        dc_velSetpointRpm = 0.0f;
        VelocityControl_SetSetpoint(0.0f, 0.0f);
        return;
    }

    if (absErr <= DC_ARRIVAL_DEADBAND_CM)
    {
        if (dc_inBand == FALSE)
        {
            dc_inBand = TRUE;
            (void)PID_Reset(&dc_pid);   /* ONCE, on entry - kills the windup   */
        }

        if (DistanceControl_Settled(absErr) != FALSE)
        {
            dc_stopped        = TRUE;
            dc_velSetpointRpm = 0.0f;
            VelocityControl_SetSetpoint(0.0f, 0.0f);
            return;
        }
        /* else: inside the band but not settled -> fall through and let the
         * (clean) PID keep driving toward the target. */
    }
    else
    {
        /* Left the band (new target, a push, or wheel slip): re-arm the edge so
         * a later re-entry gets its own reset. */
        dc_inBand = FALSE;
    }

    /* Outer PID: distance error (cm) -> velocity setpoint (RPM), clamped to
     * +/- DC_VEL_MAX_RPM by the engine's limits. P-term shrinks as the target
     * nears, so the car decelerates smoothly toward it. */
    vel = PID_Update(&dc_pid, dc_targetCm, measured);

    /* Minimum-move floor: while still OUTSIDE the arrival deadband, never let
     * the setpoint decay below DC_MIN_MOVE_RPM (or the car creeps and stalls
     * short). The travel DIRECTION is taken from the distance ERROR sign, NOT
     * from the PID output sign: the shared engine's anti-windup dumps a large
     * negative integral on the first (saturated) tick, which can drag the PID
     * output to ~0 (or briefly the wrong sign) while still short of target. If
     * the floor keyed off the PID output sign, that near-zero dither would flip
     * the command +/- every tick -> motor jitter (worse at longer targets,
     * where the dump is bigger). Keying off the error makes the floor:
     *   short of target  (err >  deadband):  vel = max(vel, +floor)  (forward)
     *   overshot target  (err < -deadband):  vel = min(vel, -floor)  (reverse)
     * so a near-zero/wrong-sign PID output can never reverse the command, and
     * the car is never driven backward while still short of the goal. INSIDE
     * the deadband this block is skipped, so arrival/hold can stop AT target. */
    if (err > DC_ARRIVAL_DEADBAND_CM)
    {
        if (vel < DC_MIN_MOVE_RPM)  { vel = DC_MIN_MOVE_RPM; }
    }
    else if (err < -DC_ARRIVAL_DEADBAND_CM)
    {
        if (vel > -DC_MIN_MOVE_RPM) { vel = -DC_MIN_MOVE_RPM; }
    }

    dc_velSetpointRpm = vel;

    /* Straight line: same setpoint to both wheels (no heading correction in v1). */
    VelocityControl_SetSetpoint(vel, vel);
}

void DistanceControl_Stop(void)
{
    dc_velSetpointRpm = 0.0f;
    VelocityControl_SetSetpoint(0.0f, 0.0f);
    VelocityControl_Stop();          /* hard stop the inner loop */
    (void)PID_Reset(&dc_pid);        /* reset outer integrator   */

    /* D11-4: clear the arrival state so a later SetTarget starts clean. Note
     * dc_stopped is cleared (not set) here: this is an ABORT, not an arrival -
     * the caller disarms, and re-arming goes through SetTarget. */
    dc_inBand  = FALSE;
    dc_stopped = FALSE;
}

float32 DistanceControl_GetMeasuredDistanceCm(void) { return DistanceControl_MeasuredCm(); }
float32 DistanceControl_GetVelSetpointRpm(void)     { return dc_velSetpointRpm; }
float32 DistanceControl_GetTargetCm(void)           { return dc_targetCm; }  /* TEMP DIAG */
