/******************************************************************************
 *
 * Module: SteeringControl
 *
 * File Name: steering_control.c
 *
 * Description: Steering policy layer: command angle -> servo, read measured
 *              angle <- pot, build status bitfield. No PID, no loop. Wraps the
 *              proven servo + servo_feedback HALs; calibration is USED via the
 *              HAL, never re-derived here.
 *
 ******************************************************************************/

#include "steering_control.h"
#include "servo.h"           /* Servo_Init/SetAngleRad/Center                   */
#include "servo_feedback.h"  /* ServoFb_Init/ReadAngleRad/IsPotFaulted          */

/* ============================================================================
 *  Validated travel limits, REP-103 (+left / -right). RE-CALIBRATED 2026-07-30
 *  after the steering linkage rework. These MUST stay equal in magnitude to
 *  SERVO_MAX_ANGLE_LEFT/RIGHT_RAD in servo_cfg.h - they are a DUPLICATE of the
 *  HAL's command half-ranges, and de-syncing them de-calibrates the steering
 *  (the app would clamp somewhere the HAL map does not end).
 *
 *  Landmarks behind them (measured, approached from center; raw data in
 *  docs/servo/HWVERIFY_01_pot_reference.md):
 *      max LEFT  =  600us -> POT  557   limit +0.2421 rad (13.9 deg, ATTAINABLE)
 *      CENTER    = 1450us -> POT 1991   0.000 rad (wheels straight)
 *      max RIGHT = 2500us -> POT 3790   limit -0.3037 rad (17.4 deg, CAD design)
 *
 *  WHY THE TWO SIDES DIFFER (R2-1, resolved 2026-07-30 via Option 1): the CAD
 *  design angle is 17.4 deg on BOTH sides, and the right endpoint achieves it.
 *  The left does not - the servo runs out of authority at 600us, where the pot
 *  measures only 13.9 deg. The limit here therefore carries the ATTAINABLE left
 *  angle, not the design one, so that commanded and measured AGREE at the
 *  endpoint. That is what makes the measurement-derived status bits work:
 *      AT_TARGET : err = |cmd 0.2421 - measured 0.2421| = 0.0000 <= 0.02   OK
 *      SATURATED : measured 0.2421 >= (0.2421 - 0.02) = 0.2221             OK
 *  With the old 0.3037 left limit both were unreachable (err was 0.0616, 3x the
 *  deadband). Margin is comfortable: even stacking the ~40-count backlash
 *  (0.0068 rad) and the ~9.5-count endpoint noise (0.0016 rad) keeps err at
 *  ~0.0084 rad, well inside the 0.02 deadband.
 * ==========================================================================*/
/* RE-DERIVED 2026-08-16 with the steering re-calibration (servo_cfg.h).
 *
 * These are now in TRUE wheel-angle radians, like SERVO_MAX_ANGLE_*_RAD. They are
 * deliberately SMALLER than those constants, and that is not an inconsistency:
 *
 *   SERVO_MAX_ANGLE_*  = the SCALE of the angle->pulse map (set from the linear region)
 *   SC_LIMIT_*         = the largest angle we will actually COMMAND
 *
 * The servo saturates before the linkage does, so the angles the map names at the
 * pulse endpoints (0.5595 / 0.5800) are not reliably reachable. Measured: commanding
 * old-0.21991 and old-0.2421 both produced the same 0.95 m circle, i.e. no further
 * travel, and endpoint travel also drifts with battery voltage. Inside the linear
 * region the response is repeatable to better than 0.07 deg.
 *
 * 0.286 rad is the largest angle VERIFIED linear on BOTH sides (right side measured
 * to 0.28605, left to 0.34731). Chosen symmetric so the vehicle behaves the same in
 * both directions and planners get one number:
 *
 *     minimum turning radius = L / tan(0.286) = 0.23529 / 0.29388 = 0.80 m
 *
 * That is still tighter than the 0.96-1.00 m the Nav2 configs currently assume, so
 * nothing is lost operationally by clamping here.
 *
 * TO EXTEND: measure circles between cmd 0.15 and 0.22 (old units) to find where
 * saturation actually begins; the left side is known-linear to at least 0.347. Do
 * not raise these without that measurement. */
#define SC_LIMIT_LEFT_RAD     ( 0.286f)
#define SC_LIMIT_RIGHT_RAD    (-0.286f)

/* S10-4 (REVIEW 10) - UNGUARDED BY CHOICE, not by impossibility. See below.
 *
 * SC_LIMIT_* is the app-side clamp; SERVO_MAX_ANGLE_*_RAD (servo_cfg.h, already
 * visible here via servo.h) is the HAL-side map scale. They are duplicated BY
 * DESIGN - different meanings, different layers - and MUST agree in magnitude,
 * or the app clamps somewhere the servo map does not end and the endpoints
 * de-calibrate SILENTLY. Sign note: this module's axis is REP-103 (+left /
 * -right), so the RIGHT limit is the NEGATIVE of the HAL's magnitude.
 *
 * A compile-time assert in the FIX 24/25/26 negative-array-size idiom was
 * ATTEMPTED here. MEASURED behaviour on the toolchain this project actually
 * builds with (arm-none-eabi-gcc 4.8.4, gnu90 - NOT the 12.3.1 also installed):
 *
 *   values AGREE    -> 1 warning "variably modified 'X' at file scope", exit 0
 *   values DISAGREE -> error "size of array 'X' is negative", exit 1
 *
 * So a float assert DOES genuinely guard: it catches a de-sync and FAILS THE
 * BUILD. What it costs is ONE UNSUPPRESSIBLE WARNING PER ASSERT, emitted even
 * when the values agree - because a float relational expression is not an
 * integer constant expression in C90 (a floating constant qualifies only as the
 * IMMEDIATE operand of an integer cast, so scaling like (sint32)(X * 1e7f) does
 * not rescue it: that operand is a floating EXPRESSION, not a constant). The
 * warning carries no -W name ("[enabled by default]"), so
 * `#pragma GCC diagnostic ignored` cannot silence it - verified.
 *
 * That is also why timer_pwm_cfg.h can #error on the analogous PULSE endpoints
 * at no such cost: those are integers.
 *
 * THREE OPTIONS FOR THE DESK (none taken here - all are decisions, not edits):
 *   A) Collapse the duplication: define SC_LIMIT_LEFT_RAD as
 *      ( SERVO_MAX_ANGLE_LEFT_RAD) and SC_LIMIT_RIGHT_RAD as
 *      (-SERVO_MAX_ANGLE_RIGHT_RAD). De-sync becomes IMPOSSIBLE rather than
 *      merely detected, values and emitted code are unchanged, and it compiles
 *      CLEAN (verified). Cost: the app-layer clamp then references a HAL config
 *      header - though servo_cfg.h is already in this TU, so the dependency is
 *      real either way. RECOMMENDED.
 *   B) Add the float asserts and accept 2 permanent warnings in this file. They
 *      DO work; they just cost the clean sheet, permanently and unsuppressibly.
 *   C) Keep both and add parallel scaled-INTEGER companions to assert against.
 *      Warning-free, but adds a third and fourth copy of the same number.
 * Until one is chosen, THE PROSE ABOVE IS THE ONLY GUARD. If you edit either
 * constant, edit BOTH, and re-read the recalibration record in
 * docs/servo/REVIEW_02_servo_feedback.md. */

/* AT_TARGET deadband (rad). Above the ~0.008 rad pot backlash + noise
 * (servo_feedback_cfg.h). (D-DEADBAND) */
#define SC_AT_TARGET_DEADBAND_RAD   (0.02f)

/* SATURATED epsilon: measured angle within this of a travel endpoint counts as
 * "at the mechanical limit" (no HAL saturation flag exists -- D-SATURATED). */
#define SC_SATURATED_EPS_RAD        (0.02f)

/* Rejection bound for implausible setpoints (R3-1). Deliberately FAR outside any
 * real steering command (~0.3 rad max) so that ordinary out-of-travel values like
 * +/-10 rad still take the normal clamp path and still raise OUT_OF_RANGE exactly
 * as before - only genuinely non-finite / absurd inputs are rejected. */
#define SC_IMPLAUSIBLE_RAD          (1000.0f)

/* ---------------------------------------------------------------------------
 *  Encapsulated state (file-static).
 *
 *  CONCURRENCY INVARIANT: these are plain (non-volatile) statics, which is safe
 *  ONLY because every caller runs in main context - SetAngle via JetsonComm_Poll
 *  and GetStatus via JetsonComm_SendSteeringFeedback, both from the main
 *  super-loop (main.c:164 / :243). The CAN ISR only fills a ring buffer. Moving
 *  either entry point into the ISR would make these two fields a genuine race
 *  (they must agree with each other, and a float32 update is not atomic).
 * -------------------------------------------------------------------------*/
static float32 sc_lastCommandRad = 0.0f;   /* REP-103, AFTER clamp (for AT_TARGET) */
static boolean sc_lastWasClamped = FALSE;  /* request exceeded a limit -> OUT_OF_RANGE */

/* S10-1: published-observation counter. Advances ONCE per successful snapshot,
 * never on a refused one. Wraps at 255 by design - it is a change detector, not
 * a counter (see steering_control_types.h). Write-only under the super-loop;
 * it exists so the port does not have to retrofit staleness detection. */
static uint8   sc_snapshotSeq    = 0U;

/* S10-3: init gating. BSS-zeroed => FALSE before Init, so the pre-init reading
 * is correct from reset without a runtime initialiser. */
static boolean sc_initialized = FALSE;

/* S10-3: saturating count of entry-point calls refused for arriving before
 * Init, plus actuation attempts the HAL refused. Same "a guard that refuses
 * work COUNTS it" idiom as velocity's vc_preInitCalls. 0 in a healthy system. */
static uint32  sc_preInitCalls = 0U;

/**
 * @brief  Reject-and-count a call that arrived before SteeringControl_Init().
 * @return TRUE if the caller must return immediately.
 */
static boolean SteeringControl_RejectPreInit(void)
{
    if (sc_initialized != FALSE)
    {
        return FALSE;
    }

    if (sc_preInitCalls < 0xFFFFFFFFUL)
    {
        sc_preInitCalls++;
    }
    return TRUE;
}

/**
 * @brief  TRUE only for a finite, plausible setpoint (R3-1 guard).
 *
 *  WHY hand-rolled instead of isfinite()/isnan(): this is a -ffreestanding
 *  -fno-builtin build with no libm, so <math.h> classification macros are not
 *  dependable here.
 *    - NaN  is caught by (v != v): NaN is the only IEEE-754 value not equal to
 *      itself. This is what the original clamp missed - EVERY comparison with
 *      NaN is false, so NaN fell through the >/< tests into the "in range"
 *      branch and reached the servo.
 *    - +/-Inf is caught by the bounded compare (Inf exceeds any finite bound).
 *  Deliberately NOT written as ((v - v) == 0.0f): that idiom is correct but a
 *  compiler permitted to assume finite math may fold it away.
 */
static boolean SteeringControl_IsPlausible(float32 v)
{
    if (v != v)
    {
        return FALSE;                       /* NaN */
    }

    if ((v > SC_IMPLAUSIBLE_RAD) || (v < -SC_IMPLAUSIBLE_RAD))
    {
        return FALSE;                       /* +/-Inf (and absurd finite values) */
    }

    return TRUE;
}

/* ===========================================================================*/

void SteeringControl_Init(void)
{
    Servo_Init();     /* TimerPWM_Init + Servo_Center (self-centered at boot) */
    ServoFb_Init();   /* ADC MCAL bring-up                                    */

    sc_lastCommandRad = 0.0f;
    sc_lastWasClamped = FALSE;

    /* LAST: open the gate only once the whole module state is consistent. */
    sc_initialized = TRUE;
}

void SteeringControl_SetAngle(float32 angleRad)
{
    float32 cmd     = angleRad;          /* REP-103 (+left / -right) */
    boolean clamped = FALSE;

    /* S10-3: THE GATE MUST COME FIRST - before sc_lastCommandRad or
     * sc_lastWasClamped is touched, and before the R3-1 check below, which
     * writes sc_lastWasClamped on the reject path.
     *
     * The pre-fix ordering was the actual defect, and it was worse than "no
     * guard". SetAngle recorded the command at :157-158 and only then called
     * Servo_SetAngleRad, whose MCAL correctly refused with NOT_INITIALIZED and
     * wrote no hardware. Net effect: the module recorded a command the servo
     * NEVER RECEIVED, and a later GetStatus computed AT_TARGET against that
     * phantom angle - it could report the steering as on-target for an angle
     * that was never actuated. Rejecting up here means a pre-Init call records
     * NOTHING, which is the only honest outcome. */
    if (SteeringControl_RejectPreInit() != FALSE)
    {
        return;
    }

    /* R3-1: REJECT a non-finite / implausible setpoint. This module is the LAST
     * line of defence - the DBC's ..._is_in_range() returns true unconditionally
     * and jetson_comm passes the unpacked float straight through, so a NaN out of
     * a ROS controller (uninitialised float, degenerate division) arrives here
     * intact. Left unguarded it used to survive the clamp below, survive the HAL
     * clamp, and end as (uint16)(NaN+0.5f) -> a 600us FULL-LEFT LOCK.
     *
     * Policy: HOLD THE LAST VALID COMMAND. We do NOT re-center - snapping the
     * wheels straight mid-corner is its own hazard - and we do not hand garbage
     * to the servo. sc_lastCommandRad is left untouched so AT_TARGET keeps
     * evaluating against the angle we are actually still holding.
     *
     * It must not be SILENT (that was half the original bug): OUT_OF_RANGE is
     * raised, on the reading that an implausible command is out of any valid
     * range. If the fleet ever needs to distinguish "clamped" from "rejected",
     * that wants its own bit in the DBC contract - flagged, not invented here. */
    if (SteeringControl_IsPlausible(cmd) == FALSE)
    {
        sc_lastWasClamped = TRUE;   /* visible as OUT_OF_RANGE */
        return;                     /* servo and sc_lastCommandRad UNCHANGED */
    }

    /* Clamp to validated asymmetric travel (the HAL clamps only the pulse). */
    if (cmd > SC_LIMIT_LEFT_RAD)
    {
        cmd = SC_LIMIT_LEFT_RAD;
        clamped = TRUE;
    }
    else if (cmd < SC_LIMIT_RIGHT_RAD)
    {
        cmd = SC_LIMIT_RIGHT_RAD;
        clamped = TRUE;
    }
    else
    {
        /* in range */
    }

    /* SIGN RECONCILIATION (D-SIGN; see header): the Servo command HAL is
     * REP-103-INVERTED (+angle drives RIGHT in servo.c), while this module's
     * boundary and the pot feedback are REP-103 (+left). Negate once here so a
     * +left command physically steers left and matches the +left/-right
     * measured angle (otherwise AT_TARGET could never assert).
     *
     * S10-3: ACTUATE FIRST, RECORD SECOND, and only on success. The recorded
     * command is what AT_TARGET is computed against, so it must describe an
     * angle the servo actually received - "what we told the hardware", never
     * "what we intended to tell it". The HAL's verdict used to be discarded at
     * two layers (Servo_SetAngleRad was void and itself did
     * `(void)TimerPWM_SetPulseUs`); both now propagate.
     *
     * Behaviourally identical for a correctly-ordered caller: post-Init the
     * MCAL cannot refuse a pulse, so this branch is never taken and the same
     * two stores happen in the same order as before, one call later. */
    if (Servo_SetAngleRad(-cmd) != E_OK)
    {
        if (sc_preInitCalls < 0xFFFFFFFFUL)
        {
            sc_preInitCalls++;   /* actuation refused - visible, not silent */
        }
        return;                  /* record NOTHING: the servo never moved */
    }

    sc_lastCommandRad = cmd;
    sc_lastWasClamped = clamped;
}

/* ===========================================================================
 *  S10-1: THE ONE ACQUISITION PATH.
 *
 *  This function is the only place in the module that reads the pot. Everything
 *  published about the steering comes out of it, in one struct, from one sample.
 *
 *  WHAT CHANGED AND WHY (REVIEW 10 S10-1, decided REVIEW A4 §1):
 *  R3-2 had already made the STATUS byte internally coherent - one
 *  ReadRawFiltered, and `measured`/`potFault`/AT_TARGET/SATURATED all derived
 *  from it. But it then threw `measured` away and returned only the byte, so
 *  the caller had to call SteeringControl_GetMeasuredAngle() to get an angle -
 *  a SECOND, independent 8-sample acquisition. The tear R3-2 closed inside the
 *  module reopened one level up, at the frame: jetson_comm shipped sample B's
 *  angle beside sample A's bits, 16 conversions per 0x130 frame to produce an
 *  incoherent result. HV-1 has the capture (docs/steering/HV1_*).
 *
 *  The fix is to stop discarding a value that was already computed: return
 *  `measured` alongside the bits. Nothing about the bits' derivation changed.
 * ==========================================================================*/
Std_ReturnType SteeringControl_GetSnapshot(SteeringSnapshotType *out)
{
    uint8   status;
    uint16  raw;
    float32 measured;
    boolean potFault;
    float32 err;

    if (out == NULL_PTR)
    {
        return E_NOT_OK;   /* nowhere to publish; touch nothing */
    }

    /* S10-3: reject before touching the ADC. This is the one entry point that
     * already degraded gracefully - Adc_ReadRaw is init-guarded (FIX 01) and
     * returns ADC_READ_INVALID, which ServoFb_IsCountFaulted turns into
     * POT_FAULT with the measurement bits suppressed - so returning POT_FAULT
     * here is the SAME byte the ungated path produced, just without eight
     * pointless blocking conversions. Behaviour-neutral by construction, and it
     * keeps the module's answer to "what do you know?" honest: nothing.
     *
     * The snapshot is still FULLY WRITTEN on this path - a caller that ignores
     * the return value must not read uninitialised stack. Angle is 0.0 (before
     * Init there is no last-known measurement to hold) and POT_FAULT already
     * declares the value carries no information. `seq` is NOT advanced: no
     * acquisition happened, and the whole point of the field is that it only
     * moves when a real observation was made. */
    if (SteeringControl_RejectPreInit() != FALSE)
    {
        out->angle_rad = 0.0f;
        out->status    = STEERING_STATUS_POT_FAULT;
        out->seq       = sc_snapshotSeq;
        return E_NOT_OK;
    }

    /* R3-2: ONE acquisition, EVERY verdict derived from it. Previously this read
     * the pot TWICE (ReadAngleRad, then IsPotFaulted) and treated two independent
     * samples of a moving quantity as a single observation - so POT_FAULT could
     * be decided from a sample that `measured` never saw, gating AT_TARGET and
     * SATURATED on evidence other than the value they were computed from.
     *
     * S10-2: sc_lastCommandRad and sc_lastWasClamped are each read EXACTLY ONCE,
     * here, in one pass. That is what stops AT_TARGET and OUT_OF_RANGE from
     * describing two different commands under the port. */
    status   = 0U;
    raw      = ServoFb_ReadRawFiltered();
    measured = ServoFb_CountToAngleRad(raw);
    potFault = (ServoFb_IsCountFaulted(raw) != FALSE);
    err      = sc_lastCommandRad - measured;

    if (err < 0.0f)
    {
        err = -err;
    }

    /* bit1 POT_FAULT: independent plausibility flag from the feedback HAL. */
    if (potFault != FALSE)
    {
        status |= STEERING_STATUS_POT_FAULT;
    }

    /* bit2 OUT_OF_RANGE: the last requested setpoint exceeded travel (clamped).
     * Command-side -- valid even when the pot is faulted. */
    if (sc_lastWasClamped != FALSE)
    {
        status |= STEERING_STATUS_OUT_OF_RANGE;
    }

    /* Measurement-derived bits (AT_TARGET, SATURATED) are only meaningful when
     * the pot is trustworthy. Suppress them while POT_FAULT is set (D-POTFAULT). */
    if (potFault == FALSE)
    {
        /* bit0 AT_TARGET: commanded vs measured within the deadband. */
        if (err <= SC_AT_TARGET_DEADBAND_RAD)
        {
            status |= STEERING_STATUS_AT_TARGET;
        }

        /* bit3 SATURATED: measured angle sits at/near a mechanical travel endpoint. */
        if ((measured >= (SC_LIMIT_LEFT_RAD  - SC_SATURATED_EPS_RAD)) ||
            (measured <= (SC_LIMIT_RIGHT_RAD + SC_SATURATED_EPS_RAD)))
        {
            status |= STEERING_STATUS_SATURATED;
        }
    }

    /* PUBLISH: the angle is the sample the bits above were formed on, so the
     * two cannot contradict each other. It is published even under POT_FAULT -
     * untrustworthy, but it IS the evidence for the fault verdict, which is what
     * makes a dropout diagnosable from the frame that carries it (HV-1 §3). */
    sc_snapshotSeq++;              /* wraps at 255 - a change detector */
    out->angle_rad = measured;
    out->status    = status;
    out->seq       = sc_snapshotSeq;

    return E_OK;
}

/* DEPRECATED - see steering_control.h. A thin wrapper over the ONE acquisition
 * path, kept for the diagnostic callers that want only the byte. It is NOT a
 * second acquisition path, and its angle twin (GetMeasuredAngle) is GONE, so
 * the S10-1 pairing is not writable any more.
 *
 * Behaviour-identical to the pre-fix function on both paths: pre-Init it still
 * returns exactly STEERING_STATUS_POT_FAULT, still counts the refusal once
 * (RejectPreInit runs once, inside GetSnapshot), and still touches no ADC. */
uint8 SteeringControl_GetStatus(void)
{
    SteeringSnapshotType snap;

    (void)SteeringControl_GetSnapshot(&snap);   /* fills snap on BOTH paths */

    return snap.status;
}

/* S10-5: EXPLICIT-COMMAND re-centre - NOT the bus-off/failsafe action. Policy is
 * "stop driving, HOLD the wheel - do NOT re-centre" (accepted 2026-07-31, same
 * reasoning as R3-1's hold-on-NaN); main.c's bus-off path calls
 * VelocityControl_Stop() and deliberately not this. Has ZERO callers today, and
 * that is the policy working, not an oversight - see steering_control.h before
 * wiring it anywhere. Also: "centred" is a one-shot action, not a state. */
void SteeringControl_Center(void)
{
    /* S10-3: gated like SetAngle, and for the same reason - it writes the same
     * two state fields. Note this is NOT the failsafe (velocity's Stop() is,
     * and that one is deliberately left ungated): S10-5 records that the
     * bus-off policy is "hold the wheel, do NOT re-centre", so Center is an
     * explicit command and belongs behind the gate with the other commands. */
    if (SteeringControl_RejectPreInit() != FALSE)
    {
        return;
    }

    if (Servo_Center() != E_OK)
    {
        if (sc_preInitCalls < 0xFFFFFFFFUL)
        {
            sc_preInitCalls++;
        }
        return;                  /* record NOTHING: the servo never centred */
    }

    sc_lastCommandRad = 0.0f;
    sc_lastWasClamped = FALSE;
}

uint32 SteeringControl_GetPreInitCallCount(void)
{
    return sc_preInitCalls;
}
