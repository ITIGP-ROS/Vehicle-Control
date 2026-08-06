/******************************************************************************
 *
 * Module: VelocityControl
 *
 * File Name: velocity_control.c
 *
 * Description: Per-wheel closed-loop velocity control. Two independent PID
 *              loops on measured wheel RPM -> signed motor speed.
 *
 *              The loop STRUCTURE (per-wheel PID, Ts, output limits, the
 *              sign chain) was seeded from the validated harness
 *              test/closed_loop_speed_encoder/main.c. The GAIN VALUES were
 *              NOT: they were subsequently re-derived and confirmed by
 *              RETUNE 13 (2026-07-31) on the corrected true-16 MHz plant.
 *
 *              AUTHORITATIVE SOURCE FOR THE GAINS: RETUNE 13
 *              (motor_modeling/RETUNE_16MHZ_REPORT.md, summarised in
 *              docs/MEMORY.md) -- NOT the harness. The harness still carries
 *              its own older Z-N values and they are NOT what ships here;
 *              do not "restore" them. See V9-1 in
 *              docs/velocity/REVIEW_09_velocity_control.md.
 *
 ******************************************************************************/

#include "velocity_control.h"
#include "pid.h"        /* PID_Init/Update/Reset, PID_*Type, PID_DERIVATIVE_FILTER_N */
#include "encoder.h"    /* Encoder_GetRPM, ENCODER_LEFT/RIGHT                        */
#include "Motor.h"      /* Motor_SetSignedSpeed, Motor_Stop, MOTOR_LEFT/RIGHT       */

/*----------------------------------------------------------------------------
 * B9: the setpoint PAIR becomes cross-task.
 *
 * tRosRx (prio 7) writes it via SetSetpoint; tVelocity (prio 8) reads it in
 * Update. Each float32 is an aligned 32-bit access and therefore atomic on
 * Cortex-M4 - no half-values are possible. What is NOT atomic is the PAIR.
 *
 * ⚠️ THE DIRECTION OF THE HAZARD IS ASYMMETRIC, and it is worth being precise
 * because it decides where the guard is actually needed:
 *   - tRosRx CANNOT preempt tVelocity (lower priority), so once Update starts,
 *     its two setpoint reads cannot be separated by a write. The reader is safe
 *     by the ladder alone.
 *   - tVelocity CAN preempt tRosRx mid-SetSetpoint, between the left store and
 *     the right store. Update would then drive LEFT from the new command and
 *     RIGHT from the previous one for a whole 20 ms cycle - a phantom
 *     differential velocity, i.e. a small yaw disturbance the host never asked
 *     for. THE WRITER is where the guard belongs.
 *
 * Both sides are guarded anyway. The writer because it is necessary; the reader
 * because "safe by the ladder alone" is a property of today's priority table,
 * not of this module, and the port has already re-ordered that table twice
 * (BUDGET-5, then the ladder step). Cost is a handful of instructions on a
 * 20 ms path. Same reasoning as battery_service's B4 publish.
 *
 * No-ops in every non-RTOS build - one context, no pair to tear.
 *--------------------------------------------------------------------------*/
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#define VC_PAIR_ENTER()   taskENTER_CRITICAL()
#define VC_PAIR_EXIT()    taskEXIT_CRITICAL()
#else
#define VC_PAIR_ENTER()   do { } while (0)
#define VC_PAIR_EXIT()    do { } while (0)
#endif

/* ============================================================================
 *  VALIDATED control constants.
 *
 *  This is a PID controller: P + I + FILTERED D. It is NOT the PI controller
 *  the original harness used -- Kd is non-zero and the derivative path is live
 *  through the N filter below.
 *
 *  GAINS: from RETUNE 13 (2026-07-31), motor_modeling/RETUNE_16MHZ_REPORT.md.
 *  Re-derived on the corrected true-16 MHz plant (post C6-4) and independently
 *  confirmed by a Stage-4 grid search, which found no reason to move them.
 *
 *  TO RETUNE: follow RETUNE 13's method (fresh sysID on the current plant, then
 *  the grid search) and update the report. Do NOT copy values back from
 *  test/closed_loop_speed_encoder/main.c -- that harness holds older Z-N
 *  Tyreus-Luyben PI values (1.875 / 10.65 / 0.0) that were superseded here.
 * ==========================================================================*/
#define VC_KP             (0.5f)     /* RETUNE 13                             */
#define VC_KI             (8.0f)     /* RETUNE 13 (parallel form)             */
#define VC_KD             (0.01f)    /* RETUNE 13 -- non-zero: this is a PID  */
#define VC_TS_SECONDS     (0.02f)    /* MUST equal the call cadence and
                                      * QEI_VELOCITY_PERIOD_MS. Note the
                                      * caller's cadence is ">= 20 ms", so the
                                      * true period is 20-21 ms (V9-4, known
                                      * and accepted under the super-loop).    */
#define VC_OUTPUT_MIN     (-100.0f)  /* matches the Motor_SetSignedSpeed range */
#define VC_OUTPUT_MAX     ( 100.0f)  /* matches the Motor_SetSignedSpeed range */
/* N = PID_DERIVATIVE_FILTER_N (10.0f, pid_cfg.h) -- the derivative filter. */

/* ---------------------------------------------------------------------------
 *  Encapsulated state (file-static): one independent PID per wheel + setpoints.
 * -------------------------------------------------------------------------*/
static PID_HandleType vc_leftPid;
static PID_HandleType vc_rightPid;

/* V9-3: init gating. BSS-zeroed => FALSE before Init runs, so no runtime
 * initialiser is needed and the pre-init reading is correct from reset. */
static boolean vc_initialized = FALSE;

/*----------------------------------------------------------------------------
 * V9-R4 - THE INHIBIT LATCH. Added at B10 with tSafety, and the two MUST ship
 * together: tSafety cannot safely exist without this.
 *
 * THE RACE IT CLOSES. tSafety (prio 9) calls VelocityControl_Stop() - Motor_Stop
 * x2 plus PID_Reset x2. If it preempts tVelocity (prio 8) MID-Update(), then
 * when tVelocity resumes it finishes its Update by writing a freshly-computed
 * PID output to the motors - OVERWRITING THE STOP. The failsafe would appear to
 * fire (log printed, counter incremented) and the vehicle would drive on.
 * Silent, intermittent, and dependent on where in Update the preemption lands:
 * the worst possible shape for a safety defect.
 *
 * This is the exact race A4-1 rejected the WDT-interrupt option over. It came
 * back the moment the failsafe became a task that outranks the control loop.
 *
 * THE MECHANISM:
 *   Stop()        SETS   the latch  (the failsafe, and the bus-off path)
 *   Update()      OBEYS  the latch  (drives zero, never a computed output)
 *   SetSetpoint() CLEARS the latch  (a fresh accepted command = re-arm)
 *
 * ⚠️ WHY THIS BEATS A MUTEX, which was the obvious alternative:
 *   - It is SELF-HEALING. A mutex makes the stop win a race; the latch makes the
 *     race irrelevant. Under ANY interleaving - including Stop() being preempted
 *     halfway through, before it has even reached Motor_Stop - the very next
 *     Update() sees the latch and re-asserts the stopped state. Correctness
 *     stops depending on who ran first.
 *   - A mutex would let the HIGHEST-priority task in the system block on the
 *     control loop. Priority inheritance would bound it, but "the failsafe waits
 *     for anything" is the wrong shape for a failsafe.
 *   - single aligned word => atomic load/store on Cortex-M4. No critical
 *     section, no blocking, no inversion, nothing to hold across a switch.
 *
 * ⚠️ NO NEW CALL SITES ANYWHERE. A fresh accepted command was ALREADY the
 * re-arm condition in the A4-1 design, so clearing on SetSetpoint() makes the
 * existing implicit re-arm explicit and testable rather than adding a rule.
 *--------------------------------------------------------------------------*/
static volatile boolean vc_inhibit = FALSE;

/* V9-3: saturating count of calls rejected for arriving before Init. Same "a
 * guard that refuses work COUNTS it" idiom as vc_rejectedSetpoints below,
 * Can_GetIfTimeoutCount() (C6-3) and UART_GetTxDroppedCount() (FIX 23) - a
 * silent guard is only half a fix, because the ordering bug it catches stays
 * invisible. Should be 0 forever in a correctly-ordered system. */
static uint32  vc_preInitCalls = 0U;

static uint32  vc_rejectedSetpoints = 0U;  /* R8-1: non-finite setpoints refused */
static float32 vc_leftSetpointRpm  = 0.0f;
static float32 vc_rightSetpointRpm = 0.0f;

/* Most-recent iteration snapshot (read-only observability). */
static float32 vc_leftMeasRpm  = 0.0f;
static float32 vc_rightMeasRpm = 0.0f;
static float32 vc_leftOutput   = 0.0f;
static float32 vc_rightOutput  = 0.0f;

/* ===========================================================================*/

void VelocityControl_Init(void)
{
    PID_ConfigType cfg;

    /* One shared config; each wheel still gets its own handle/state. */
    cfg.gains.Kp   = VC_KP;
    cfg.gains.Ki   = VC_KI;
    cfg.gains.Kd   = VC_KD;
    cfg.Ts         = VC_TS_SECONDS;
    cfg.N          = PID_DERIVATIVE_FILTER_N;
    cfg.limits.min = VC_OUTPUT_MIN;
    cfg.limits.max = VC_OUTPUT_MAX;

    (void)PID_Init(&vc_leftPid,  &cfg);
    (void)PID_Init(&vc_rightPid, &cfg);

    vc_leftSetpointRpm  = 0.0f;
    vc_rightSetpointRpm = 0.0f;
    vc_leftMeasRpm  = 0.0f;
    vc_rightMeasRpm = 0.0f;
    vc_leftOutput   = 0.0f;
    vc_rightOutput  = 0.0f;

    /* LAST: the module is only open for business once its whole state is
     * consistent. Deliberately not set at the top - a fault-and-retry Init
     * would otherwise leave the gate open across a half-built state. */
    vc_initialized = TRUE;
}

/**
 * @brief  Reject-and-count a call that arrived before VelocityControl_Init().
 * @return TRUE if the caller must return immediately.
 *
 *  V9-3. Before this, neither Update() nor SetSetpoint() checked anything: the
 *  module was safe purely BY INHERITANCE from three lower layers (PID_Update
 *  returns 0.0f on PID_STATE_UNINIT, Encoder_GetRPM returns 0.0f uninitialised,
 *  Motor's own init guard). The outcome was right - "command zero" - but the
 *  service took no responsibility for it, so relaxing any one of those guards,
 *  or adding a member that does not delegate, would have removed the protection
 *  silently. The ordering requirement was documented thoroughly in the header
 *  and enforced by nothing.
 */
static boolean VelocityControl_RejectPreInit(void)
{
    if (vc_initialized != FALSE)
    {
        return FALSE;
    }

    if (vc_preInitCalls < 0xFFFFFFFFUL)
    {
        vc_preInitCalls++;
    }
    return TRUE;
}

/* Rejection bound for an implausible RPM setpoint (R8-1). Far outside any real
 * wheel speed, so ordinary over-fast commands still pass through and are handled
 * by the PID's own output clamp - only genuinely non-finite / absurd values are
 * refused. */
#define VC_IMPLAUSIBLE_RPM      (100000.0f)

/**
 * @brief  TRUE only for a finite, plausible RPM setpoint (R8-1 guard).
 *
 *  Same hand-rolled form as SteeringControl_IsPlausible: this is a
 *  -ffreestanding -fno-builtin build with no libm, so <math.h> classification
 *  macros are not dependable. NaN is caught by (v != v) - the only IEEE-754
 *  value not equal to itself - and +/-Inf by the bounded compare.
 */
static boolean VelocityControl_IsPlausible(float32 v)
{
    if (v != v)
    {
        return FALSE;                       /* NaN */
    }

    if ((v > VC_IMPLAUSIBLE_RPM) || (v < -VC_IMPLAUSIBLE_RPM))
    {
        return FALSE;                       /* +/-Inf (and absurd finite values) */
    }

    return TRUE;
}

void VelocityControl_SetSetpoint(float32 leftRpm, float32 rightRpm)
{
    /* V9-3: gate BEFORE the plausibility check, so a pre-init call is not
     * miscounted as a garbage-setpoint rejection - the two counters must keep
     * meaning different things. Nothing is latched either way. */
    if (VelocityControl_RejectPreInit() != FALSE)
    {
        return;
    }

    /* R8-1: REJECT a non-finite setpoint instead of handing it to the PID.
     *
     * This was the unguarded twin of the steering NaN bug fixed in R3-1 - two
     * lines apart in jetson_comm.c, where the DBC's is_in_range() returns true
     * unconditionally and nothing else checks. PROVEN ON HARDWARE: a NaN
     * setpoint latched PID_STATE_FAULT_NAN, after which PID_Update returns
     * lastOutput and IGNORES every later setpoint - the wheels kept turning at
     * 31%/35% duty with the setpoint reading 0.000000. See REVIEW 08 R8-1.
     *
     * Policy matches steering: reject the garbage and HOLD the last valid
     * setpoint (a corrupt frame is treated like a dropped one), and count it so
     * the rejection is visible rather than silent. Holding - not stopping - is
     * deliberate: a single corrupt frame should not lurch the vehicle, and
     * genuine comms loss is the bus-off path's job, which stops the drive.
     *
     * The point is that the PID now never latches, so the bus-off stop and every
     * later command keep working. */
    if ((VelocityControl_IsPlausible(leftRpm) == FALSE) ||
        (VelocityControl_IsPlausible(rightRpm) == FALSE))
    {
        if (vc_rejectedSetpoints < 0xFFFFFFFFUL)
        {
            vc_rejectedSetpoints++;
        }
        return;                             /* hold the last valid setpoint */
    }

    /* B9: commit the PAIR, so a preempting tVelocity cannot see a new left
     * against an old right. See the note at the top of this file. */
    VC_PAIR_ENTER();
    vc_leftSetpointRpm  = leftRpm;
    vc_rightSetpointRpm = rightRpm;
    /* V9-R4 RE-ARM. Reaching here means the setpoint passed the pre-init gate
     * AND the plausibility check, i.e. this is a genuinely ACCEPTED command -
     * which is exactly A4-1's re-arm condition, so the latch clears with it.
     * Inside the same commit as the pair so a preempting Update() can never see
     * "inhibit cleared" against a half-written setpoint. */
    vc_inhibit = FALSE;
    VC_PAIR_EXIT();
}

uint32 VelocityControl_GetRejectedSetpointCount(void)
{
    return vc_rejectedSetpoints;
}

boolean VelocityControl_IsInhibited(void)
{
    /* V9-R4 observability. Single aligned word => atomic read, no guard. */
    return vc_inhibit;
}

uint32 VelocityControl_GetPreInitCallCount(void)
{
    return vc_preInitCalls;
}

void VelocityControl_Update(void)
{
    /* V9-3: no PID iteration, no encoder read, and above all NO Motor command
     * before Init. The three lower-layer guards still stand behind this; the
     * difference is that the service now enforces its own contract instead of
     * borrowing theirs. */
    if (VelocityControl_RejectPreInit() != FALSE)
    {
        return;
    }

    /* Per-wheel, fully independent. Sign chain preserved EXACTLY from the
     * reference (:234-240): Encoder_GetRPM is signed, PID output is signed
     * (-100..100), Motor_SetSignedSpeed maps sign -> direction. No extra
     * sign handling is added or needed. */
    /* ⚠️ V9-R4: OBEY THE LATCH, FIRST. If the failsafe (or bus-off) has stopped
     * the drive, this must not write a computed output over that stop - which is
     * exactly what happens if tSafety preempted a running Update(). Re-assert
     * the stopped state instead; that is what makes the latch self-healing
     * rather than merely a flag.
     *
     * Motor_Stop is re-issued rather than simply returning: if Stop() was itself
     * preempted before it reached Motor_Stop, returning here would leave the
     * motors running with the latch set - stopped in name only. */
    if (vc_inhibit != FALSE)
    {
        (void)Motor_Stop(MOTOR_LEFT);
        (void)Motor_Stop(MOTOR_RIGHT);
        vc_leftOutput  = 0.0f;
        vc_rightOutput = 0.0f;
        return;
    }

    /* B9: take the setpoint pair as ONE snapshot, so both wheels are driven
     * from the same command even if the ladder is ever re-ordered such that the
     * writer can preempt this. The PID/motor work below then runs on locals. */
    {
        float32 leftSp;
        float32 rightSp;

        VC_PAIR_ENTER();
        leftSp  = vc_leftSetpointRpm;
        rightSp = vc_rightSetpointRpm;
        VC_PAIR_EXIT();

        vc_leftMeasRpm = Encoder_GetRPM(ENCODER_LEFT);
        vc_leftOutput  = PID_Update(&vc_leftPid, leftSp, vc_leftMeasRpm);
        (void)Motor_SetSignedSpeed(MOTOR_LEFT, vc_leftOutput);

        vc_rightMeasRpm = Encoder_GetRPM(ENCODER_RIGHT);
        vc_rightOutput  = PID_Update(&vc_rightPid, rightSp, vc_rightMeasRpm);
        (void)Motor_SetSignedSpeed(MOTOR_RIGHT, vc_rightOutput);
    }
}

void VelocityControl_Stop(void)
{
    /* V9-3: DELIBERATELY NOT INIT-GATED, and this asymmetry is the point.
     *
     * The guards above reject COMMANDS - things that would make the vehicle do
     * something. Stop() is the FAILSAFE: it is what the CAN bus-off path calls.
     * Refusing to stop because a module was not initialised inverts the whole
     * purpose of the gate, so the safe direction is to let it through.
     *
     * Pre-init it is harmless and still does the right thing: Motor_Stop has
     * its own init guard, and PID_Reset on a BSS-zeroed handle returns E_NOT_OK
     * without touching anything. Note it does NOT set vc_initialized - stopping
     * an uninitialised controller must not open the gate for Update(). */
    /* V9-R4: LATCH FIRST, before touching anything else. Setting it before the
     * Motor_Stop calls is deliberate - if this function is itself preempted
     * immediately after this store, the latch is already in place and the
     * resuming Update() will refuse to drive. Latch-then-act, never act-then-
     * latch. */
    vc_inhibit = TRUE;

    vc_leftSetpointRpm  = 0.0f;
    vc_rightSetpointRpm = 0.0f;
    (void)Motor_Stop(MOTOR_LEFT);
    (void)Motor_Stop(MOTOR_RIGHT);

    /* ... and reset both PID handles so a later restart does not carry a
     * charged integrator (D4: clean restart for a reusable module). */
    (void)PID_Reset(&vc_leftPid);
    (void)PID_Reset(&vc_rightPid);

    vc_leftOutput  = 0.0f;
    vc_rightOutput = 0.0f;
}

float32 VelocityControl_GetLeftMeasuredRpm(void)  { return vc_leftMeasRpm;  }
float32 VelocityControl_GetRightMeasuredRpm(void) { return vc_rightMeasRpm; }
float32 VelocityControl_GetLeftOutput(void)       { return vc_leftOutput;   }
float32 VelocityControl_GetRightOutput(void)      { return vc_rightOutput;  }

/* V9-6: VelocityControl_GetLeftSetpointRpm() was removed here (FIX 26). It was
 * a TEMPORARY accessor for the LEFT actuator RAM-buffer investigation, and that
 * investigation is closed - the fault was the PD6<->PC6 encoder-harness short,
 * repaired and verified powered in DIAG 29. It also made the API asymmetric
 * (a LEFT setpoint getter with no RIGHT one), which wrongly implied LEFT is
 * special. Do not re-add it for diagnostics; add a symmetric pair, or better,
 * the snapshot getter the port needs (V9-R1). */
