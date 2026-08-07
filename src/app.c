/******************************************************************************
 * app.c - Tier 4: APPLICATION / ORCHESTRATION. See app.h for the contract.
 *
 * B11, 2026-08-06: THE PORT COMPLETES HERE. Before this commit main.c held the
 * init block, every task function, every static task buffer, the FreeRTOS hooks
 * AND a residual super-loop - roughly 2,000 lines doing four different jobs.
 * B11 splits that: hardware bring-up stays in main.c, and everything about the
 * TASK SET lives here.
 *
 * ⚠️ WHAT MOVED IS ONLY *WHERE*, NOT *WHAT*. Every task body below is byte-for-
 * byte the code that was proven at B4-B10; the only genuinely new code in this
 * file is tBusHealth and tHeartbeat (Part 1 of B11), which are themselves the
 * last two residents of the retired super-loop, moved verbatim.
 *
 * ⚠️ THE STATIC BUFFERS LIVE NEXT TO THEIR TASKS, not gathered in one block at
 * the top. Tier 4 "owns the task set", and both layouts satisfy that - but a
 * task's stack size is a property OF that task, and the measured high-water
 * mark that justifies the number is in the comment beside it. Separating them
 * would put the number and its justification on different screens, which is how
 * stack sizes drift.
 *
 * ⚠️ NO CONTROL MATH, NO PROTOCOL PACKING, NO REGISTER ACCESS. This file is
 * wiring. If logic starts accumulating here it belongs in a service.
 *****************************************************************************/

#include "app.h"
#include "app_cfg.h"

#include "uart.h"
#include "can.h"
#include "can_tx_queue.h"
#include "velocity_control.h"
#include "steering_control.h"
#include "jetson_comm.h"
#include "cluster_comm.h"
#include "node_ping.h"
#include "odo.h"
#include "encoder.h"
#include "battery_service.h"
#include "imu_service.h"
#include "timer0.h"

/*----------------------------------------------------------------------------
 * ⚠️ THE KERNEL IS COMPILED IN BOTH BUILD MODES.
 *
 * `build_src_filter` cannot be made conditional on a build flag, so the
 * vendored FreeRTOS sources are compiled even when USE_FREERTOS is off - the
 * flag gates main.c and the startup vector table, not the file list. That is
 * harmless (the scheduler is simply never started), but it has one consequence
 * that MUST be honoured: FreeRTOSConfig.h defines configASSERT unconditionally,
 * so tasks.c/queue.c reference vAssertCalled in EVERY build.
 *
 * The three FreeRTOS callbacks below therefore live OUTSIDE the USE_FREERTOS
 * guard. Everything else in this file - the tasks, App_Start - is inside it.
 *
 * ⚠️ This is exactly what broke the non-RTOS rollback link, and it is worth
 * remembering: `pio run` building 20/20 does NOT prove the rollback works,
 * because only the production env compiles main.c and only it defines the flag.
 * The rollback has to be built explicitly.
 *--------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/*----------------------------------------------------------------------------
 * configASSERT's landing site (FreeRTOSConfig.h section 9).
 *
 * A failure here is a CONFIGURATION error - an illegal ISR priority, a FromISR
 * call from an interrupt above configMAX_SYSCALL_INTERRUPT_PRIORITY, a
 * mis-routed vector - never a data error, so limping on is never right.
 *
 * ⚠️ MOTORS FIRST, LOG SECOND. Unlike the B1 harness, THIS image can be driving
 * when an assert fires. Stopping the wheels matters more than the message, and
 * VelocityControl_Stop() is deliberately un-gated (V9-3) so it works even if a
 * module never initialised.
 *--------------------------------------------------------------------------*/
void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
    VelocityControl_Stop();
    taskDISABLE_INTERRUPTS();

    (void)UART_SendString(DIAG_UART, "\r\n# ASSERT ");
    (void)UART_SendString(DIAG_UART, pcFile);
    (void)UART_SendString(DIAG_UART, ":");
    (void)UART_SendInteger(DIAG_UART, (sint32)ulLine);
    (void)UART_SendString(DIAG_UART, " - MOTORS STOPPED, HALTED\r\n");

    for (;;) { }
}

/* configCHECK_FOR_STACK_OVERFLOW 2. Same policy: stop the wheels, then say so.
 * ⚠️ pcTaskName may itself be corrupt if the overflow scribbled the TCB - trust
 * the fact of the trap more than the name it prints. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;

    VelocityControl_Stop();
    taskDISABLE_INTERRUPTS();

    (void)UART_SendString(DIAG_UART, "\r\n# STACK OVERFLOW in task '");
    (void)UART_SendString(DIAG_UART, (pcTaskName != NULL_PTR) ? pcTaskName : "?");
    (void)UART_SendString(DIAG_UART, "' - MOTORS STOPPED, HALTED\r\n");

    for (;;) { }
}

/* Idle task memory. Required because configSUPPORT_DYNAMIC_ALLOCATION is 0 and
 * configKERNEL_PROVIDED_STATIC_MEMORY is 0 - see the FINDING in
 * FreeRTOSConfig.h section 2 for why the kernel does NOT provide it here (its
 * version drags in a dead ~512 B timer-task stack on this no-gc-sections
 * build). Only the IDLE callback is needed; the timer callback's sole caller,
 * timers.c, is compiled away entirely by configUSE_TIMERS 0. */
static StaticTask_t sl_idleTcb;
static StackType_t  sl_idleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE *puxIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &sl_idleTcb;
    *ppxIdleTaskStackBuffer = sl_idleStack;
    *puxIdleTaskStackSize   = (configSTACK_DEPTH_TYPE)configMINIMAL_STACK_SIZE;
}

#ifdef USE_FREERTOS

/*----------------------------------------------------------------------------
 * AppTask_PhaseAlign - park the caller until the tick counter reaches its
 * assigned residue, and return the xTaskDelayUntil anchor.
 *
 * Locking to an ABSOLUTE residue (rather than "period ms from boot") is what
 * makes the phase plan above hold forever instead of depending on the order
 * tasks happened to start in. Introduced at BUDGET-6 as two copy-pasted blocks
 * in tBattery and tOdo; factored out here at B7 because two more tasks need it
 * and a phase plan enforced by duplicated code is a phase plan waiting to drift.
 *--------------------------------------------------------------------------*/
static TickType_t AppTask_PhaseAlign(uint32 periodMs, uint32 phaseMs)
{
    TickType_t nowTick = xTaskGetTickCount();
    TickType_t target  = (nowTick - (nowTick % periodMs)) + phaseMs;

    if (target <= nowTick) { target += periodMs; }
    vTaskDelay(target - nowTick);

    return target;
}



/*----------------------------------------------------------------------------
 * B4: tBattery. Static allocation, lowest priority, 100 ms.
 *
 * Observability, published for the heartbeat to print (both plain uint32 -
 * single-word, so tear-free on Cortex-M4 without any guard):
 *   bt_hwmWords - this task's stack high-water mark, the budget's [E] -> [M].
 *   bt_maxUpdateUs - the WORST Update() seen since boot. This is the first real
 *                    per-task WCET number in the project.
 *--------------------------------------------------------------------------*/
static StaticTask_t bt_tcb;
static StackType_t  bt_stack[BATTERY_TASK_STACK_WORDS];

static volatile uint32 bt_hwmWords    = 0U;
static volatile uint32 bt_maxUpdateUs = 0U;

static void BatteryTask(void *pvParameters)
{
    TickType_t lastWake;

    (void)pvParameters;

    /* ⚠️ PHASE-LOCK TO THE OLD SLOT RESIDUE (43 mod 100). RESTORED AT B6 - and
     * dropping it at B4 was a real bug that only became VISIBLE here.
     *
     * The old super-loop ran this at now%100==43, and 43 was not arbitrary:
     * 43 mod 10 = 3, so it could never coincide with the 0x110 (0 mod 10) or
     * 0x130 (5 mod 10) transmit milliseconds. B4 turned the slot into a task
     * with a plain 100 ms period seeded at boot - which lands on ticks
     * congruent to 0 mod 100, i.e. 0 MOD 10: precisely the 0x110 slot.
     *
     * It was harmless while tBattery sat at the lowest priority. The moment B6
     * raised it to 4 (above tSuperLoop's 2), its wake began preempting
     * tSuperLoop in exactly the millisecond tSuperLoop needed to post 0x110 -
     * so that iteration missed the tick, skipped the slot, and the frame was
     * never posted. Measured on the wire: 0x110 fell to 90.06 Hz, a clean 10 %
     * loss - one collision per 100 ms against a 10 ms frame period, exactly as
     * the arithmetic predicts. 0x130 (residue 5) was untouched.
     *
     * Locking to an absolute residue rather than "100 ms from boot" reproduces
     * the original design intent exactly, and keeps doing so across every
     * future priority change. */
    lastWake = AppTask_PhaseAlign(BATTERY_TASK_PERIOD_MS, BATTERY_TASK_PHASE_MS);

    for (;;)
    {
        /* ---- speed injection: IDENTICAL to the old slot 43, deliberately ----
         * battery_service never includes encoder.h (the DESIGN §4 dependency-
         * injection choice), so the caller supplies speed for range_m. This is
         * the same signed two-wheel average cluster_comm uses for 0x200, so
         * both frames still describe the same motion.
         *
         * ℹ️ These two getters read the QEI HARDWARE registers directly
         * (encoder.c:203 -> QEI_GetVelocityMmPerSec), not RAM written by
         * another task - so peeling this into its own task introduces NO new
         * cross-task hazard. The two reads can still straddle a QEI window
         * update, giving a slightly mismatched L/R pair, but that is the
         * PRE-EXISTING A4-4 family and is unchanged by moving: the super-loop
         * did exactly these two back-to-back reads. Fixing it is B12's job, at
         * the encoder layer, for every consumer at once. */
        float32 vLeft  = Encoder_GetLinearVelocityM(ENCODER_ID_LEFT);
        float32 vRight = Encoder_GetLinearVelocityM(ENCODER_ID_RIGHT);
        float32 speedMps = (vLeft + vRight) * 0.5f;

        if (speedMps < 0.0f) { speedMps = -speedMps; }  /* range is direction-agnostic */

        /* ---- WCET instrumentation ----
         * TIMER0, not DWT: it is ALREADY running in this image (it is the I2C
         * timeout base, so it must be up before any INA226 read anyway), it is
         * a free-running 16 MHz counter read by ONE tear-free LDR, and unlike
         * DWT it needs no debug power. Budget §8.1 names it the preferred
         * on-vehicle instrument for exactly these reasons. 16 ticks = 1 us. */
        {
            uint32 t0 = Timer0_GetTicks();
            uint32 us;

            BatteryService_Update(speedMps);

            us = Timer0_ElapsedTicks(t0) / TIMER0_TICKS_PER_US;
            if (us > bt_maxUpdateUs) { bt_maxUpdateUs = us; }
        }

        bt_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        /* Period, not delay-after-work: vTaskDelayUntil keeps the 10 Hz rate
         * that BATTERY_TICKS_PER_SECOND depends on. ⚠️ That constant (=10) MUST
         * match this period - the coulomb integral scales DIRECTLY with tick
         * rate, so a mismatch does not make the gauge noisier, it makes it
         * wrong by that ratio, permanently and silently.
         *
         * Safe to use the catch-up form here (unlike tSuperLoop's slot loop,
         * which must never double-fire a residue): this task has no phase
         * relationship with anything, so if it ever overran, running again
         * promptly is exactly the right recovery - it keeps the coulomb
         * integral's tick count honest. */
        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BATTERY_TASK_PERIOD_MS));
    }
}

/*----------------------------------------------------------------------------
 * B13: tImu. 50 Hz MPU6050 read + the 0x150/0x160 pair. See app_cfg.h for why
 * this task both SAMPLES and TRANSMITS while tBattery only samples.
 *
 * Observability (all plain uint32 - single aligned words, atomic on M4, so the
 * heartbeat reads them without a guard, same discipline as every other peel):
 *   im_hwmWords     - stack high-water, [E] -> [M] for the budget.
 *   im_maxUpdateUs  - worst Update+pack+post seen. This is the number that says
 *                     whether a 15-command capped I2C burst really costs what
 *                     the budget estimated (150 us [E]).
 *   im_txDrops      - times the pair was NOT fully shipped. ★ Expected 0 once
 *                     the sensor is up; it counts the pre-first-sample gate, an
 *                     out-of-range/NaN reading, and a refused post. Non-zero
 *                     with a healthy IMU means the TX queue is under pressure.
 *--------------------------------------------------------------------------*/
static StaticTask_t im_tcb;
static StackType_t  im_stack[IMU_TASK_STACK_WORDS];

static volatile uint32 im_hwmWords    = 0U;
static volatile uint32 im_minUpdateUs = 0xFFFFFFFFUL;
static volatile uint32 im_maxUpdateUs = 0U;
static volatile uint32 im_txDrops     = 0U;

static void ImuTask(void *pvParameters)
{
    TickType_t lastWake;

    (void)pvParameters;

    lastWake = AppTask_PhaseAlign(IMU_TASK_PERIOD_MS, IMU_TASK_PHASE_MS);

    for (;;)
    {
        uint32 t0 = Timer0_GetTicks();
        uint32 us;

        /* ★ THE COHERENCY RULE, EXPRESSED AS ADJACENCY: exactly one Update, then
         * exactly one Send, with nothing between them. JetsonComm_SendImuFrames
         * reads the service's getters directly, so a second Update here - or a
         * second Send - would tear the sample the two frames are supposed to
         * share. Do not add a "send accel now, gyro next cycle" alternation: the
         * host keeps only the LAST frame of each ID per 30 Hz drain, so split
         * posts make its sequence-match check fail about half the time and it
         * discards the ENTIRE sensor read, encoder ticks included. */
        ImuService_Update();

        if (JetsonComm_SendImuFrames() != E_OK)
        {
            if (im_txDrops < 0xFFFFFFFFUL) { im_txDrops++; }
        }

        /* TIMER0, not DWT - same reasoning as tBattery: already running (it is
         * the I2C timeout base, so it must be up before any MPU6050 read),
         * free-running 16 MHz, one tear-free LDR. 16 ticks = 1 us. */
        /* ⚠️ MIN AND MAX, NOT JUST MAX - the B4 correction applies here too and
         * matters more for this task than for any other. Four tasks outrank
         * tImu, so the MAX is a RESPONSE TIME (execution + whatever preempted
         * it), not a WCET. The MIN is the clean-run figure and is the honest
         * input to the utilisation sum; the spread between them is the
         * interference. Reporting only the max would triple-count preemption
         * that the higher-priority tasks are already charged for. */
        us = Timer0_ElapsedTicks(t0) / TIMER0_TICKS_PER_US;
        if (us > im_maxUpdateUs) { im_maxUpdateUs = us; }
        if (us < im_minUpdateUs) { im_minUpdateUs = us; }

        im_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        /* Catch-up form is safe here: unlike tVelocity there is no
         * fire-early hazard. The MPU6050's own 50 Hz sample register simply
         * re-reads the same value if we ever came round twice quickly, which
         * costs a duplicate sequence and nothing else - the PAIR stays matched,
         * which is the only property the host actually checks. */
        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

/*----------------------------------------------------------------------------
 * B5: tOdo. Static allocation, lowest priority, 100 ms.
 *
 * Deliberately trivial - the whole state machine already lives in odo.c and is
 * already written to never block. All this task does is give it a context of
 * its own, at a priority where its millisecond-scale hardware latency cannot
 * matter to anything.
 *--------------------------------------------------------------------------*/
static StaticTask_t od_tcb;
static StackType_t  od_stack[ODO_TASK_STACK_WORDS];

static volatile uint32 od_hwmWords = 0U;


static void OdoTask(void *pvParameters)
{
    TickType_t lastWake;

    (void)pvParameters;

    /* Phase-lock to the old slot residue (53 mod 100) - same reasoning, and the
     * same latent bug, as tBattery: see the long note in BatteryTask. */
    lastWake = AppTask_PhaseAlign(ODO_TASK_PERIOD_MS, ODO_TASK_PHASE_MS);

    for (;;)
    {
        Odo_MainFunction();

        od_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        /* Catch-up form is fine here for the same reason as tBattery: this task
         * has no phase relationship with anything, and the save state machine
         * is self-pacing (one word per call, gated on Eeprom_IsBusy). Running
         * promptly after an overrun just drains the save sooner. */
        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ODO_TASK_PERIOD_MS));
    }
}

/*----------------------------------------------------------------------------
 * B7: tRosTx - 0x110 VelocityFeedback / 0x130 SteeringFeedback, alternating.
 *
 * Both sources are coherent by construction now:
 *   0x110 -> CommData_GetWheelTicks(), made atomic in B7 (A4-4 / V9-R1)
 *   0x130 -> SteeringControl_GetSnapshot(), coherent since S10-1
 * so neither frame can carry a torn pair even though a higher-priority task can
 * preempt this one at any instruction.
 *--------------------------------------------------------------------------*/
static StaticTask_t rt_tcb;
static StackType_t  rt_stack[ROSTX_TASK_STACK_WORDS];
static volatile uint32 rt_hwmWords = 0U;

static void RosTxTask(void *pvParameters)
{
    TickType_t lastWake;
    boolean    sendVelocity = TRUE;

    (void)pvParameters;

    lastWake = AppTask_PhaseAlign(ROSTX_TASK_PERIOD_MS, ROSTX_TASK_PHASE_MS);

    for (;;)
    {
        if (sendVelocity != FALSE)
        {
            (void)JetsonComm_SendVelocityFeedback();   /* 0x110 */
        }
        else
        {
            (void)JetsonComm_SendSteeringFeedback();   /* 0x130 */
        }
        sendVelocity = (sendVelocity != FALSE) ? FALSE : TRUE;

        rt_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ROSTX_TASK_PERIOD_MS));
    }
}

/*----------------------------------------------------------------------------
 * B7: tClusterTx - 0x200 VehicleStatus / 0x210 BatteryStatus, alternating,
 * plus the 0x7A1 ping response.
 *
 * Sources: BatteryService_GetStatus (coherent since B4's deferred commit), and
 * Odo_GetMetres / the trip accumulator (single aligned words, atomic on M4).
 *--------------------------------------------------------------------------*/
static StaticTask_t cl_tcb;
static StackType_t  cl_stack[CLUSTERTX_TASK_STACK_WORDS];
static volatile uint32 cl_hwmWords = 0U;
static boolean         cl_batteryOk = FALSE;

static void ClusterTxTask(void *pvParameters)
{
    TickType_t lastWake;
    boolean    sendVehicle = TRUE;

    (void)pvParameters;

    lastWake = AppTask_PhaseAlign(CLUSTERTX_TASK_PERIOD_MS, CLUSTERTX_TASK_PHASE_MS);

    for (;;)
    {
        if (sendVehicle != FALSE)
        {
            (void)ClusterComm_SendVehicleStatus();     /* 0x200 */
        }
        else if (cl_batteryOk != FALSE)
        {
            (void)ClusterComm_SendBatteryStatus();     /* 0x210 */
        }
        else
        {
            /* Battery chain never came up: the old loop skipped this slot, and
             * so do we. Same containment, expressed as a skipped alternation. */
        }
        sendVehicle = (sendVehicle != FALSE) ? FALSE : TRUE;

        /* 0x7A1 liveness echo. 🔶 DELIBERATELY NOT GATED on batteryOk or any
         * init flag - the ping reports only "this node is powered and its CAN
         * stack is alive", and withholding it because a service is unhappy would
         * report this node as DEAD (node_ping.h). */
        (void)NodePing_MainFunction();

        cl_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CLUSTERTX_TASK_PERIOD_MS));
    }
}

/*----------------------------------------------------------------------------
 * B8: tRosRx - drains the CAN RX ring and routes each frame.
 *
 * JetsonComm_Poll() is UNCHANGED: it already drains-all-per-call (its own
 * comment calls it "the node's ONE drain point"), which is exactly the shape an
 * event-driven consumer wants. Only WHERE it runs has moved.
 *--------------------------------------------------------------------------*/
static StaticTask_t rr_tcb;
static StackType_t  rr_stack[ROSRX_TASK_STACK_WORDS];
static volatile uint32 rr_hwmWords = 0U;

static void RosRxTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        /* Sleep until the ISR says the ring has something, or the backstop
         * expires. The return value is deliberately ignored: on BOTH paths the
         * right action is the same - drain the ring. Treating a timeout as an
         * error would mean an idle bus looked like a fault. */
        (void)Can_WaitRxReady(ROSRX_WAIT_TIMEOUT_MS);

        JetsonComm_Poll();

        rr_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);
    }
}

/*----------------------------------------------------------------------------
 * B9: tVelocity. VelocityControl_Update() is called VERBATIM - only where and
 * when it runs has changed.
 *
 * Instrumentation (diagnostics only, no logic):
 *   vt_minIntervalMs / vt_maxIntervalMs - the measured gap between consecutive
 *     Updates. ★ vt_min >= 20 IS THE PROOF THE GUARD WORKS. A single sub-20
 *     reading would mean a stale QEISPEED reached the PID.
 *   vt_skipped - cycles the guard refused. Expected 0 on a healthy system;
 *     non-zero means xTaskDelayUntil caught up after an overrun and the guard
 *     did its job.
 *   vt_maxUpdateUs - timer0-measured Update cost. Nothing but tSafety (not yet
 *     peeled) and ISRs can preempt this task, so unlike bt_wcet the MINIMUM of
 *     this is very close to true execution time; the max may include one CAN
 *     ISR (~15 us).
 *--------------------------------------------------------------------------*/
static StaticTask_t vt_tcb;
static StackType_t  vt_stack[VELOCITY_TASK_STACK_WORDS];
static volatile uint32 vt_hwmWords     = 0U;
static volatile uint32 vt_minIntervalMs = 0xFFFFFFFFUL;
static volatile uint32 vt_maxIntervalMs = 0U;
static volatile uint32 vt_skipped       = 0U;
static volatile uint32 vt_minUpdateUs   = 0xFFFFFFFFUL;
static volatile uint32 vt_maxUpdateUs   = 0U;

static void VelocityTask(void *pvParameters)
{
    TickType_t lastWake;
    TickType_t lastUpdateTick;
    boolean    firstUpdate = TRUE;

    (void)pvParameters;

    lastWake = AppTask_PhaseAlign(VEL_PERIOD_MS, VELOCITY_TASK_PHASE_MS);

    /* Seed so the FIRST wake is allowed through: unsigned wrap-safe, so
     * subtracting one period below zero is fine and yields >= VEL_PERIOD_MS. */
    lastUpdateTick = lastWake - (TickType_t)pdMS_TO_TICKS(VEL_PERIOD_MS);

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();
        uint32     interval = (uint32)(now - lastUpdateTick);

        /* ★ THE GUARD. This is `lastVelMs = now` from the super-loop, and it is
         * what makes the QEI window a structural property rather than an
         * assumption about how the scheduler behaved. */
        if (interval >= (uint32)pdMS_TO_TICKS(VEL_PERIOD_MS))
        {
            uint32 t0 = Timer0_GetTicks();
            uint32 us;

            VelocityControl_Update();

            us = Timer0_ElapsedTicks(t0) / TIMER0_TICKS_PER_US;
            if (us > vt_maxUpdateUs) { vt_maxUpdateUs = us; }
            if (us < vt_minUpdateUs) { vt_minUpdateUs = us; }

            /* RE-REFERENCE to now, exactly as the super-loop did - never
             * `lastUpdateTick += period`, which would re-admit catch-up. */
            lastUpdateTick = now;

            if (firstUpdate != FALSE)
            {
                firstUpdate = FALSE;      /* the seeded interval is not real */
            }
            else
            {
                if (interval < vt_minIntervalMs) { vt_minIntervalMs = interval; }
                if (interval > vt_maxIntervalMs) { vt_maxIntervalMs = interval; }
            }
        }
        else
        {
            /* An early (catch-up) wake. Skipping is the SAFE direction: a
             * missed window costs one control cycle; a stale QEISPEED costs
             * closed-loop stability. */
            if (vt_skipped < 0xFFFFFFFFUL) { vt_skipped++; }
        }

        vt_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(VEL_PERIOD_MS));
    }
}

/*----------------------------------------------------------------------------
 * B10: tSafety. The A4-1 logic, moved verbatim - every comment that justified
 * it in the super-loop still applies and is preserved at the decision points.
 *
 * State is file-scope rather than local because the heartbeat (in tSuperLoop)
 * reports it. All plain uint32/boolean => single aligned words, atomic on
 * Cortex-M4, so the cross-task read needs no guard - the same argument as the
 * command counters themselves (B8).
 *--------------------------------------------------------------------------*/
static StaticTask_t sf_tcb;
static StackType_t  sf_stack[SAFETY_TASK_STACK_WORDS];
static volatile uint32  sf_hwmWords        = 0U;
static volatile uint32  sf_failsafeCount   = 0U;   /* sticky episode count */
static volatile boolean sf_failsafeActive  = FALSE;
static volatile boolean sf_everSeen        = FALSE;

static void SafetyTask(void *pvParameters)
{
    TickType_t lastWake;
    uint32     lastCmdActivityMs;
    uint32     lastCmdSeen;

    (void)pvParameters;

    lastWake = AppTask_PhaseAlign(SAFETY_TASK_PERIOD_MS, SAFETY_TASK_PHASE_MS);

    /* Seed the deadline at boot rather than at zero (see the cmdEverSeen note). */
    lastCmdActivityMs = (uint32)xTaskGetTickCount();
    lastCmdSeen       = JetsonComm_GetVelocityCommandCount() +
                        JetsonComm_GetSteeringCommandCount();

    for (;;)
    {
        uint32 now = (uint32)xTaskGetTickCount();

        /* MECHANISM: jetson_comm publishes monotonic counts of ACCEPTED
         * commands. We watch for CHANGE, not for a rate - so this stays correct
         * whatever cadence the Jetson turns out to use.
         *
         * ⚠️ The two channels are COMBINED deliberately: the trip rule fires
         * only when NEITHER has advanced, i.e. on "the host is gone" rather than
         * "one channel went quiet". Tripping per-channel would demand that BOTH
         * streams be periodic - an assumption we cannot make about an unmeasured
         * host, and getting it wrong means stopping a vehicle whose controller
         * is demonstrably alive and still steering it.
         *
         * ⚠️ B8: this read is cross-task (tRosRx writes) and is coherent without
         * a lock - each counter is an atomic uint32 and both are monotonic, so a
         * torn SUM still differs iff one advanced, which is the only question
         * asked. */
        uint32 cmdSeen = JetsonComm_GetVelocityCommandCount() +
                         JetsonComm_GetSteeringCommandCount();

        if (cmdSeen != lastCmdSeen)
        {
            /* Fresh accepted command: re-arm. The setpoint it carried has
             * ALREADY been applied by tRosRx - and that same SetSetpoint call
             * cleared the V9-R4 inhibit latch - so recovery needs no action here
             * beyond clearing our own latch. Certainly no power-cycle. */
            lastCmdSeen       = cmdSeen;
            lastCmdActivityMs = now;
            sf_failsafeActive = FALSE;
            sf_everSeen       = TRUE;
        }
        /* ⚠️ `everSeen` GATES THE TRIP, AND IT IS NOT COSMETIC. Without it the
         * failsafe fires CMD_TIMEOUT_MS after EVERY boot on any bench with no
         * host attached - observed on hardware, first run. The vehicle is
         * already stopped at that point, so the ACTION was harmless; what it
         * wrecked was the SIGNAL. A failsafe counts LOSS, and you cannot lose
         * what you never had, so "never commanded" is a distinct state. */
        else if ((sf_everSeen != FALSE) &&
                 (sf_failsafeActive == FALSE) &&
                 ((now - lastCmdActivityMs) >= CMD_TIMEOUT_MS))
        {
            /* R8-1: Stop(), NOT SetSetpoint(0,0) - a PID latched in FAULT_NAN
             * ignores the setpoint and keeps driving. Un-gated by design (V9-3):
             * a failsafe must fire even if a module never initialised.
             *
             * ⚠️ V9-R4: Stop() also SETS the inhibit latch, which is what makes
             * this stop survive preempting tVelocity mid-Update(). Without the
             * latch this call could be silently undone. See velocity_control.c.
             *
             * ⚠️ STEERING IS NOT TOUCHED. That IS the action: HOLD the last
             * angle (S10-5). SteeringControl_Center() must NOT be called here -
             * snapping the wheels straight mid-corner is its own hazard. */
            VelocityControl_Stop();

            sf_failsafeActive = TRUE;
            if (sf_failsafeCount < 0xFFFFFFFFUL) { sf_failsafeCount++; }

            /* Latched, so this runs ONCE per loss episode. Nothing can rewrite
             * the setpoint while no commands are accepted, so re-applying would
             * only re-reset the PIDs and spam the log. */
            UART_SendString(DIAG_UART,
                "# FAILSAFE: no accepted 0x100/0x120 for ");
            UART_SendInteger(DIAG_UART, (sint32)CMD_TIMEOUT_MS);
            UART_SendString(DIAG_UART,
                " ms -> velocity ZEROED, steering HELD\r\n");
        }
        else
        {
            /* armed and fresh, or already latched - nothing to do */
        }

        sf_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS));
    }
}


/*----------------------------------------------------------------------------
 * B11 - tBusHealth (prio 5): the C6-1 CAN bus-off check.
 *
 * The second-to-last resident of the super-loop, moved VERBATIM. It gets its
 * own task rather than being folded into tHeartbeat because the two answer
 * completely different questions - "is the bus alive?" versus "what are the
 * numbers?" - and only one of them commands the vehicle to stop. Merging a
 * safety action into a diagnostics task would be exactly the kind of tidy-
 * looking coupling that is painful later.
 *
 * ⚠️ PRIORITY 5 IS DELIBERATE: it is the level tSuperLoop itself occupied, so
 * this check keeps EXACTLY the relative position it has had all along - below
 * tCanTx/tBattery/control, above the telemetry producers. B11 is a structural
 * step and must not quietly re-rank a safety action.
 *
 * ℹ️ Since B10 this path also benefits from the V9-R4 latch: VelocityControl_
 * Stop() sets it, so an Update() between two 100 ms re-assertions can no longer
 * drive the motors again.
 *--------------------------------------------------------------------------*/
static StaticTask_t bh_tcb;
static StackType_t  bh_stack[BUSHEALTH_TASK_STACK_WORDS];
static volatile uint32 bh_hwmWords         = 0U;
static volatile uint32 bh_recoverAttempts  = 0U;

static void BusHealthTask(void *pvParameters)
{
    TickType_t lastWake;

    (void)pvParameters;

    lastWake = AppTask_PhaseAlign(BUSHEALTH_TASK_PERIOD_MS, BUSHEALTH_TASK_PHASE_MS);

    for (;;)
    {
        /* C6-1: the ISR only OBSERVES bus-off (recovery involves a rejoin delay
         * that must not run in interrupt context), so the recovery call belongs
         * in a task.
         *
         * SAFETY POLICY on bus-off, unchanged: command ZERO VELOCITY. Without
         * it the last velocity setpoint persists and the vehicle keeps driving
         * with its comms dead - "recovered the bus but kept driving on a stale
         * command" is only half a fix.
         *
         * ⚠️ Steering is deliberately NOT re-centred - the same S10-5 reasoning
         * as the failsafe: snapping the wheels straight mid-corner is its own
         * hazard. Stop driving, hold the wheel.
         *
         * R8-1: Stop(), NOT SetSetpoint(0,0) - a PID latched in FAULT_NAN
         * ignores the setpoint and keeps returning its last output. */
        if (Can_GetLastBusError() == CAN_ERROR_BUS_OFF)
        {
            VelocityControl_Stop();                    /* safe state first */
            (void)Can_RecoverBusOff();                 /* then rejoin      */
            if (bh_recoverAttempts < 0xFFFFFFFFUL) { bh_recoverAttempts++; }
        }

        bh_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUSHEALTH_TASK_PERIOD_MS));
    }
}

/*----------------------------------------------------------------------------
 * B11 - tHeartbeat (prio 1, LOWEST): the 1 Hz debug console line.
 *
 * The LAST resident of the super-loop. Pure diagnostics, so it sits at the very
 * bottom of the ladder: a late heartbeat is harmless, and this is by far the
 * longest task body in the system (80-odd UART calls), so it must never be able
 * to delay anything.
 *
 * ⚠️ EVERY VALUE IT PRINTS IS A CROSS-TASK READ, and every one is a plain
 * uint32/boolean - a single aligned word, atomic on Cortex-M4. So this task
 * needs no guard anywhere, which is why the observability counters were all
 * declared that way as each task was peeled. A struct-valued counter would have
 * needed the B4 treatment; none was ever introduced.
 *
 * ⚠️ UART_SendString pushes into a ring buffer and drops on overflow rather
 * than blocking (128-spin cap per byte), so even at the lowest priority this
 * cannot stall - it degrades by losing console text, which is the right
 * failure for diagnostics.
 *--------------------------------------------------------------------------*/
static StaticTask_t hb_tcb;
static StackType_t  hb_stack[HEARTBEAT_TASK_STACK_WORDS];
static volatile uint32 hb_hwmWords = 0U;
static boolean         hb_batteryOk = FALSE;
static boolean         hb_imuOk     = FALSE;

static void HeartbeatTask(void *pvParameters)
{
    TickType_t lastWake;

    (void)pvParameters;

    lastWake = AppTask_PhaseAlign(HEARTBEAT_TASK_PERIOD_MS, HEARTBEAT_TASK_PHASE_MS);

    for (;;)
    {
        UART_SendString(DIAG_UART, "# alive t=");
        UART_SendInteger(DIAG_UART, (sint32)xTaskGetTickCount());
        UART_SendString(DIAG_UART, " bus_err=");
        UART_SendInteger(DIAG_UART, (sint32)Can_GetLastBusError());
        UART_SendString(DIAG_UART, " rx_overrun=");
        UART_SendInteger(DIAG_UART, (sint32)Can_GetRxOverrunCount());
        UART_SendString(DIAG_UART, " can_recov=");
        UART_SendInteger(DIAG_UART, (sint32)bh_recoverAttempts);

        /* A4-1: sticky failsafe EPISODE count, not a level - a non-zero count
         * with FS:0 means "it happened and recovered", which is exactly the
         * history a bench operator needs and which a bare boolean would lose. */
        UART_SendString(DIAG_UART, " cmd_fs=");
        UART_SendInteger(DIAG_UART, (sint32)sf_failsafeCount);

        /* FS:W = never commanded (deadline not policed yet)
         * FS:0 = armed, host fresh
         * FS:1 = failsafe LATCHED, velocity zeroed, steering held
         * The three are distinct on purpose: "W" and "0" would otherwise be
         * indistinguishable on a bench and mean very different things about
         * whether the vehicle is being watched. */
        if      (sf_everSeen == FALSE) { UART_SendString(DIAG_UART, " FS:W"); }
        else if (sf_failsafeActive   != FALSE) { UART_SendString(DIAG_UART, " FS:1"); }
        else                                        { UART_SendString(DIAG_UART, " FS:0"); }

        /* B10 / V9-R4: the LATCH itself, so an operator can see the stop is
         * being HELD rather than merely having been issued once. */
        UART_SendString(DIAG_UART,
            (VelocityControl_IsInhibited() != FALSE) ? " INH:1" : " INH:0");

        if (hb_batteryOk != FALSE)
        {
            BatteryStatusType batt;
            (void)BatteryService_GetStatus(&batt);
            UART_SendString(DIAG_UART, " batt=");
            UART_SendInteger(DIAG_UART, batt.voltage_mV);
            UART_SendString(DIAG_UART, "mV/");
            UART_SendInteger(DIAG_UART, batt.current_mA);
            UART_SendString(DIAG_UART, "mA soc=");
            UART_SendFloat(DIAG_UART, batt.soc_pct, 1U);
            UART_SendString(DIAG_UART, "% flags=");
            UART_SendInteger(DIAG_UART, (sint32)batt.flags);
        }
        else
        {
            UART_SendString(DIAG_UART, " batt=DISABLED");
        }

        /* B13: the IMU stream, in the form a bench operator needs to answer
         * "is the Jetson's R-E odometry gate satisfied?" without a candump.
         * imu_seq ADVANCING is the liveness proof (it only increments on a good
         * sample); H/S is health vs the merely-held sample; rst is the 0x160
         * imu_reset bit; drop counts pairs that did not fully ship. */
        if (hb_imuOk != FALSE)
        {
            UART_SendString(DIAG_UART, " imu=");
            UART_SendString(DIAG_UART,
                (ImuService_HasSample() == FALSE) ? "NOSAMPLE" :
                ((ImuService_IsHealthy() != FALSE) ? "H" : "S"));
            UART_SendString(DIAG_UART, "/seq");
            UART_SendInteger(DIAG_UART, (sint32)ImuService_GetSequence());
            UART_SendString(DIAG_UART,
                (ImuService_GetResetFlag() != FALSE) ? "/rst1" : "/rst0");
            UART_SendString(DIAG_UART, " im=");
            UART_SendInteger(DIAG_UART, (sint32)im_hwmWords);
            UART_SendString(DIAG_UART, "/");
            UART_SendInteger(DIAG_UART, (sint32)im_minUpdateUs);
            UART_SendString(DIAG_UART, "-");
            UART_SendInteger(DIAG_UART, (sint32)im_maxUpdateUs);
            UART_SendString(DIAG_UART, "us drop=");
            UART_SendInteger(DIAG_UART, (sint32)im_txDrops);
        }
        else
        {
            UART_SendString(DIAG_UART, " imu=DISABLED");
        }

        /* ---- per-task stacks + the instrumentation each peel added ---- */
        UART_SendString(DIAG_UART, " hb=");
        UART_SendInteger(DIAG_UART, (sint32)hb_hwmWords);
        UART_SendString(DIAG_UART, " bh=");
        UART_SendInteger(DIAG_UART, (sint32)bh_hwmWords);
        UART_SendString(DIAG_UART, " bt=");
        UART_SendInteger(DIAG_UART, (sint32)bt_hwmWords);
        UART_SendString(DIAG_UART, "/");
        UART_SendInteger(DIAG_UART, (sint32)bt_maxUpdateUs);
        UART_SendString(DIAG_UART, "us od=");
        UART_SendInteger(DIAG_UART, (sint32)od_hwmWords);
        UART_SendString(DIAG_UART, " rt=");
        UART_SendInteger(DIAG_UART, (sint32)rt_hwmWords);
        UART_SendString(DIAG_UART, " cl=");
        UART_SendInteger(DIAG_UART, (sint32)cl_hwmWords);
        UART_SendString(DIAG_UART, " rr=");
        UART_SendInteger(DIAG_UART, (sint32)rr_hwmWords);
        UART_SendString(DIAG_UART, " vt=");
        UART_SendInteger(DIAG_UART, (sint32)vt_hwmWords);
        UART_SendString(DIAG_UART, " sf=");
        UART_SendInteger(DIAG_UART, (sint32)sf_hwmWords);
        UART_SendString(DIAG_UART, " ct=");
        UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetStackHwm());

        UART_SendString(DIAG_UART, " odo=");
        UART_SendInteger(DIAG_UART, (sint32)Odo_GetMetres());
        UART_SendString(DIAG_UART, "m saves=");
        UART_SendInteger(DIAG_UART, (sint32)Odo_GetSaveCount());
        UART_SendString(DIAG_UART, "/slot");
        UART_SendInteger(DIAG_UART, (sint32)Odo_GetNextSlot());

        /* ★ B9: vel_ivl min MUST never read below 20 - it is the proof the QEI
         * window guard holds. skip counts refused early wakes. */
        UART_SendString(DIAG_UART, " vel_ivl=");
        UART_SendInteger(DIAG_UART, (sint32)vt_minIntervalMs);
        UART_SendString(DIAG_UART, "/");
        UART_SendInteger(DIAG_UART, (sint32)vt_maxIntervalMs);
        UART_SendString(DIAG_UART, "ms skip=");
        UART_SendInteger(DIAG_UART, (sint32)vt_skipped);
        UART_SendString(DIAG_UART, " vel_us=");
        UART_SendInteger(DIAG_UART, (sint32)vt_minUpdateUs);
        UART_SendString(DIAG_UART, "/");
        UART_SendInteger(DIAG_UART, (sint32)vt_maxUpdateUs);

        /* B6 / A4-3: three DISTINCT causes, deliberately not lumped. */
        UART_SendString(DIAG_UART, " tx=");
        UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetTxCount());
        UART_SendString(DIAG_UART, " peak=");
        UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetPeakDepth());
        UART_SendString(DIAG_UART, " qfull=");
        UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetQueueFullDrops());
        UART_SendString(DIAG_UART, " txfail=");
        UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetTxFailCount());
        UART_SendString(DIAG_UART, " txtmo=");
        UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetTxTimeoutCount());
        UART_SendString(DIAG_UART, "\r\n");

        hb_hwmWords = (uint32)uxTaskGetStackHighWaterMark(NULL);

        (void)xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HEARTBEAT_TASK_PERIOD_MS));
    }
}

/*============================================================================
 *  App_Start - create the task set, then hand the CPU to the scheduler.
 *
 * ⚠️ EVERY CREATION HALTS ON FAILURE, without exception. With static allocation
 * xTaskCreateStatic can only fail on a NULL buffer, i.e. a programming error -
 * there is no "out of memory" outcome to degrade around, which is precisely why
 * static allocation was chosen (budget section 1). A vehicle missing any one of
 * these tasks is a vehicle that should not move.
 *
 * ORDER: creation order does not matter to FreeRTOS (nothing runs until the
 * scheduler starts), but the queue/semaphore owners are created first so their
 * objects exist before any task could reference them.
 *==========================================================================*/
#define APP_CREATE(fn, name, stackWords, prio, stack, tcb)                    \
    do {                                                                      \
        if (xTaskCreateStatic((fn), (name), (stackWords), NULL, (prio),       \
                              (stack), &(tcb)) == NULL)                       \
        {                                                                     \
            UART_SendString(DIAG_UART, "# xTaskCreateStatic(" name            \
                                       ") FAILED - HALTED\r\n");             \
            while (1) { }                                                     \
        }                                                                     \
    } while (0)

Std_ReturnType App_Start(boolean batteryOk, boolean imuOk)
{
    hb_batteryOk = batteryOk;
    cl_batteryOk = batteryOk;
    hb_imuOk     = imuOk;

    /* Transport first: this creates the CAN TX queue AND the ISR's TX-complete
     * semaphore, which tCanTx blocks on. */
    if (CanTxQueue_Init() != E_OK)
    {
        UART_SendString(DIAG_UART, "# CanTxQueue_Init FAILED - no TX possible - HALTED\r\n");
        while (1) { }
    }
    /* ...and the RX wake semaphore, before the task that waits on it. */
    Can_RxReadySignalInit();

    APP_CREATE(SafetyTask,    "SAFETY", SAFETY_TASK_STACK_WORDS,    SAFETY_TASK_PRIORITY,    sf_stack, sf_tcb);
    APP_CREATE(VelocityTask,  "VEL",    VELOCITY_TASK_STACK_WORDS,  VELOCITY_TASK_PRIORITY,  vt_stack, vt_tcb);
    APP_CREATE(RosRxTask,     "ROSRX",  ROSRX_TASK_STACK_WORDS,     ROSRX_TASK_PRIORITY,     rr_stack, rr_tcb);
    if (batteryOk != FALSE)
    {
        /* Created ONLY when the battery chain came up - the old loop simply
         * skipped both slots, and a task polling a dead I2C bus 10 times a
         * second would be strictly worse. Same containment, expressed as
         * "no task" instead of "skipped slot". */
        APP_CREATE(BatteryTask, "BATT", BATTERY_TASK_STACK_WORDS, BATTERY_TASK_PRIORITY, bt_stack, bt_tcb);
    }
    if (imuOk != FALSE)
    {
        /* ⚠️ GATED ON THE I2C BUS, NOT ON THE SENSOR - and that is the whole
         * difference from tBattery above. If the bus itself never came up there
         * is nothing to retry, so no task. But a sensor that merely failed to
         * answer at boot is NOT a reason to skip: imu_service retries on its own
         * backoff and starts producing when the part appears, and until then the
         * HasSample gate keeps it silent rather than publishing zeroes. */
        APP_CREATE(ImuTask, "IMU", IMU_TASK_STACK_WORDS, IMU_TASK_PRIORITY, im_stack, im_tcb);
    }
    APP_CREATE(BusHealthTask, "BUSHLTH", BUSHEALTH_TASK_STACK_WORDS, BUSHEALTH_TASK_PRIORITY, bh_stack, bh_tcb);
    APP_CREATE(RosTxTask,     "ROSTX",  ROSTX_TASK_STACK_WORDS,     ROSTX_TASK_PRIORITY,     rt_stack, rt_tcb);
    APP_CREATE(ClusterTxTask, "CLUSTX", CLUSTERTX_TASK_STACK_WORDS, CLUSTERTX_TASK_PRIORITY, cl_stack, cl_tcb);
    APP_CREATE(OdoTask,       "ODO",    ODO_TASK_STACK_WORDS,       ODO_TASK_PRIORITY,       od_stack, od_tcb);
    APP_CREATE(HeartbeatTask, "HBEAT",  HEARTBEAT_TASK_STACK_WORDS, HEARTBEAT_TASK_PRIORITY, hb_stack, hb_tcb);

    UART_SendString(DIAG_UART,
        "# RTOS: task set up - SAFETY(11) VEL(10) ROSRX(9) BATT(8) IMU(7) "
        "CANTX(6) BUSHLTH(5) ROSTX(4) CLUSTX(3) ODO(2) HBEAT(1). No super-loop.\r\n");

    vTaskStartScheduler();

    /* Unreachable while the scheduler runs. With static allocation this can only
     * mean the Idle task could not be created - a configuration error, not heap
     * exhaustion, because there is no heap. */
    return E_NOT_OK;
}

#else  /* !USE_FREERTOS ------------------------------------------------------ */

Std_ReturnType App_Start(boolean batteryOk, boolean imuOk)
{
    /* No RTOS in this build: main() runs the legacy super-loop instead. */
    (void)batteryOk;
    (void)imuOk;
    return E_NOT_OK;
}

#endif /* USE_FREERTOS */
