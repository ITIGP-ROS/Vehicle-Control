/******************************************************************************
 * app_cfg.h - timing and sizing constants shared by the application layer.
 *
 * Created at B11, when app.c took over task creation. These constants were
 * defined in main.c and used only there, because main.c WAS the application.
 * Once the tasks moved to app.c the two files needed the same numbers, so they
 * live here - the repo's existing X_cfg.h convention.
 *
 * ⚠️ Included by BOTH builds. The RTOS build (app.c) uses the task
 * stack/priority/period/phase values; the non-RTOS rollback build (main.c's
 * SuperLoop_Run) uses the periods and the slot residues. Anything used by only
 * one of them is marked.
 *****************************************************************************/

#ifndef APP_CFG_H_
#define APP_CFG_H_

#include "app_priorities.h"

#ifdef USE_FREERTOS
#define SYS_TICK_MS()    ((uint32)xTaskGetTickCount())
#else
#define SYS_TICK_MS()    (SysTick_GetTickCount())
#endif

#define DIAG_UART        UART_ID_1
#define VEL_PERIOD_MS    (20U)      /* control cadence: QEI window / PID Ts (0.02 s) */
#define HEARTBEAT_MS     (1000U)    /* debug-only UART heartbeat cadence             */

/* C6-1: how often the loop checks CAN bus health and, if bus-off, retries
 * recovery. Bus-off rejoin needs 128 x 11 idle bit-times (~2.8 ms at 500 kbps),
 * so retrying every 100 ms is far above the hardware minimum - it recovers
 * promptly without hammering the controller. */
#define CAN_HEALTH_PERIOD_MS  (100U)

/*----------------------------------------------------------------------------
 * A4-1: RX COMMAND-LOSS FAILSAFE - the deadline.
 *
 * ✅ SET FROM MEASUREMENT 2026-08-06. Was 500 ms provisional from 2026-08-05,
 * chosen when the Jetson's cadence had never been observed and the file warned
 * in capitals that it MUST be re-derived before any on-ground operation.
 *
 * HOW 150 WAS EARNED, in two steps:
 *   1. The cadence was READ FROM THE ROS SOURCE (2026-08-06): update_rate 30 in
 *      controllers.yaml:3, with steering and velocity both written
 *      UNCONDITIONALLY every ros2_control cycle (ackermann_system.cpp:430,:435).
 *      So the period is 33.3 ms, fixed-rate - not on-receive.
 *          150 ms = 33.3 x 4.5, inside the 3-5x rule (100-167 ms).
 *   2. The tail was MEASURED on the wire with tools/mock_jetson.py, which
 *      reproduces that behaviour frame-for-frame:
 *          per channel : p50 33.3 | p90 41.5 | p99 51.2 | max 63.3 ms
 *          any command : p50  0.5 | p99 47.6 | max 63.2 ms
 *      (p50 is 0.5 ms for "any command" because the two frames go back-to-back
 *       within one cycle - the failsafe re-arms on EITHER, so that is the gap
 *       it actually sees.)
 *      => 150 ms is 2.9x the p99 and 2.4x the worst gap observed. Normal
 *         scheduler jitter cannot trip it.
 *
 * ⚠️ WHAT THE MEASUREMENT DOES *NOT* PROVE. The mock injects its own jitter,
 * deliberately pessimistic (sigma 4 ms plus a 3 % heavy tail), because the ROS
 * tree has NO real-time configuration at all - no SCHED_FIFO, no priority, no
 * memory locking - so 30 Hz is a soft target on a plain Linux loop. The tail
 * above is therefore MODELLED, not the real Jetson's. It is a deliberate
 * over-estimate, so the direction of any error is safe; but the real host's p99
 * still wants confirming when the real host is on the bus.
 *
 * WHAT IT COSTS: at 0.7-2 m/s, 150 ms of stale command carries the vehicle
 * 0.11-0.30 m before the stop - down from 0.35-1.00 m at 500 ms.
 * ⚠️ Plus ~0.4 s of MECHANICAL coast, measured in the A4-1 hardware run. No
 * timeout removes that; below ~200 ms it dominates the total, which is why
 * going tighter than 150 buys almost nothing while spending jitter margin.
 *
 * ⚠️ THE on_activate INTERACTION IS EXPECTED AND DOCUMENTED - see the note at
 * the failsafe in app.c. The ROS node sends ONE centring steering frame and
 * then sleeps 1500 ms, which is 10x this deadline, so the failsafe trips once
 * during activation and re-arms the instant the stream starts. Harmless: the
 * vehicle is stationary and unarmed at that point. Do NOT raise this value to
 * suppress it.
 *--------------------------------------------------------------------------*/
#define CMD_TIMEOUT_MS        (150U)

/* How often the loop evaluates the command-loss deadline. Shares the C6-1
 * cadence: both are "is the command source still there?" checks, both act by
 * calling VelocityControl_Stop(), and 100 ms granularity is negligible against
 * a 500 ms deadline. */
#define CMD_CHECK_PERIOD_MS   (CAN_HEALTH_PERIOD_MS)

/* A3 battery telemetry slots, both inside the per-millisecond edge guard.
 * BATTERY_TICKS_PER_SECOND (=10) MUST match this 100 ms spacing: the coulomb
 * integral scales DIRECTLY with tick rate, so a mismatch does not make the
 * gauge noisier, it makes it wrong by that ratio, permanently and silently. */
#define BATT_UPDATE_SLOT_MS   (43U)   /* I2C read + estimator, ~480 us, NO TX  */
#define BATT_TX_SLOT_MS       (47U)

/* 0x7A1 NodePingRespTiva. EXCLUSIVITY PROOF against every other TX slot:
 *   51 % 10  = 1  -> never collides with 0x110 (0 mod 10) or 0x130 (5 mod 10)
 *   51 % 100 = 51 -> never collides with 0x200 (9), the battery read (43) or
 *                    0x210 (47)
 * so at most one Can_Transmit still happens per millisecond. The slot comes
 * round every 100 ms against a ~1 Hz ping, so reply latency is bounded by
 * 100 ms and is a non-issue - the host is asking "are you alive", not timing us. */
#define PING_TX_SLOT_MS       (51U)

/* Odometer EEPROM save state machine. NOT a Can_Transmit - it issues at most
 * one EEPROM word write per call and never spins - so it does not participate
 * in the one-transmit-per-ms invariant and may share a millisecond with a TX
 * slot. It is given its own slot anyway, off every TX slot, so the EEPROM work
 * and a frame are never in the same millisecond:
 *   53 % 10 = 3 (not 0, not 5), and 53 is not 9, 43, 47 or 51.
 * A save needs two word writes, so a full cycle takes ~3 slots = ~300 ms - far
 * inside the ~20 s between saves at this robot's speed. */
#define ODO_SLOT_MS           (53U)   /* 0x210, 4 ms later - sample is fresh   */

#ifdef USE_FREERTOS
/*----------------------------------------------------------------------------
 * B3 - THE SWITCHOVER. This file now has TWO build modes, and the difference
 * between them is deliberately as small as it can possibly be.
 *
 *   WITHOUT USE_FREERTOS : byte-identical to the pre-B3 firmware. The init
 *                          block runs, then SuperLoop_Run() spins forever on
 *                          our 1 ms SysTick. This is the ROLLBACK and the
 *                          proof-of-no-harm - it is still built and verified.
 *
 *   WITH USE_FREERTOS    : the SAME init block runs, then the SAME
 *                          SuperLoop_Run() runs as the body of ONE FreeRTOS
 *                          task, and the scheduler is started instead of
 *                          spinning.
 *
 * ⚠️ THE LOOP BODY IS UNCHANGED. Every slot (now%10==0, ==5, now%100==9/43/47/
 * 51/53), the 20 ms control tick, the C6-1 bus-off check, the A4-1 failsafe and
 * the heartbeat are character-for-character what they were. The only body edits
 * are the tick source (SYS_TICK_MS below) and a 1 ms yield at the bottom.
 *
 * That is the whole point of B3: if anything misbehaves after this step it was
 * the RTOS, not a logic change - because there IS no logic change to blame.
 * Every improvement the reviews queued (A4-3, A4-4, A4-6, BUDGET-1, BUDGET-2,
 * the snapshot discipline) is its own later step, deliberately isolated.
 *--------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* tSuperLoop stack, in WORDS. Generous on purpose: this ONE task currently does
 * everything the firmware does - PID float maths, the battery SoC estimator,
 * DBC packing, UART formatting - so it is the deepest stack in the system until
 * B4 onward peel work out of it. It gets cut to fit once the high-water mark is
 * measured (budget 8.2 policy: max(128, roundup32(used * 1.5))). Starting tight
 * would just produce a stack overflow instead of a measurement. */
/* (SUPERLOOP_* retired at B11 - there is no super-loop under the RTOS.) */

/*----------------------------------------------------------------------------
 * B4 - tBattery, the FIRST task peeled out of the super-loop.
 *
 * It goes first because it is the safest thing to move, on three counts:
 *   - LEAST timing-critical: 100 ms period against a 20 ms control tick.
 *   - LOWEST priority, so it can never delay anything above it. If it is
 *     mid-Update when a higher task runs, it simply resumes.
 *   - NO Can_Transmit in the peeled work. Slot 43 was the I2C read + estimator
 *     ONLY; the 0x210 frame (slot 47) STAYS in tSuperLoop. Keeping every
 *     transmit in one task preserves the one-Can_Transmit-per-ms invariant
 *     untouched until the CAN TX queue arrives at B8.
 *
 * So the split is: tBattery WRITES the published snapshot (BatteryService_
 * Update), tSuperLoop READS it (BatteryService_GetStatus) when it packs 0x210
 * and when it prints the heartbeat. That boundary is the whole reason A3 built
 * this service around a single-snapshot getter - see battery_service.c's B4
 * note for why it needs a coherent commit and NOT a mutex.
 *
 * 100 ms via vTaskDelay, matching the old slot's 10 Hz exactly. The old slot
 * fired at now%100==43; the PHASE no longer matters, because phase only ever
 * existed to keep the I2C read out of the same millisecond as a TX - and a
 * separate task cannot share a millisecond with anything anyway.
 *--------------------------------------------------------------------------*/

/*============================================================================
 * ⚠️ STACK SIZES - TRIMMED 2026-08-06 FROM MEASUREMENT, NOT FROM GUESSWORK.
 *
 * Every number below is the budget's policy applied to the DEEPEST high-water
 * mark observed across one deliberate deep-path mission:
 *
 *     stack_words = max(128, roundup32(observed_peak x 1.5))
 *
 * THE MISSION (tools/mock_jetson.py mission, 75 s, wheels up): the ROS
 * on_activate sequence and its failsafe trip, an idle re-arm, 20 s driving at
 * 70 rpm with +0.20 rad steer, 15 s at 100 rpm with -0.25 rad, a COMMAND CUT
 * WHILE DRIVING (failsafe under load), a re-arm under load, and a wind-down -
 * during which the odometer crossed a 10 m boundary and completed an EEPROM
 * save (odo 84 m, saves=1, ring wrapped slot 7 -> 0). So PID, QEI, I2C, the
 * TX queue, the failsafe and the EEPROM writer were all exercised, several of
 * them overlapping.
 *
 *          task          was   observed peak   now
 *          tSafety       384        67         128
 *          tVelocity     512        66         128
 *          tRosRx        512        80         128
 *          tBattery      512       108         192   <- see below
 *          tCanTx        256        52         128   (in can_tx_queue.c)
 *          tBusHealth    256        31         192   <- see below
 *          tRosTx        512        92         160
 *          tClusterTx    512        98         160
 *          tOdo          256        38         128
 *          tHeartbeat    512        95         160
 *
 * ⚠️ tBattery WAS 160 FOR ONE BUILD, AND THE RE-VERIFICATION CAUGHT IT. The
 * first mission measured a 106-word peak (policy -> 160); repeating the mission
 * on the trimmed build measured 108 (policy -> 192), leaving only 1.48x rather
 * than the required 1.5x. The peak MOVES between runs - which is the entire
 * reason the policy carries margin, and the entire reason a trim must be
 * re-verified by repeating the mission rather than trusting one measurement.
 *
 * ⚠️ tBusHealth GETS 192, NOT THE 128 THE POLICY WOULD GIVE, and the exception
 * is the point: its 31-word measurement is the "no bus-off" path only. The bus
 * could not be forced off in this session (no privilege to down the interface),
 * so its DEEP branch - VelocityControl_Stop() + Can_RecoverBusOff() - is
 * UNMEASURED. tSafety makes the same Stop() call and measured 67 words, so the
 * deep branch is expected near that; 192 is ~2.9x it. Trimming an unmeasured
 * branch to a floor derived from the branch that was measured is exactly how a
 * stack overflow ships.
 *
 * ⚠️ ALSO UNMEASURED, system-wide: the configASSERT and stack-overflow hook
 * paths, which by design halt and so cannot be exercised in a passing run. The
 * mitigation is that configCHECK_FOR_STACK_OVERFLOW = 2 stays ON - it is
 * precisely the detector for a trim that turned out too tight.
 *
 * ⚠️ DO NOT trim further without repeating a mission of at least this depth.
 * Every earlier figure in this port was a lower bound taken from a shallow run.
 *==========================================================================*/

#define BATTERY_TASK_STACK_WORDS (192U)

/*  ⚠️⚠️ RAISED 1 -> 4 AT B6, AND THIS IS A CORRECTNESS FIX, NOT TUNING.
 *
 *  B4 put tBattery at the lowest priority on the reasoning that a fuel gauge
 *  must never delay anything. That was right until B6 added tCanTx at priority
 *  3, and then it broke the battery service outright:
 *
 *      B5 (no tCanTx):            flags=0 on every sample
 *      B6 with tBattery at 1:     flags=1 (BATTERY_FLAG_I2C_FAIL) on 5 of 12
 *      B6 with tBattery at 4:     flags=0 on every sample, and bt_wcet
 *                                 collapses from 1350 us back to 361 us
 *
 *  ROOT CAUSE: i2c.c bounds every command with a WALL-CLOCK deadline read from
 *  the free-running TIMER0 (I2C_ABORT_CAP_US = 200 us against a ~60 us nominal
 *  command). Wall-clock does not stop for a context switch. tCanTx wakes ~220
 *  times a second - once per transmitted frame - and each wake preempts a
 *  lower-priority task. Land two or three of those inside one I2C command and
 *  the 140 us of slack is gone, so the command "times out" while the bus is
 *  perfectly healthy.
 *
 *  WHY IT MATTERS RATHER THAN BEING COSMETIC: a failed read holds the last good
 *  sample AND does not advance the coulomb integrator. BATTERY_TICKS_PER_SECOND
 *  is 10 and the integral scales DIRECTLY with the successful tick rate, so a
 *  42 % failure rate does not make the gauge noisier - it makes SoC WRONG by
 *  that ratio, permanently and silently. Exactly the hazard battery_service_cfg.h
 *  warns about.
 *
 *  THE GENERAL RULE, worth carrying to every remaining peel: A DRIVER WITH A
 *  WALL-CLOCK TIMEOUT IS EFFECTIVELY A REAL-TIME CRITICAL SECTION. It cannot sit
 *  below anything that runs inside its timeout window. I2C is not "slow
 *  telemetry" just because the DATA is telemetry.
 *
 *  ✅ RESOLVED 2026-08-06 - THIS WAS THE "REVISIT AT B9" ITEM, and it is done.
 *  The stop-gap parked tBattery at 4, which is tVelocity's slot, and equal
 *  priorities TIME-SLICE. The answer was never "higher" - it was "in its own
 *  band": tBattery now sits at APP_PRIO_BATTERY (6), ABOVE tCanTx (5) so the
 *  wall-clock I2C wait is never torn, and BELOW the control tasks (7/8/9) so a
 *  480 us read can never delay a 20 ms control deadline. The full ladder and
 *  the reasoning for every level live in include/app_priorities.h.
 *
 *  Cost: tBattery can delay CAN drainage by one ~480 us Update per 100 ms -
 *  0.5 % of the time, absorbed by the 8-deep queue (peak observed: 0).
 */
#define BATTERY_TASK_PRIORITY    APP_PRIO_BATTERY
#define BATTERY_TASK_PERIOD_MS   (100U)   /* = the old slot's 10 Hz      */
/* Phase, in ticks mod 100 - the OLD SLOT RESIDUE. 43 mod 10 = 3, so it can
 * never land on the 0x110 (0 mod 10) or 0x130 (5 mod 10) transmit tick. */
#define BATTERY_TASK_PHASE_MS    (43U)

/*----------------------------------------------------------------------------
 * B5 - tOdo, the persistent-odometer EEPROM writer.
 *
 * WHAT MOVES: Odo_MainFunction() only - the non-blocking save state machine.
 * WHAT STAYS: Odo_AddMetres() (called from ClusterComm_UpdateTrip inside the
 * 0x200 TX path) and Odo_GetMetres() (packed into 0x200 odo_m) both remain in
 * tSuperLoop. Only the EEPROM SERVICING is peeled.
 *
 * ⚠️ THIS PEEL HAS A CONCRETE SAFETY PAYOFF, unlike B4's which was structural.
 * A TM4C EEPROM word program can take MILLISECONDS when the internal copy
 * buffer fills. Odo_MainFunction is written to never spin - it issues at most
 * one word and polls Eeprom_IsBusy() on later calls - but that discipline
 * existed precisely BECAUSE it sat inline in a 1 ms super-loop slot. As its own
 * lowest-priority task the millisecond-scale hardware latency becomes
 * structurally harmless: anything above it simply preempts. The write can no
 * longer stall control even if that no-spin discipline were ever broken.
 *
 * 100 ms period = the old slot's rate exactly (it fired at now%100==53, i.e.
 * 10 Hz). Phase is irrelevant now: slot 53 was chosen only to keep the EEPROM
 * work out of a millisecond that carried a CAN frame, and a separate task
 * cannot share a millisecond with anything.
 *
 * NO MUTEX, and that is a CHECKED fact, not an assumption - see the shared-state
 * analysis in the B5 block of docs/MEMORY.md. Two independent reasons:
 *   1. odo.c is the SOLE owner of the EEPROM (grepped: every Eeprom_Init /
 *      ReadWord / WriteWordStart / IsBusy call in the tree is in odo.c;
 *      main.c touches only the read-only Eeprom_GetErrorCount for its boot
 *      log). A sole-owner task IS the mutual exclusion.
 *   2. Every cross-task datum is a single aligned word - uint32 / enum / uint8
 *      - so loads and stores are atomic on Cortex-M4. There is no 24-byte
 *      struct here, which is exactly why B5 needs none of B4's deferred-commit
 *      machinery.
 *--------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * B7 - THE GLOBAL PHASE PLAN. Read this before adding ANY periodic task.
 *
 * BUDGET-6 was the lesson: converting a super-loop slot to a task silently
 * discarded its PHASE, two tasks ended up waking on the same tick as the 0x110
 * transmit, and 0x110 quietly fell to 90 Hz. The fix there was per-task. The
 * generalisation, which B7 needs because it adds two more periodic tasks, is
 * that PHASE IS A GLOBAL PROPERTY: it is not enough for each task to have "a"
 * phase, the set must be mutually non-colliding.
 *
 * Every periodic task is phase-locked to an ABSOLUTE tick residue, so the plan
 * holds forever rather than depending on boot timing:
 *
 *      task          period   residue        residue mod 5
 *      tRosTx           5 ms   0 mod 5             0
 *      tClusterTx      50 ms   2 mod 50            2
 *      tBattery       100 ms  43 mod 100           3
 *      tOdo           100 ms  53 mod 100           3
 *
 * WHY MOD 5 IS THE THING TO CHECK: tRosTx is the fastest producer at 5 ms, so
 * any task whose residue is 0 mod 5 will periodically wake on the SAME tick it
 * does. tClusterTx (2), tBattery (3) and tOdo (3) all avoid that.
 * tBattery and tOdo share residue 3 mod 5 but are 43 vs 53 mod 100, so they
 * never coincide with each other either.
 *
 * ⚠️ WHAT THIS BUYS, and why it is not merely tidy: two posts landing on one
 * tick would be serialised by tCanTx into two transmits ~222 us apart - which
 * is CORRECT (the queue guarantees one at a time) but breaks the "<=1
 * Can_Transmit per millisecond" property the owner asked to preserve. The phase
 * plan is what keeps the wire behaviour identical to the super-loop while the
 * SLOT RESIDUES THEMSELVES ARE RETIRED.
 *--------------------------------------------------------------------------*/
#define ODO_TASK_STACK_WORDS     (128U)
#define ODO_TASK_PRIORITY        APP_PRIO_ODO
#define ODO_TASK_PERIOD_MS       (100U)   /* = the old slot 53's 10 Hz   */
/* Same reasoning as BATTERY_TASK_PHASE_MS: 53 mod 10 = 3, off both TX ticks. */
#define ODO_TASK_PHASE_MS        (53U)

/*----------------------------------------------------------------------------
 * B7 - tRosTx (prio 3) and tClusterTx (prio 2): the telemetry producers.
 *
 * ⚠️ THE CADENCE IS THE ALTERNATION, NOT A "POST BOTH" BURST, and that choice
 * is what keeps the wire identical.
 *
 * tRosTx runs at 5 ms and alternates 0x110 / 0x130, so each goes out at 100 Hz
 * exactly 5 ms apart - reproducing the retired `now%10==0` / `now%10==5` slots
 * precisely, but from the task's OWN cadence rather than from residue
 * arithmetic in the super-loop. Posting both on one 10 ms wake would also give
 * 100 Hz each, but tCanTx would then serialise them ~222 us apart and break the
 * "<=1 transmit per ms" property.
 *
 * tClusterTx runs at 50 ms and alternates 0x200 / 0x210, so each goes out at
 * 10 Hz, 50 ms apart. (The retired slots were 38 ms apart at residues 9 and 47;
 * the rate is what matters and it is identical.)
 *
 * THE PING (0x7A1) folds into tClusterTx rather than getting a task of its own:
 * NodePing_MainFunction() no-ops cheaply when no request is pending, the old
 * slot ran it at 100 ms, and 50 ms is strictly better. Only the POST moved -
 * the latch-on-RX stays in JetsonComm_Poll where it was.
 *--------------------------------------------------------------------------*/
#define ROSTX_TASK_STACK_WORDS   (160U)   /* packs + the 8-sample pot read */
#define ROSTX_TASK_PRIORITY      APP_PRIO_ROS_TX
#define ROSTX_TASK_PERIOD_MS     (5U)     /* alternating => 100 Hz per frame */
#define ROSTX_TASK_PHASE_MS      (0U)     /* 0 mod 5 - see the phase plan    */

#define CLUSTERTX_TASK_STACK_WORDS (160U)
#define CLUSTERTX_TASK_PRIORITY    APP_PRIO_CLUSTER_TX
#define CLUSTERTX_TASK_PERIOD_MS   (50U)  /* alternating => 10 Hz per frame  */
#define CLUSTERTX_TASK_PHASE_MS    (2U)   /* 2 mod 50 - never 0 mod 5        */

/*----------------------------------------------------------------------------
 * B8 - tRosRx (prio 7): CAN RX routing.
 *
 * EVENT-DRIVEN, not periodic: it blocks on the ISR's "ring non-empty" edge and
 * drains the WHOLE ring per wake. So it has no phase and takes no part in the
 * phase plan - it runs when frames arrive, which is the host's cadence (30 Hz),
 * not ours. Zero CPU while the bus is quiet.
 *
 * PRIORITY 7 - above tSuperLoop (4), so an arriving command is routed BEFORE the
 * failsafe next re-checks and therefore re-arms promptly; below tVelocity (8),
 * because a hard QEI deadline must never wait on RX routing. See
 * include/app_priorities.h.
 *
 * ⚠️ tRosRx BECOMES THE SINGLE WRITER of the velocity/steering setpoints
 * (V9-R3's single-writer invariant). That is not a side effect - it is what the
 * RTOS design intends: one owner for SetSetpoint/SetAngle, and it is the task
 * that decodes the commands.
 *
 * The timeout on the wait is a liveness backstop only. A missed edge would
 * otherwise park this task forever with frames sitting in the ring; waking
 * anyway every 50 ms and draining costs nothing on an idle bus and cannot
 * silently wedge RX.
 *--------------------------------------------------------------------------*/
#define ROSRX_TASK_STACK_WORDS   (128U)
#define ROSRX_TASK_PRIORITY      APP_PRIO_ROS_RX
#define ROSRX_WAIT_TIMEOUT_MS    (50U)

/*----------------------------------------------------------------------------
 * B9 - tVelocity (prio 8): the closed-loop velocity control task.
 *
 * ⚠️⚠️ THE ONE HAZARD OF THIS WHOLE PORT. Read this before touching the timing.
 *
 * QEISPEED - the QEI hardware velocity sample the PID reads - refreshes ONCE
 * PER 20 ms WINDOW. Calling VelocityControl_Update() sooner than 20 ms after
 * the previous call re-reads the SAME STALE SAMPLE: a fresh setpoint paired
 * with an unchanged measurement, so the PID integrates an error that has
 * already been acted on. The loop then chatters REGARDLESS OF GAIN TUNING.
 * (qei_cfg.h, QEI_WINDOW_AUDIT.md.)
 *
 * ⚠️ SO THE DEADLINE IS VIOLATED BY FIRING **EARLY**, NOT LATE - the opposite
 * of every other deadline in this system. Being late is harmless (polling
 * slower than the window just wastes a window); being early breaks control.
 *
 * ⚠️ AND xTaskDelayUntil ALONE DOES NOT PREVENT IT. It is a FIXED-INCREMENT
 * scheme: it computes the next wake from the PREVIOUS wake, so if this task
 * ever overruns its slot (tSafety at prio 9 is the only thing that can cause
 * that once it is peeled at B10), xTaskDelayUntil RETURNS IMMEDIATELY to catch
 * up - and the next Update lands < 20 ms after the last one. That is precisely
 * the failure the super-loop's `lastVelMs = now` re-referencing was written to
 * avoid, and precisely why the plan (section 1.2) called this the most likely way
 * the port silently degrades control quality.
 *
 * THE FIX IS BOTH, BELT AND BRACES:
 *   xTaskDelayUntil  - gives the drift-free 20 ms cadence in the normal case;
 *   an explicit guard - `(now - lastUpdateTick) >= 20` before every Update,
 *                       re-referencing lastUpdateTick to `now` on each Update.
 * That guard IS `lastVelMs = now`, transliterated. Update can then never run
 * less than 20 ms after the previous one, whatever the scheduler does. If a
 * catch-up wake arrives early, the cycle is SKIPPED (counted, see vt_skipped)
 * rather than fed a stale sample - skipping is the safe direction here.
 *
 * PHASE 7 mod 20, per the global phase plan. tVelocity is above every producer,
 * so what matters is what IT preempts: a plain 20 ms period would be 0 mod 20 =
 * 0 mod 5, i.e. tRosTx's tick, every single time. 7 mod 20 is 2 mod 5 and
 * collides with none of tRosTx (0 mod 5), tClusterTx (2 mod 50), tBattery (43
 * mod 100) or tOdo (53 mod 100).
 *
 * ⚠️ NOT IN B9: the V9-R4 inhibit flag. tSafety's Stop() racing a resuming
 * Update() is a B10 problem and gets built there, with tSafety.
 *--------------------------------------------------------------------------*/
#define VELOCITY_TASK_STACK_WORDS (128U)
#define VELOCITY_TASK_PRIORITY    APP_PRIO_VELOCITY
#define VELOCITY_TASK_PHASE_MS    (7U)     /* 7 mod 20 - see the phase plan */

/*----------------------------------------------------------------------------
 * B10 - tSafety (prio 9, HIGHEST): the A4-1 RX command-loss failsafe.
 *
 * The last control peel, and the most safety-critical. This logic has lived
 * continuously inside the super-loop since it was written and hardware-proven;
 * B10 moves it, VERBATIM, into the one task nothing can preempt.
 *
 * ⚠️ NOTHING IS ABOVE IT, BY DEFINITION. A failsafe that can be starved by the
 * thing it polices is not a failsafe (plan section 2.2). Its work is two counter
 * loads, a compare, and rarely a Stop() - so putting it on top costs nothing,
 * and it holds no lock, so it cannot invert anything.
 *
 * ⚠️ WHY IT IS A DEDICATED PERIODIC TASK and not a check inside tRosRx: a
 * partially-crashed host spraying MALFORMED frames would keep tRosRx's queue
 * non-empty, so a blocking receive would never time out - while the ACCEPTED-
 * command counters never advanced. The failsafe would be silently defeated by
 * exactly the failure mode it exists for. A periodic task's liveness does not
 * depend on the bus being alive, which is the whole point.
 *
 * PERIOD 10 ms against CMD_TIMEOUT_MS = 500: detection granularity is 2 % of
 * the deadline, i.e. negligible. (Better than the super-loop's 100 ms check
 * against the same deadline, which was 20 %.)
 *
 * ⚠️ tSafety IS THE FIRST TASK THAT CAN PREEMPT tVelocity. That makes two
 * earlier pieces of work load-bearing exactly here:
 *   - V9-R4's inhibit latch (velocity_control.c) - without it, a tVelocity
 *     resuming after Stop() would overwrite the stop with a fresh PID output;
 *   - B9's >= 20 ms guard - tSafety preempting tVelocity is precisely what
 *     makes xTaskDelayUntil return early to catch up, which the guard refuses.
 *
 * PHASE 3 mod 10: 3 mod 5 = 3, so it never shares a tick with tRosTx (0 mod 5)
 * or tClusterTx (2 mod 50); and 3 mod 20 != 7, so it never coincides with
 * tVelocity either - the task it would otherwise most often preempt.
 *--------------------------------------------------------------------------*/
#define SAFETY_TASK_STACK_WORDS  (128U)
#define SAFETY_TASK_PRIORITY     APP_PRIO_SAFETY
#define SAFETY_TASK_PERIOD_MS    (10U)
#define SAFETY_TASK_PHASE_MS     (3U)     /* 3 mod 10 - see the phase plan */


/*----------------------------------------------------------------------------
 * B11 - tBusHealth and tHeartbeat: the last two residents of the super-loop.
 *
 * PHASES, checked against the global phase plan so neither shares a tick with
 * an existing task:
 *   tBusHealth  71 mod 100 -> 71%5=1 (not tRosTx 0), 71%50=21 (not tClusterTx 2),
 *               71%20=11 (not tVelocity 7), 71%10=1 (not tSafety 3)
 *   tHeartbeat 137 mod 1000 -> 137%5=2, %50=37, %20=17, %10=7, %100=37
 *               - clear of all five.
 *--------------------------------------------------------------------------*/
#define BUSHEALTH_TASK_STACK_WORDS (192U)
#define BUSHEALTH_TASK_PRIORITY    APP_PRIO_BUS_HEALTH
#define BUSHEALTH_TASK_PERIOD_MS   CAN_HEALTH_PERIOD_MS   /* 100 ms, unchanged */
#define BUSHEALTH_TASK_PHASE_MS    (71U)

/* The longest body in the system (80-odd UART calls) but the lowest priority,
 * so its length cannot matter. 512 words: UART_SendFloat's formatting is the
 * deepest thing it does. */
#define HEARTBEAT_TASK_STACK_WORDS (160U)
#define HEARTBEAT_TASK_PRIORITY    APP_PRIO_HEARTBEAT
#define HEARTBEAT_TASK_PERIOD_MS   HEARTBEAT_MS           /* 1000 ms, unchanged */
#define HEARTBEAT_TASK_PHASE_MS    (137U)

#endif /* USE_FREERTOS */

#endif /* APP_CFG_H_ */
