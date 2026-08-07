/******************************************************************************
 * app_priorities.h - THE priority ladder. One file, one authority.
 *
 * Settled 2026-08-06, resolving the BUDGET-5 clash. Before this the ladder was
 * scattered - `SUPERLOOP_PRIORITY` and `BATTERY_TASK_PRIORITY` in main.c,
 * `CANTX_TASK_PRIORITY` in can_tx_queue.c - so no single place stated the
 * ordering and no reviewer could check it. That is precisely how BUDGET-5's
 * inversion (a wall-clock I2C wait sitting BELOW the CAN drainer) got in.
 *
 * ⚠️ THE ONE RULE: every pair of tasks that can interfere gets a DISTINCT
 * level. Equal priorities TIME-SLICE under `configUSE_TIME_SLICING`, and a
 * time-slice between two tasks that share a deadline or a bus is the same
 * hazard as a mis-ordered pair. `tBattery` was parked at 4 as a stop-gap - which
 * is `tVelocity`'s slot - and that is what this file exists to prevent
 * recurring.
 *
 * ===========================================================================
 *  THE LADDER (high number = more urgent; 0 is the idle task)
 * ===========================================================================
 *
 *  11  tSafety      SAFETY       RX command-loss failsafe
 *  10  tVelocity    CONTROL      20 ms PID against the QEI window
 *   9  tRosRx       COMMAND      route 0x100/0x120/0x140 to control
 *   8  tBattery     SENSING      INA226 read (I2C0) + SoC estimator
 *   7  tImu         SENSING      MPU6050 read (I2C1) + 0x150/0x160 -> queue
 *   6  tCanTx       TRANSPORT    drains the CAN TX queue
 *   5  tBusHealth   SAFETY-ish   C6-1 bus-off observe + recover
 *   4  tRosTx       TELEMETRY    pack 0x110/0x130 -> queue
 *   3  tClusterTx   TELEMETRY    pack 0x200/0x210 -> queue
 *   2  tOdo         PERSISTENCE  EEPROM save state machine
 *   1  tHeartbeat   DIAGNOSTICS  the 1 Hz console line
 *   0  idle         -
 *
 * ⚠️ RENUMBERED AT B11, +1 across the board above tOdo, to open level 1 for
 * tHeartbeat and level 5 for tBusHealth. EVERY RELATIVE ORDER IS PRESERVED -
 * this is a shift, not a re-rank. tBusHealth deliberately inherits the level
 * tSuperLoop held (it was the code that ran there), so the C6-1 check keeps
 * exactly the position it has had since B3.
 *
 * ⚠️ RENUMBERED AGAIN AT B13, +1 across the board ABOVE tCanTx, to open level 7
 * for tImu. Again a SHIFT, NOT A RE-RANK: tSafety/tVelocity/tRosRx/tBattery all
 * moved up one, tCanTx and everything below kept their numbers, and every
 * relative order in the system is unchanged. tImu had to go between tBattery
 * and tCanTx (see its entry below) and there was no free level there, which is
 * the only reason anything moved at all. configMAX_PRIORITIES 11 -> 12.
 *
 * ---------------------------------------------------------------------------
 * WHY EACH LEVEL SITS WHERE IT DOES
 *
 * 11 tSafety - NOTHING above it, by definition. A failsafe that can be starved
 *    by the thing it polices is not a failsafe (plan section 2.2). Its work is a
 *    two-word read and a compare, so putting it on top costs essentially
 *    nothing and it holds no lock, so it cannot invert anything.
 *
 * 10 tVelocity - the tightest deadline in the system: the QEI velocity register
 *    refreshes once per 20 ms window, and the loop is violated by firing EARLY
 *    as well as late. Only the failsafe may delay it.
 *
 *  9 tRosRx - BELOW tVelocity, and that is a deliberate call rather than an
 *    oversight. Both were at 4 in the original design table. RX frames are
 *    buffered (the ring buffer today, a queue from B8) and the host sends at
 *    30 Hz, so a few hundred microseconds of routing delay costs nothing
 *    measurable. The QEI window is a HARD hardware deadline that cannot be
 *    buffered. When a soft-deadline task and a hard-deadline task compete, the
 *    hard one wins.
 *
 *  8 tBattery - ⚠️ THE BUDGET-5 CONSTRAINT, and the whole reason this file
 *    exists. i2c.c bounds every command with a WALL-CLOCK deadline read from
 *    TIMER0 (200 us cap against a ~60 us nominal command). Wall-clock does not
 *    stop for a context switch, so any task that runs INSIDE that window eats
 *    the margin. With tBattery below tCanTx, tCanTx's ~220 wakes/second caused
 *    BATTERY_FLAG_I2C_FAIL on 5 of 12 reads - and because a failed read does not
 *    advance the coulomb integrator, SoC would have been wrong by 42 %,
 *    silently.
 *      => tBattery MUST stay ABOVE tCanTx.
 *      => but it need NOT be at control's level, and must not be: a 480 us I2C
 *         read has no business delaying a 20 ms control deadline.
 *    Placing it BETWEEN control and transport satisfies both. That is the key
 *    move this step makes - the stop-gap bumped it to 4 and collided with
 *    tVelocity; the answer was never "higher", it was "in its own band".
 *
 *    ⚠️ THE GENERAL RULE, worth carrying to every future driver: A DRIVER WITH A
 *    WALL-CLOCK TIMEOUT IS EFFECTIVELY A REAL-TIME CRITICAL SECTION. I2C is not
 *    "slow telemetry" just because the DATA it carries is telemetry. The same
 *    reasoning now sizes the ADC cap (BUDGET-1).
 *
 *  7 tImu - B13. The SECOND wall-clock-bounded I2C reader, and it is placed by
 *    the same rule as tBattery, applied twice.
 *
 *    ⚠️ FIRST, THE PREMISE THE B13 PROMPT GOT WRONG: the MPU6050 does NOT share
 *    a bus with the INA226. The IMU is on I2C1 (PA6/PA7), the INA226 on I2C0
 *    (PB2/PB3) - see imu_service_cfg.h and ina226_cfg.h, and note that the
 *    single no-arg I2C_Init() brings up BOTH. So there is no bus arbitration
 *    question here at all, and no shared-transaction hazard. What remains is
 *    purely the BUDGET-5 CPU effect, which is enough on its own.
 *
 *    ABOVE tCanTx, non-negotiably: i2c.c caps every command on a WALL-CLOCK
 *    TIMER0 deadline (200 us against a ~60 us nominal), and a 14-byte burst read
 *    is 15 such capped commands back to back. tCanTx now wakes ~320 times a
 *    second. Below it, this task would be torn exactly the way tBattery was, and
 *    the general rule from that finding applies unchanged: A DRIVER WITH A
 *    WALL-CLOCK TIMEOUT IS EFFECTIVELY A REAL-TIME CRITICAL SECTION.
 *
 *    BELOW tBattery, and that ordering is deliberate rather than arbitrary -
 *    one of the two had to be able to preempt the other, so the question is
 *    which failure is worse. They are not symmetric:
 *      - tBattery torn  -> a failed read does not advance the coulomb
 *        integrator, so SoC drifts WRONG in proportion to the failure rate,
 *        permanently and SILENTLY. (Measured at 42 % when this was got wrong.)
 *      - tImu torn      -> one sample is dropped. The service holds the last
 *        good sample and the sequence simply does not advance, which is visible
 *        in any capture. (⚠️ VISIBLE, not yet ACTED ON: can_comms.cpp stores
 *        `expected_seq_` and never reads it, so the host does not currently
 *        reject repeats - a ROS-side follow-up. The point stands either way:
 *        the failure leaves a trace, where the battery's does not.)
 *    A silent cumulative error outranks a loud transient one, so the fuel gauge
 *    wins. Cost: up to one dropped IMU sample per 100 ms worst case, 2 % of the
 *    stream - and in practice zero, because the phase plan (app_cfg.h) gives
 *    tBattery residue 43 mod 100 and tImu residue 8 mod 20, so they never wake
 *    in the same millisecond and neither read is ~1 ms long.
 *
 *  6 tCanTx - above the telemetry PRODUCERS so a posted frame drains promptly
 *    and the queue stays shallow (observed peak depth: 0), below tBattery per
 *    the constraint above, and below control so draining telemetry can never
 *    delay a control update or the failsafe.
 *
 *  5 tBusHealth - the C6-1 bus-off check: what was LEFT of tSuperLoop at B11.
 *    It deliberately INHERITS tSuperLoop's level, so the check keeps exactly the
 *    position it has held since B3 - below control/battery/transport, above the
 *    telemetry producers. B11 is a structural step and must not quietly re-rank
 *    a path that commands the vehicle to stop.
 *    It gets its OWN task rather than folding into tHeartbeat because the two
 *    answer different questions - "is the bus alive?" vs "what are the numbers?"
 *    - and only one of them stops the vehicle.
 *
 *  1 tHeartbeat - pure diagnostics, LOWEST, and it is also by far the longest
 *    body in the system (80-odd UART calls). Lowest precisely because of that: a
 *    late heartbeat is harmless and it must never delay anything real.
 *    ⚠️ Every value it prints is a cross-task read of a plain uint32/boolean -
 *    a single aligned word, atomic on M4 - which is why it needs no guard. That
 *    was not luck: each peel declared its observability that way on purpose.
 *
 *  4 tRosTx / 3 tClusterTx - telemetry producers. Distinct levels even though
 *    they could safely share one: neither has a deadline that the other can
 *    violate, but the cost of a spare level is ~20 bytes of ready list and the
 *    benefit is that "no two tasks time-slice" stays a rule with no exceptions
 *    to remember. 0x110/0x130 at 100 Hz outrank 0x200/0x210 at 10 Hz because a
 *    late feedback frame reaches a control loop, while a late cluster frame
 *    reaches a display.
 *
 *  2 tOdo - near the bottom. An EEPROM word program can take MILLISECONDS when the copy
 *    buffer fills; at the bottom of the ladder that latency can only ever delay
 *    tOdo itself.
 *
 * ---------------------------------------------------------------------------
 * PRIORITY INVERSION: structurally absent, and it is worth saying why rather
 * than assuming it. Classical unbounded inversion needs a LOCK held by a low
 * task that a high task wants. This system has no such lock:
 *   - battery status uses a deferred single commit inside taskENTER_CRITICAL -
 *     bounded, cannot be held across a context switch, cannot block a task;
 *   - odo needs nothing at all (sole EEPROM owner, single-word cross-task data);
 *   - the CAN TX-complete semaphore is a SIGNAL given by an ISR, not a lock -
 *     an ISR owns nothing and can inherit nothing;
 *   - the TX queue is posted to with a ZERO timeout, so a producer never waits.
 * Nothing holds anything. Re-check this list whenever a mutex is added.
 *****************************************************************************/

#ifndef APP_PRIORITIES_H_
#define APP_PRIORITIES_H_

/* ⚠️ configMAX_PRIORITIES (FreeRTOSConfig.h) must be > the highest value here.
 * It is 12, giving usable levels 0..11. */
#define APP_PRIO_IDLE            (0U)
#define APP_PRIO_HEARTBEAT       (1U)   /* B11 - diagnostics only, lowest   */
#define APP_PRIO_ODO             (2U)
#define APP_PRIO_CLUSTER_TX      (3U)
#define APP_PRIO_ROS_TX          (4U)
#define APP_PRIO_BUS_HEALTH      (5U)   /* B11 - took tSuperLoop's old slot */
#define APP_PRIO_CAN_TX          (6U)
#define APP_PRIO_IMU             (7U)   /* B13 - I2C1, above the TX drainer */
#define APP_PRIO_BATTERY         (8U)
#define APP_PRIO_ROS_RX          (9U)
#define APP_PRIO_VELOCITY        (10U)
#define APP_PRIO_SAFETY          (11U)

#endif /* APP_PRIORITIES_H_ */
