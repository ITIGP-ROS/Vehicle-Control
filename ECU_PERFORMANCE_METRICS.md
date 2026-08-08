# ECU Performance Metrics — TM4C123 Ackermann Robot Controller

**Target:** TI TM4C123GH6PM · ARM Cortex-M4F @ **16 MHz** · 32 KB SRAM · 256 KB Flash
**RTOS:** FreeRTOS v11.1.0 (MIT), GCC/ARM_CM4F port, **preemptive**, **static allocation only**
**Scheduling:** fixed-priority preemptive, **11 tasks** + idle · 1 kHz tick
**Last measured:** 2026-08-08 (I2C1 bus-recovery fix; B13/B13b/B13c before it) · tags: **[M] = measured on hardware**, **[E] = estimated / worst-case bound**

> This is the standalone ECU analysis: WCET, CPU load, schedulability, jitter, interrupt latency, and
> memory budget. Method is stated per metric so every number is reproducible. This is the clean
> summary; the longer working analysis it was distilled from lives in the development repository and
> is deliberately not copied here (this repo is production firmware only).

---

## 1. Headline numbers

| Metric | Value | Verdict |
|---|---|---|
| **Total CPU utilization `U`** | **≈ 8.6 %** (incl. 2.0 % kernel tick) | 🟢 ~91 % headroom |
| **Schedulability (RM bound, n=11)** | `U` 8.6 % vs bound **71.4 %** | 🟢 passes ~8× |
| **Schedulability (RTA, exact, `tVelocity`)** | worst-case response **≈ 95 µs** vs a **20,000 µs** deadline | 🟢 ~210× margin |
| **Worst-case interrupt latency** | **≈ 4 µs** (masked) / **0.75 µs** (above syscall priority) | 🟢 |
| **RAM usage** | **37.8 %** of 32 KB (B13 tImu adds ~0.9 KB) | 🟢 |
| **Flash usage** | ~59 KB of 256 KB (kernel adds ~11 KB) | 🟢 |
| **Memory model** | static allocation (MISRA-C:2012 Dir 4.12 / AUTOSAR "no dynamic memory after init") | 🟢 |

---

## 2. Per-task timing (WCET & utilization)

`Ti` = period / min inter-arrival · `Ci` = worst-case execution time · `Ui = Ci/Ti`

| task | prio | `Ti` | `Ci` (WCET) | tag | `Ui` | notes |
|---|---|---|---|---|---|---|
| `tSafety` | 11 | 10 ms | **2 µs** steady | [E] | 0.01 % | command-loss failsafe: 2 counter loads + compare + tick read. Trip path non-recurrent |
| `tVelocity` | 10 | **20 ms** (hard, QEI) | **60 µs** | [E] | 0.30 % | 2× QEI read + 2× PID (Tustin + deriv filter + anti-windup) + 2× Motor_SetSpeed. **Measured interval 20/20 ms, 0 early fires [M]** |
| `tRosRx` | 9 | 33.3 ms, burst 2 | 20 µs/frame | [E] | 0.12 % | ISR-semaphore woken; routes steering+velocity per 30 Hz cycle |
| `tBattery` | 8 | 100 ms | **479.9 µs** | **[M]** | 0.48 % | **MEASURED** — Ina226_ReadAll ~312 µs + ~168 µs SoC estimator |
| `tImu` | **7** | 20 ms | **927 µs** | **[M]** | **4.64 %** | ✅ **B13** — MPU6050 14-byte burst (15 capped I2C commands) + 2 packs + 2 posts. **Measured 847–927 µs; the 80 µs min–max spread proves this is execution, not interference.** 🔴 6.2× the 150 µs estimate. ⚠️ On a bus-recovery cycle (2026-08-08 fix) this rises to **~2.65 ms** — 13 % of the 20 ms slot, verified not to disturb the CAN queue |
| `tCanTx` | 6 | event, 4.5 ms avg | 10 µs CPU | [E] | 0.22 % | pop + Can_Transmit load. The **222 µs on the wire is blocked time, not CPU** |
| `tRosTx` | 4 | 10 ms (5 ms alt) | ≤ ~1.6 ms | [E] | — | 0x110/0x130; the 0x130 path does 8 ADC conversions, TIMER0-bounded at 200 µs each (see §5) |
| `tClusterTx` | 3 | 100 ms | 30 µs | [E] | 0.03 % | 0x200 + 0x210 pack + posts |
| `tOdo` | 2 | 100 ms | 20 µs | [E] | 0.02 % | ≤1 EEPROM word-write issue/call; never blocks on EEDONE (polls next call) |
| `tBusHealth` | 5 | periodic | small | [E] | — | C6-1 bus-off observe + recovery retry |
| `tHeartbeat` | 1 | 1000 ms | — | [E] | negligible | UART debug print (longest body, lowest priority) |
| **CAN0 ISR** | NVIC | 281 IRQ/s | 15 µs | [E] | 0.42 % | IF read + ring push / TX-done semaphore give |
| **kernel tick** | — | 1 kHz | — | [M] | **2.0 %** | measured; 42 % of total U — the cost of a 16 MHz core |

**Key measured building blocks [M]:**

| operation | measured | method |
|---|---|---|
| `BatteryService_Update()` | **479.9 µs** | timer0 / DWT, worst branch |
| `Ina226_ReadAll()` | min 307.8 / mean 311.8 / max 352.7 µs (200 calls, 0 err) | timer0 |
| I2C largest single capped wait | ~60 µs (cap 200 µs → 3.3× headroom) | timer0 |
| I2C command | read 120.4 / 149.2 µs, write 120.2 µs; 1 wire byte = 28.8 µs ⇒ **400 kHz confirmed** | timer0 |
| CAN frame on the wire | ~222 µs (8-byte std @ 500 kbps); **~7.1 % bus load** at **320 fr/s** (was 4.9 % / 221 fr/s before B13 added 0x150+0x160) | datasheet + candump |
| `tVelocity` Update interval | **20/20 ms min/max, 0 early fires** | on-target instrumentation |

⚠️ **WCET is worst-case EXECUTION, not response time.** Early attempts measured with a wall-clock
(timer0) captured *response* time (execution + preemption) — corrected to measure execution only
(critical-section-bracketed or lowest-preemption context), per the B4 correction.

---

## 3. CPU utilization & schedulability

**Total:** `U = Σ Ci/Ti ≈ 0.0864 ≈ 8.6 %` → headroom `1 − U ≈ 91.4 %`.
⚠️ Was 4.9 % before B13. The jump is almost entirely `tImu`'s **measured** 927 µs against an
estimated 150 µs — a bad estimate corrected by
measurement, not a regression in anything that already existed.

- **Rate-Monotonic sufficient test:** for n=11, the RM bound is `n(2^(1/n) − 1) ≈ 71.4 %`. `U = 8.6 %` is
  ~8× under it → sufficient condition satisfied. (Note: our priority order is by **criticality**, not
  strictly rate-monotonic — `tSafety` outranks faster tasks — so RM is indicative; RTA is the exact test.)
- **Response-Time Analysis (exact) for the hard deadline:** `R(tVelocity) ≈ 95 µs` against its **20 ms**
  QEI deadline → **~210× margin**. (Improved from 135 µs when the priority ladder was settled — moving
  `tRosRx` below `tVelocity` removed an interferer.)
- **No priority inversion possible:** no task holds a lock across a context switch. Cross-task data uses
  coherent single-commit snapshots (battery, steering, wheel-ticks) or atomic single-word reads
  (odometer, command counters) — the RTOS-readiness discipline applied throughout the service layer.

---

## 4. Jitter & interrupt latency

**Release jitter** = variation between a task's nominal release instant and its actual start.

| task | jitter bound | why |
|---|---|---|
| `tVelocity` | ≤ 1 tick (1 ms), **non-accumulating** | highest-but-safety priority + `xTaskDelayUntil` (fixed-increment, drift-free) + the ≥20 ms guard converting residual jitter into a *skipped* update, never an early one |
| others | ≤ 1 tick | quantised to the 1 kHz tick |

All jitter is quantised to the **1 ms tick** (a 20 ms period = exactly 20 ticks) — this is by design, not
a defect, so it isn't mistaken for one.

**Interrupt latency:**

| case | latency | note |
|---|---|---|
| worst case, IRQ below syscall priority | **≈ 4 µs** | masked window (`configMAX_SYSCALL_INTERRUPT_PRIORITY`) |
| worst case, IRQ above syscall priority | **0.75 µs** | never masked by the kernel — available for control-critical IRQs |

---

## 5. Bounded execution — no unbounded waits (hard-real-time hygiene)

Every busy-wait in the system is **time-bounded**, never iteration-count-bounded (which is
compiler/-O-dependent) and never unbounded:
- **I2C** waits are TIMER0-tick-capped (200 µs cap, ~3.3× a healthy 60 µs command). A command that
  reports `BUS_STUCK` now runs the TIMER0-timed SCL-toggle recovery (≤9 pulses) and is retried once —
  see §8. Recovery is per-`i2cId` and touches only that module's registers and its own two pins, so
  **I2C0/INA226 is unaffected** (the two devices share the mechanism, not a bus).
- **ADC** (`Adc_ReadRaw`) was a 100,000-iteration spin (~500 ms WCET on a dead ADC) → **fixed to a
  TIMER0 time bound at 200 µs**, dropping `ServoFb_ReadRawFiltered` (×8) from ~500 ms to ~1.6 ms.
- **CAN TX** completion is signalled by an **ISR semaphore** (no `TX_BUSY` spin).
- **EEPROM** writes are non-blocking (one word issued per call, status polled on the *next* call).
- A **not-running TIMER0** is detected (`Timer0_IsRunning()`) so a stopped timebase can't turn a bounded
  wait unbounded.

---

## 6. Memory budget (static allocation)

| resource | usage | ceiling | margin |
|---|---|---|---|
| **SRAM** | **37.8 %** (post-trim + B13's tImu: 768 B stack + TCB + one ready list) | 32 KB | 🟢 62.2 % free |
| **Flash** | ~59 KB (kernel ~11 KB) | 256 KB | 🟢 ~77 % free |

- **Static allocation** (`configSUPPORT_STATIC_ALLOCATION=1`, dynamic off): every task/queue/semaphore
  buffer is fixed at link time → insufficient RAM is a **linker error on the bench**, never a runtime
  allocation failure in the field.
- **Stack sizing:** high-water marks (`uxTaskGetStackHighWaterMark`) taken after a **deep-path mission**
  (streaming commands + wheels turning + failsafe + EEPROM save overlapping), then the **margin policy**
  (≥50 % over observed worst) applied. All tasks ≥ **1.60× margin**; **zero** stack-overflow-hook fires
  across 3 verification missions (`configCHECK_FOR_STACK_OVERFLOW=2` on).
- ⚠️ **`tBusHealth`** is deliberately left at 6.19× margin: its bus-off deep branch could not be forced,
  so its worst case is genuinely unmeasured — over-provisioned rather than trimmed on incomplete data.

---

## 7. Measurement methodology (so every number is reproducible)

| metric | how measured |
|---|---|
| **WCET** | Cortex-M4 DWT `CYCCNT` or free-running TIMER0 around the task body, ÷16 MHz → µs, **worst-case input named** (e.g. INA226 read with a slow-but-not-failed bus; PID with saturation + anti-windup active). Bracketed to exclude preemption (execution, not response). |
| **Stack** | `uxTaskGetStackHighWaterMark` per task, sampled after ≥ several iterations (a single-shot read under-reports — observed 245→217 words on tHeartbeat), taken after a deep-path mission, + ≥50 % margin. |
| **CPU load** | Σ Ci/Ti from the measured/estimated table; kernel tick measured directly (2.0 %). |
| **Jitter** | task release vs actual start over N cycles; `tVelocity` interval instrumented on-target (20/20 ms). |
| **Interrupt latency** | NVIC priority analysis + the kernel's syscall-masking window. |
| **Bus load** | candump frame count × 222 µs/frame ÷ wall-clock. |

---

## 8. What is NOT yet measured (recorded honestly, not claimed)

- ✅ **`tImu` I2C1 ROBUSTNESS — DIAGNOSED AND FIXED (2026-08-08).** ⚠️ **This entry previously said
  "AN OPEN HARDWARE-LEVEL FAULT, NOT A TIMING GAP", declared the fault ISOLATED TO HARDWARE, and
  warned "Do not add an I2C bus-recovery routine on this evidence." That conclusion was WRONG and is
  corrected here, because it steered a reader away from what turned out to be the actual fix.**
  **What was really happening**, measured on hardware with per-command instrumentation and GPIODATA
  pad-level reads (not inferred):
  - The initiating fault is `I2C_ERROR_BUS_STUCK` **mid-burst** (receive byte 6 of 14) with **SDA
    measured LOW at the pad** — the MPU6050 stranded holding the line. Exactly the stranded-slave
    condition the old text said was "excluded".
  - `I2C_Reset()` re-inits the **master** and cannot touch a **slave** holding the line, so the wedge
    survived it — and survived an MCU reset *and* a reflash (first call of the next boot faulted).
  - 🔴 **The permanence was 100 % SOFTWARE.** The SCL-toggle recovery (`I2C_RecoverBegin/Step/End`)
    already existed, was correct, and was called by **nobody** in the production image — `i2c.h`
    documents the contract ("HAL drives them ONLY on `I2C_ERROR_BUS_STUCK`") but no HAL implemented
    it. Worse, once `MPU6050_ReadRaw` marked the device uninitialised the only `I2C_Reset` call site
    became unreachable, so the wedge was permanent *by construction*.
  - ⚠️ **Why the old "power-cycle excludes a stranded slave" inference failed:** it generalised from a
    single episode that was genuinely also physical (that one was cured by reseating the I2C1 leads).
    The reproducible wedge is a stranded slave, and **4–7 SCL pulses free it** — measured three times.
  **The fix (2026-08-08)** wires the existing recovery into the `BUS_STUCK` path per the `i2c.h`
  contract, gives the fault path a route back to the bus, and makes phase-1 `TIMEOUT` assert the STOP
  the header always promised (a burst abandoned mid-flight is what strands the slave).
  **Verified:** 7 min under a 30 Hz host stream **+ real motion** → **22 `BUS_STUCK` events, every one
  auto-cleared, 64 recoveries needing 1–4 SCL pulses, 21,017 good reads after the first fault, ending
  HEALTHY with no MCU reset** — the condition that previously wedged permanently within ~5 s.
  **Cost:** the recovery primitive is ~185 µs, but the whole fault path (200 µs cap + 200 µs abort +
  a full 14-byte retry) takes tImu's worst cycle from **927 µs → ~2.65 ms**. That is 13 % of tImu's
  own 20 ms slot; through 64 recoveries the CAN queue held `peak=2 qfull=0 txfail=0 txtmo=0`, all six
  frames stayed nominal and `vel_ivl=20/20ms skip=0`. **The healthy path is unchanged.**
  **Observability:** the heartbeat now prints **`irec=N/Mp`** — recoveries since boot / SCL pulses the
  last one needed. (New field; distinct from `can_recov`, which counts CAN bus-health recoveries.)
  **Containment still holds and is still worth knowing**, for the case where the sensor is genuinely
  absent: the `ImuService_HasSample()` gate publishes **nothing** rather than zeroes
  (`imu=NOSAMPLE/seq0`, 0 × 0x150/0x160 on the wire, verified), the task collapses to **9 µs**/cycle,
  and the other frames stay at their nominal rates.
  ✅ The B13b axis-sign check is **DONE** (all six motions PASS, CCW-yaw → +gz through the production
  TX path), and the **180° mount yaw** it surfaced is **corrected in firmware (B13c)**:
  `ImuService_Latch` negates accel X/Y + gyro X/Y, gyro Z untouched.
- 🟡 **The TRIGGER of that bus fault is NOT isolated — deliberately, and the fix does not depend on
  it.** It fires with **zero actuation** (wedged at 13 s and again at 49 s with rpm=0, steer=0), yet a
  **60 Hz** command stream produced **0 faults in 120 s** (~6000 cycles). Rate and jitter profile both
  differed between those runs and which one matters is unproven, so it is not guessed at. The design
  response is **survive the event, not prevent it** — hence recovery-on-occurrence. **Watch `irec`:**
  N rising slowly with `imu=H` is the system working as intended; N rising fast, or the pulse count
  drifting upward, means the electrical trigger is worsening and wants a physical check of the
  PA6/PA7 leads, pull-ups and module.
- 🟠 **Per-axis accelerometer scale** — `IMU_ACCEL_SCALE_CORR` is one scalar (1.1054) for all three
  axes, but static holds show Z ~9 % low (needs it) while Y needs none, so |a| is orientation-
  dependent by up to ~10 %. Needs a three-orientation calibration (+Z up, +X up, +Y up) to resolve
  three independent axis gains — a magnitude measurement can only ever constrain one number, so the
  three must not be inferred from fewer orientations. Gyro and the odometry-critical gz are unaffected.
- **`tBusHealth` bus-off deep branch** — could not force a bus-off in the bench mock; over-provisioned.
- **Charge-path timing** (current < 0) — needs a physical charger.
- **The real Jetson's p99 command gap** — the mock injected deliberately-pessimistic jitter (no RT
  config found in the ROS tree); `CMD_TIMEOUT_MS = 150` is set against the modelled tail, still wants
  confirming against the live host.
- **On-ground execution** — all timing above is bench/wheels-up; ground traction is a separate session.

---

*Metrics as of 2026-08-08 (I2C1 bus-recovery fix; B13/B13b/B13c: tImu, the IMU axis check and mount correction).
Reproduce via the methods in §7. Task ladder: `include/app_priorities.h`.*
