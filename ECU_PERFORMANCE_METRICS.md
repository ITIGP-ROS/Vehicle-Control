# ECU Performance Metrics — TM4C123 Ackermann Robot Controller

**Target:** TI TM4C123GH6PM · ARM Cortex-M4F @ **16 MHz** · 32 KB SRAM · 256 KB Flash
**RTOS:** FreeRTOS v11.1.0 (MIT), GCC/ARM_CM4F port, **preemptive**, **static allocation only**
**Scheduling:** fixed-priority preemptive, 10 tasks + idle · 1 kHz tick
**Last measured:** 2026-08-06 · tags: **[M] = measured on hardware**, **[E] = estimated / worst-case bound**

> This is the standalone ECU analysis: WCET, CPU load, schedulability, jitter, interrupt latency, and
> memory budget. Method is stated per metric so every number is reproducible. It complements
> `RTOS_TIMING_RESOURCE_BUDGET.md` (the working analysis) — this is the clean summary.

---

## 1. Headline numbers

| Metric | Value | Verdict |
|---|---|---|
| **Total CPU utilization `U`** | **≈ 4.9 %** (incl. 2.0 % kernel tick) | 🟢 ~95 % headroom |
| **Schedulability (RM bound, n=10)** | `U` 4.9 % vs bound **71.8 %** | 🟢 passes ~15× |
| **Schedulability (RTA, exact, `tVelocity`)** | worst-case response **≈ 95 µs** vs a **20,000 µs** deadline | 🟢 ~210× margin |
| **Worst-case interrupt latency** | **≈ 4 µs** (masked) / **0.75 µs** (above syscall priority) | 🟢 |
| **RAM usage (post stack-trim)** | **35.1 %** of 32 KB (was 68.3 %; 10,880 B freed) | 🟢 |
| **Flash usage** | ~57 KB of 256 KB (kernel adds ~11 KB) | 🟢 |
| **Memory model** | static allocation (MISRA-C:2012 Dir 4.12 / AUTOSAR "no dynamic memory after init") | 🟢 |

---

## 2. Per-task timing (WCET & utilization)

`Ti` = period / min inter-arrival · `Ci` = worst-case execution time · `Ui = Ci/Ti`

| task | prio | `Ti` | `Ci` (WCET) | tag | `Ui` | notes |
|---|---|---|---|---|---|---|
| `tSafety` | 10 | 10 ms | **2 µs** steady | [E] | 0.01 % | command-loss failsafe: 2 counter loads + compare + tick read. Trip path non-recurrent |
| `tVelocity` | 9 | **20 ms** (hard, QEI) | **60 µs** | [E] | 0.30 % | 2× QEI read + 2× PID (Tustin + deriv filter + anti-windup) + 2× Motor_SetSpeed. **Measured interval 20/20 ms, 0 early fires [M]** |
| `tRosRx` | 8 | 33.3 ms, burst 2 | 20 µs/frame | [E] | 0.12 % | ISR-semaphore woken; routes steering+velocity per 30 Hz cycle |
| `tBattery` | 7 | 100 ms | **479.9 µs** | **[M]** | 0.48 % | **MEASURED** — Ina226_ReadAll ~312 µs + ~168 µs SoC estimator |
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
| CAN frame on the wire | ~222 µs (8-byte std @ 500 kbps); **~4.9 % bus load** at 221 fr/s | datasheet + candump |
| `tVelocity` Update interval | **20/20 ms min/max, 0 early fires** | on-target instrumentation |

⚠️ **WCET is worst-case EXECUTION, not response time.** Early attempts measured with a wall-clock
(timer0) captured *response* time (execution + preemption) — corrected to measure execution only
(critical-section-bracketed or lowest-preemption context), per the B4 correction.

---


### 2.1 Priority ladder and measured stacks (final, post stack-trim)

Every task has a **distinct** priority: equal priorities time-slice under
`configUSE_TIME_SLICING`, and a time-slice between two tasks that share a
deadline or a bus is the same hazard as a mis-ordered pair.

| prio | task | period | stack used / allocated | margin |
|---|---|---|---|---|
| 10 | `tSafety` | 10 ms | 67 / 128 words | 1.91× |
| 9 | `tVelocity` | 20 ms | 66 / 128 | 1.94× |
| 8 | `tRosRx` | event (ISR-woken) | 80 / 128 | 1.60× |
| 7 | `tBattery` | 100 ms | 108 / 192 | 1.78× |
| 6 | `tCanTx` | event (queue) | 52 / 128 | 2.46× |
| 5 | `tBusHealth` | 100 ms | 31 / 192 | 6.19× ⚠️ |
| 4 | `tRosTx` | 5 ms (alternating) | 92 / 160 | 1.74× |
| 3 | `tClusterTx` | 50 ms (alternating) | 98 / 160 | 1.63× |
| 2 | `tOdo` | 100 ms | 36 / 128 | 3.56× |
| 1 | `tHeartbeat` | 1000 ms | 71 / 160 | 2.25× |

⚠️ `tBusHealth` is deliberately over-allocated: its measured 31 words are the
*no-bus-off* path only — the bus could not be forced off in the measurement
session, so its deep branch (`VelocityControl_Stop()` + `Can_RecoverBusOff()`)
is **unmeasured**. `tSafety` makes the same `Stop()` call at 67 words, so the
deep branch is expected near that.

## 3. CPU utilization & schedulability

**Total:** `U = Σ Ci/Ti ≈ 0.0475 ≈ 4.9 %` → headroom `1 − U ≈ 95.1 %`.

- **Rate-Monotonic sufficient test:** for n=10, the RM bound is `n(2^(1/n) − 1) ≈ 71.8 %`. `U = 4.9 %` is
  ~15× under it → sufficient condition satisfied. (Note: our priority order is by **criticality**, not
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
- **I2C** waits are TIMER0-tick-capped (200 µs cap, ~3.3× a healthy 60 µs command).
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
| **SRAM** | **35.1 %** (post-trim; 10,880 B freed from a deep-path mission) | 32 KB | 🟢 64.9 % free |
| **Flash** | ~57 KB (kernel ~11 KB) | 256 KB | 🟢 ~78 % free |

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

- **`tBusHealth` bus-off deep branch** — could not force a bus-off in the bench mock; over-provisioned.
- **Charge-path timing** (current < 0) — needs a physical charger.
- **The real Jetson's p99 command gap** — the mock injected deliberately-pessimistic jitter (no RT
  config found in the ROS tree); `CMD_TIMEOUT_MS = 150` is set against the modelled tail, still wants
  confirming against the live host.
- **On-ground execution** — all timing above is bench/wheels-up; ground traction is a separate session.

---

*Metrics as of 2026-08-06. Reproduce via the methods in §7. Working analysis:
`docs/RTOS_TIMING_RESOURCE_BUDGET.md`. Task ladder: `include/app_priorities.h`.*
