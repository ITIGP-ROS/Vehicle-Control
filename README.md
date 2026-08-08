# Vehicle-Control ECU — TM4C123 Ackermann Robot

Firmware for the **vehicle-control ECU** of an autonomous Ackermann-steered robot.
It runs on a TI **TM4C123GH6PM** (ARM Cortex-M4F), closes the per-wheel velocity
loop, drives the steering servo, and is the node that actually moves the vehicle.
It takes commands from a **Jetson running ROS 2** and publishes telemetry to an
**instrument cluster**, both over a single 500 kbps CAN bus.

The firmware is written in bare-metal C against an **AUTOSAR-flavoured layered
architecture** (MCAL → HAL → services → comm → application), and runs under
**FreeRTOS in preemptive mode** as an **eleven-task set with fully static memory
allocation** — no heap anywhere in the image.

```
        ┌──────────────┐   0x100 velocity      ┌──────────────────┐
        │ Jetson (ROS) │ ──0x120 steering───►  │                  │
        │  30 Hz       │ ◄─0x110 wheel ticks── │  THIS ECU        │   ┌────────┐
        └──────────────┘   0x130 steering fb   │  TM4C123GH6PM    │──►│ motors │
                           0x150/0x160 IMU     │  FreeRTOS, 11    │   │ servo  │
        ┌──────────────┐   0x200 vehicle       │  tasks, static   │◄──│ QEI+IMU│
        │   Cluster    │ ◄─0x210 battery────── │                  │   │ pot    │
        └──────────────┘                       └──────────────────┘   └────────┘
```

---

## Hardware

| | |
|---|---|
| **MCU** | TI TM4C123GH6PM, Cortex-M4F @ **16 MHz** (MOSC direct, PLL bypassed) |
| **Flash / RAM** | 256 KB / 32 KB |
| **Drive** | 2 × JGB37-520 gearmotors, rear pair, independently driven |
| **Feedback** | Quadrature encoders via the hardware **QEI** peripheral, CPR 2464 |
| **Steering** | Front Ackermann linkage, one servo (50 Hz PWM), pot feedback via ADC |
| **Battery** | 12.6 V 3S Li-ion, 20 Ah, monitored by an **INA226** over I²C |
| **Bus** | CAN0 @ **500 kbps**, 11-bit standard IDs |

Full pin assignment: **[`PINOUT.md`](PINOUT.md)**.

> The vehicle is **not** differential drive. Velocity and steering are two
> independent command channels — a fact that shapes the whole comm layer.

---

## Architecture

Dependencies point **downward only**. A service never includes a comm header; a
comm module never reaches into control internals; only `app.c` spans the
service↔comm seam.

| Tier | Modules |
|---|---|
| **4 · Application** | `app.c` — creates the task set, owns the buffers, starts the scheduler. `main.c` is a thin shell: hardware init, then `App_Start()`. |
| **3 · Comm / routing** | `jetson_comm`, `cluster_comm`, `node_ping`, `can_tx_queue` — protocol only. **Nothing calls `Can_Transmit` directly** except the TX-queue task. |
| **2 · Services** | `velocity_control` (PID), `steering_control`, `battery_service` (hybrid coulomb/voltage SoC), `odo` (EEPROM-persisted odometer) |
| **1 · HAL** | `Motor`, `encoder`, `pid`, `servo`, `servo_feedback`, `comm_data`, `ina226` |
| **0/1 · MCAL** | `can`, `i2c`, `uart`, `PWM`, `qei`, `adc`, `timer0`, `timer_pwm`, `systick`, `eeprom`, `wdt`, `PORT` |

### The task set

Eleven tasks, **every one at a distinct priority** — equal priorities time-slice,
and a time-slice between tasks that share a deadline or a bus is the same hazard
as a mis-ordered pair.

| Prio | Task | Period | Role |
|---|---|---|---|
| **11** | `tSafety` | 10 ms | RX command-loss failsafe — nothing may preempt it |
| **10** | `tVelocity` | 20 ms | Per-wheel velocity PID against the QEI window |
| **9** | `tRosRx` | event | Routes `0x100`/`0x120`/`0x140` — woken by the CAN ISR |
| **8** | `tBattery` | 100 ms | INA226 read (I2C0) + SoC estimator |
| **7** | `tImu` | 20 ms | MPU6050 read (I2C1) + packs `0x150`/`0x160` → 50 Hz each |
| **6** | `tCanTx` | event | The **only** caller of `Can_Transmit`; drains the TX queue |
| **5** | `tBusHealth` | 100 ms | CAN bus-off observation and recovery |
| **4** | `tRosTx` | 5 ms | Packs `0x110` / `0x130`, alternating → 100 Hz each |
| **3** | `tClusterTx` | 50 ms | Packs `0x200` / `0x210`, alternating → 10 Hz each |
| **2** | `tOdo` | 100 ms | Non-blocking EEPROM save state machine |
| **1** | `tHeartbeat` | 1 s | Diagnostics console line |

Two design points worth calling out:

- **One hardware CAN TX object, so transmits are strictly serial.** Rather than
  hand-timing frames into exclusive millisecond slots, a single `tCanTx` drains a
  queue and *sleeps on the ISR's TX-complete edge*. The
  one-transmit-at-a-time invariant is therefore **structural** — one task, one
  object — instead of depending on someone re-deriving slot arithmetic before
  adding a sixth frame.
- **Every periodic task is phase-locked to an absolute tick residue.** Phase is a
  property of the task *set*, not of one task: two producers waking on the same
  tick would post two frames into one millisecond. A new task's residue is
  cleared against every existing one with the pairwise rule *two periods P,Q with
  residues r,s can collide iff r ≡ s (mod gcd(P,Q))* — checking "mod 5" alone
  only clears it against the fastest producer.
- **A driver with a wall-clock timeout is effectively a real-time critical
  section.** `i2c.c` caps each command against a free-running timer, so a task
  preempting an I2C reader eats that margin directly. This is why `tBattery` and
  `tImu` both sit **above** `tCanTx` (which wakes ~320×/s) despite carrying only
  telemetry — getting it wrong once made the state-of-charge estimate silently
  wrong by 42 %.

---

## ★ ECU performance

Measured and analysed rather than assumed. Full tables, per-metric methodology,
and the `[M]`easured / `[E]`stimated tag on every value are in
**[`ECU_PERFORMANCE_METRICS.md`](ECU_PERFORMANCE_METRICS.md)**.

| Metric | Value | |
|---|---|---|
| **CPU utilization** | **≈ 8.6 %** (of which the 1 kHz kernel tick is 2.0 %) | ~91 % headroom |
| **Schedulability — RM bound** | 8.6 % against the n=11 bound of 71.4 % | passes ~8× |
| **Schedulability — RTA (exact)** | `R(tVelocity)` ≈ **95 µs** vs a **20 ms** deadline | ~210× margin |
| **Worst-case interrupt latency** | ≈ **4 µs** masked / 0.75 µs above the syscall ceiling | |
| **RAM** | **37.8 %** of 32 KB, after trimming stacks to measured worst-case | ≥1.60× margin on every task |
| **Flash** | ~59 KB of 256 KB (the kernel is ~11 KB of it) | |
| **Memory model** | **Static allocation** — no heap in the image | MISRA-C:2012 Dir 4.12 |

Notes on how those numbers were obtained, because the method is the point:

- **Schedulability uses RTA, not just the utilisation bound.** The priority
  assignment is deliberately **criticality-monotonic, not rate-monotonic** —
  `tSafety` has a 20× longer period than `tRosTx` yet outranks it — so the Liu &
  Layland bound is only indicative and response-time analysis is the actual proof.
- **Timing is measured with the free-running 16 MHz TIMER0**, whose read is a
  single tear-free `LDR`, so no debug probe is needed on the vehicle.
  ⚠️ Wall-clock measurement across a preemptible region gives **response time**,
  not execution time; the document labels which is which rather than conflating them.
- **Stacks are trimmed against a deliberate deep-path mission** — driving under
  PID, a command-loss failsafe *while driving*, and an EEPROM save — not against
  an idle run. Idle-run high-water marks are lower bounds and trimming on them is
  how a stack overflow ships.
- **Every wait in the firmware is time-bounded, never iteration-bounded.** An
  iteration cap's duration changes with the optimiser; a TIMER0 deadline does not.

---

## CAN interface

500 kbps, 11-bit IDs. Signal-level detail: **[`TIVA_TO_CLUSTER_signals.md`](TIVA_TO_CLUSTER_signals.md)**;
the machine-readable source of truth is [`can_ws/dbc/robot.dbc`](can_ws/dbc/robot.dbc),
from which `robot.c`/`robot.h` are generated.

| ID | Frame | Dir | Rate | Contents |
|---|---|---|---|---|
| `0x100` | VelocityCommand | Jetson → ECU | 30 Hz | left/right wheel setpoints, f32 rad/s |
| `0x120` | SteeringCommand | Jetson → ECU | 30 Hz | steering angle, f32 rad, **+left (REP-103)** |
| `0x140` | ResetCommand | Jetson → ECU | on activate | zero the encoder ticks and trip |
| `0x110` | VelocityFeedback | ECU → Jetson | 100 Hz | cumulative signed wheel ticks, int32 |
| `0x130` | SteeringFeedback | ECU → Jetson | 100 Hz | measured angle + status bits |
| `0x150` | ImuAccel | ECU → Jetson | 50 Hz | accel x/y/z (int16 ×0.001 m/s²) + `sequence` |
| `0x160` | ImuGyroFlags | ECU → Jetson | 50 Hz | gyro x/y/z (int16 ×0.001 rad/s), `imu_reset`, `sequence` |
| `0x200` | VehicleStatus | ECU → cluster | 10 Hz | speed, gear, trip, odometer |
| `0x210` | BatteryStatus | ECU → cluster | 10 Hz | voltage, current, SoC, power, range |
| `0x7A0/1` | NodePing | host ↔ ECU | on demand | liveness echo |

**Sign convention:** everything from the CAN boundary down to `steering_control`
is REP-103 (+left). The servo HAL's native frame is inverted, and there is
**exactly one** negate in the whole chain. Do not add a second.

**The IMU pair `0x150`/`0x160` is ONE unit, not two frames.** Both carry the same
`sequence` byte (at different offsets — 6 in ImuAccel, 7 in ImuGyroFlags), and
the host **rejects its entire sensor read — encoder ticks included — if the two
disagree**. They are therefore packed from a single sample and posted
back-to-back, which is the one place this firmware deliberately spends the
"≤ 1 transmit per millisecond" *wire* property: the real invariant (one transmit
in flight) is still structural in the queue, and adjacency shrinks the host's
tear window from ~50 % to ~1 %. Verified on the wire: 4,300+ pairs, 0 mismatches.

**IMU frame:** the MPU6050 is physically mounted **180° yawed** (+X rear,
+Y right). The Tiva corrects that at its single transform site
(`ImuService_Latch` negates accel X/Y and gyro X/Y), so what goes on the bus is a
true REP-103 robot frame and **the host needs no axis remap**. ⚠️ Gyro **Z is
deliberately not negated** — yaw rate is invariant under a yaw rotation, so it
was already correct; "making it consistent" with X/Y would reintroduce a bug that
was previously found and fixed on hardware. Gyro bias is sent **raw** by design;
the host owns that calibration.

---

## Build and flash

Requires [PlatformIO](https://platformio.org/) — it fetches the `titiva`
platform and the ARM toolchain on first build. **The FreeRTOS kernel is vendored
in `lib/`, not pulled from a registry**, so this repository builds offline and
builds the same way in a year.

```bash
git clone git@github.com:ITIGP-ROS/Vehicle-Control.git
cd Vehicle-Control

pio run                  # build
pio run -t upload        # flash over the on-board TI ICDI debugger
pio device monitor       # console @115200
```

There is **one** environment, `lptm4c123gh6pm`, so those commands need no `-e`.

> **Console:** UART1 on **PB0/PB1** @115200 through an **external USB-serial
> adapter** — *not* the ICDI virtual COM port (UART0/PA0-PA1 is disabled).

```
RAM:   [====      ]  35.1% (used 11500 bytes from 32768 bytes)
Flash: [==        ]  21.9% (used 57432 bytes from 262144 bytes)
```

---

## Safety features

- **RX command-loss failsafe.** If no *accepted* `0x100`/`0x120` arrives for
  150 ms — a value derived from the host's measured 30 Hz cadence, at 2.9× the
  observed p99 inter-frame gap — the drive is zeroed and **the steering is
  HELD, never re-centred**: snapping the wheels straight mid-corner is its own
  hazard. It counts *accepted* commands, not arrivals, so a partially-crashed
  host spraying malformed frames cannot hold the failsafe off.
- **The stop cannot be undone by a race.** A latched inhibit flag means that even
  if the failsafe preempts the control loop mid-update, the next update
  *re-asserts* the stop rather than overwriting it — self-healing under any
  interleaving, with no lock and therefore no priority inversion.
- **All peripheral waits are bounded by TIME**, on the free-running TIMER0 —
  I²C, ADC and CAN cannot hang the caller, and a dead peripheral degrades to a
  reported fault instead of a stalled task.
- **Static memory only.** Insufficient RAM is a *linker* error, caught on the
  bench: the linker script asserts that ≥4 KB remains between `.bss` and the
  stack top. There is no `pvPortMalloc` in the image to fail in the field.
- **Stack-overflow detection** stays enabled (`configCHECK_FOR_STACK_OVERFLOW = 2`).
- **Persistent odometer** in EEPROM with a wear-levelling ring and a
  tear-safe write order — a power loss mid-save cannot corrupt the stored value.
- **CAN bus-off recovery** that commands a stop *before* rejoining, so the
  vehicle never resumes on a stale setpoint.
- **I²C bus-wedge recovery.** A slave that strands mid-transfer holding SDA low
  cannot be freed by resetting the master — the classic failure where the sensor
  goes silent until someone power-cycles it. On `BUS_STUCK` the driver now clocks
  the bus free with up to 9 SCL pulses and retries once, so the IMU **self-clears
  without an MCU reset**. Measured: 22 such events in a 7-minute soak under host
  commands and motion, every one recovered in 1–4 pulses, 21,017 good reads after
  the first fault. The recovery is per-bus, so the INA226 on I²C0 is untouched.

---

## Status and future work

The firmware is complete and verified on the bench: the command path, control
loop, telemetry, failsafe and persistence have all been exercised on hardware
against a mock of the real ROS node's CAN behaviour. The IMU pair `0x150`/`0x160`
**is produced** (since B13) at 50 Hz with the matched-sequence contract, and the
I²C1 bus-wedge that used to take it out permanently now self-recovers.

Honestly not yet done:

- **On-ground drive test.** All verification to date has been *wheels-up*.
  Traction-loaded behaviour, real stopping distance and odometry accuracy over a
  measured course remain to be confirmed.
- **Charge-path sign.** Negative current/power is unverified — it needs a charger
  on the pack, which was never connected.
- **Real-Jetson cadence confirmation.** The 150 ms timeout was validated against a
  mock reproducing the ROS node's behaviour with deliberately pessimistic jitter;
  the real host's p99 should be confirmed once it is on the bus.
- **CAN bus-off recovery** is implemented but was never forced during testing.
- **The I²C1 fault TRIGGER is not isolated.** The bus wedge described under safety
  features auto-recovers, but *what disturbs the bus in the first place* was never
  pinned down: it fires with zero actuation, yet doubling the host command rate
  made it stop. Rate and jitter both differed between those runs and which matters
  is unproven, so it is not guessed at — the firmware is built to **survive** the
  event rather than prevent it. Watch `irec` in the heartbeat: rising slowly with
  `imu=H` is the system working; rising fast means the electrical side wants a look.
- **Per-axis accelerometer scale.** `IMU_ACCEL_SCALE_CORR` is a single scalar for
  all three axes, so |a| is orientation-dependent by up to ~10 %. Resolving three
  independent axis gains needs a three-orientation capture. Gyro — and therefore
  the odometry-critical yaw rate — is unaffected.

---

## Licence

The vendored FreeRTOS kernel (`lib/FreeRTOS-Kernel/`) is MIT — see
[`lib/FreeRTOS-Kernel/LICENSE.md`](lib/FreeRTOS-Kernel/LICENSE.md).
