# ⚠️ Cluster team — `0x130` steering scale changed, and it is already live

**Date: 2026-08-16 · Firmware commit `c67cac1` · Image md5 `fe6df561…`**
**This is a notification, not a proposal. The firmware is flashed and running.**

---

## Read this part first — it bounds how much work this is

> ### Nothing structural changed. **Frame layout, byte offsets, types, units and sign
> ### convention are all identical.** Only the *numbers* moved.

You do **not** need to re-parse anything. `0x130` is the same 100 Hz frame, `steeringAngle`
is the same float32 in bytes 0–3, still radians, still **+ = LEFT** (REP-103), and the four
status bits are in the same places with the same meanings.

**One thing changed: the scale of the number.** If you have a gauge, it needs rescaling.
If you only display a wheel icon that rotates proportionally, you may only need to change
one full-scale constant.

---

## What changed

> ### The same physical wheel position now reports a number roughly **2× larger**.

This is not a range extension. It is a **scale correction**.

| | before | **now** |
|---|---|---|
| full LEFT | +0.2421 rad (+13.9°) | **+0.286 rad (+16.4°)** |
| full RIGHT | −0.3037 rad (−17.4°) | **−0.286 rad (−16.4°)** |
| symmetry | asymmetric | **symmetric** |

**Worked example.** With the wheels at one physical position, `0x130` now reports
**+0.14410** — that number is measured, wheels-up, on the flashed firmware. At the *same*
physical position the old firmware would have reported **≈ +0.070** (derived: that reading
corresponds to pot count 1577, and the old constants were centre 1991 / scale −0.00016882).
**Ratio ≈ 2.06**, not 2.31 — the pot *centre* moved as well (1991 → 1950), so the raw scale
ratio alone does not describe the change.

### 🔎 Exactly how an old-scaled gauge breaks — it is one-sided

Because the endpoints went from asymmetric to symmetric, the failure is **not** symmetric:

| side | new maximum | vs your old full-scale | effect |
|---|---|---|---|
| **LEFT** | +0.286 | old full-scale +0.2421 → **118.1 %** | 🔴 **overshoots by 18 %** — anything normalising or clamping against +0.2421 pins, wraps, or overflows |
| **RIGHT** | −0.286 | old full-scale −0.3037 → **94.2 %** | 🟡 never reaches full deflection — the gauge under-reads and the last 6 % is dead |

So a gauge built on the old endpoints does **not** simply saturate everywhere. It
**clips on the left and under-reads on the right**, which reads as a miscalibrated or
sticky display rather than as an obvious fault — worth knowing before someone chases it as
a hardware problem.

---

## Why it changed — the short version

The firmware's angle scale had been calibrated from a **CAD design figure and from the
steering potentiometer**, and had never been checked against the road wheels.

**The pot is not on the wheel.** It sits upstream, on the servo horn and linkage, so pot
travel and wheel angle differ by the linkage ratio — and that ratio had never been accounted
for. Measured by driving circles and tape-measuring them, the true wheel angle was
**2.31× larger on the left** and **1.91× larger on the right** than anything in the firmware
believed.

Concretely, two things in the feedback path were corrected:

| | before | now |
|---|---|---|
| counts→radians scale | one constant, −0.00016882 | **two**: −0.00038633 left (**×2.29**), −0.00032301 right (**×1.91**) |
| pot zero point | 1991 counts | **1950 counts** |

The single scale had to become two **because the linkage ratio genuinely differs by side** —
independently corroborated by hand measurement at the stops, where the outer wheel is
under-turned 4.1° at left lock and 12.1° at right lock. That asymmetry in the *linkage* is
what makes the *reported* endpoints symmetric now, where they used to be asymmetric.

**`0x130` now carries the true wheel angle.** It did not before.

---

## 🔴 Your display is misreading right now

The firmware is flashed on the vehicle. This is not a scheduled change.

| | |
|---|---|
| flashed | **2026-08-16** |
| firmware commit | **`c67cac1`** on `github.com:ITIGP-ROS/Vehicle-Control`, branch `main` |
| image md5 | **`fe6df56107120ec5f90106832f9b8c0d`** |

**To tell which firmware a given unit has:** ask whoever flashed it for the commit — that
is the only reliable way. As a behavioural hint, the **old** firmware's steering range was
**asymmetric** (noticeably more travel reported to the right than the left); the new one is
symmetric. If you see a unit whose reported right-hand maximum clearly exceeds its left-hand
maximum, it is on the old firmware.

---

## What you actually need to do

1. **Rescale the steering gauge / wheel icon** to full-scale **±0.286 rad (±16.4°)**,
   symmetric.
2. **Make full-scale a named constant, not a literal.** See the note below — this value is
   likely to widen once more measurement is done, and we would rather you change one
   constant than hunt for `0.3037` across a codebase.
3. **Re-check any clamping or saturation logic** built around the old asymmetric endpoints.
   Anything of the form "if angle < −0.30 then full right" is now unreachable.
4. **Nothing else.** See the next section.

> ### 📌 Expect this number to grow — design for it
> **±0.286 rad is a deliberately conservative ceiling, not the mechanism's limit.** It is the
> largest angle we have actually *measured* on the constraining (right) side. The vehicle can
> physically reach roughly **0.46–0.51 rad**, but we have not characterised that region on
> the right side and near the endpoints the delivered angle varies with battery voltage —
> so we clamp where the evidence stops.
>
> If we later measure the right side further, full-scale would move to about **±0.35 rad
> (±20°)**. **We will tell you if it does.** Leaving yourself a single constant to change is
> the whole of the preparation needed.

---

## What is NOT affected — so you can stop looking

We checked every frame you consume. **Only `0x130` is affected.**

| frame | signals | affected? |
|---|---|---|
| `0x200` VehicleStatus | `speed`, `gear`, `trip_m`, `odo_m` | ✅ **no change** |
| `0x210` BatteryStatus | all | ✅ **no change** |
| `0x130` SteeringFeedback | `steeringAngle` | 🔴 **scale changed** |
| `0x130` SteeringFeedback | `at_target`, `pot_fault`, `out_of_range`, `saturated` | 🟡 same meaning, **thresholds moved** |

`speed`, `gear`, `trip_m` and `odo_m` are derived from the **wheel encoders**, and the
battery signals from the battery service. The calibration touched neither. Your odometer and
speed readout are unaffected — **do not rescale them.**

### On the status bits

Their meanings are unchanged, but the angle at which two of them assert has moved, because
the firmware's clamp moved:

- **`out_of_range`** now asserts when a request exceeds **±0.286**, where it used to be
  `+0.2421 / −0.3037`.
- **`saturated`** likewise.

Two cautions that are unchanged but worth repeating, since you are touching this code anyway:

- **`at_target` must be read together with `out_of_range`** — `at_target` means "reached the
  *clamped* setpoint", so the pair together means "your request was beyond travel and we hit
  the limit", not "we are where you asked".
- **`pot_fault` means grey out the steering display.** We have observed the accompanying
  angle reading **−1.335 rad** — physically impossible, roughly 4.7× the mechanical limit.
  It is rare (order one frame in several thousand) but it is real and it does reach the bus.
  Do not render it. *(We do not currently filter it on our side either; that is logged as our
  own defect and is being fixed.)*

### One known oddity, so it does not surprise you

The **`saturated`** bit has been observed behaving inconsistently: set on **93.9 % of frames
(3423/3645)** during one 35 s arc, and on **0 % of 117,776 frames** across later runs at the
same commanded angle. We have flagged it to the firmware side. **Do not build display logic
that depends on `saturated` alone** until that is resolved — `out_of_range` is the more
trustworthy of the two.

---

## Reference

The full signal reference — `TIVA_TO_CLUSTER_signals.md` — is in the firmware repo and has
**already been updated** with the new range, so it and this notice agree.

Underlying evidence, if you want it: `REVIEW_steering_calibration_fix_20260816.md` (the
measurements, the fits, and what was deliberately *not* changed) and
`TIVA_STEERING_CALIBRATION_20260816.md` rev 3 on the Jetson side. Ask and we will send them.

**Questions to the Jetson/perception side.** If something in your display disagrees with
this note, tell us — twice in this project a confident number turned out to be wrong, and
both times it was someone downstream noticing the mismatch.
