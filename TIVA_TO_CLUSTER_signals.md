# Tiva → Cluster — CAN Signal Reference

Everything the Tiva sends the cluster, with real value ranges. **Tiva → Cluster only.**
Bus: CAN0, 500 kbps, standard 11-bit IDs. DBC: `can_ws/dbc/robot.dbc` (v2, 2026-08-05).

---

## 0x200 VehicleStatus — 10 Hz

| Signal | Bytes | Type | Scale | Unit | Real range you'll see | Notes |
|---|---|---|---|---|---|---|
| `speed` | 0–1 | u16 | 1 | **m/min** | **0 – ~120** | robot runs 0.7–2 m/s. Whole m/min (km/h looked parked). |
| `gear` | 2 | u8 enum | 1 | — | 0 / 1 / 2 | **0=N, 1=D, 2=R.** No Park. Derived from wheel direction. |
| `trip_m` | 3–4 | u16 | 1 | **m** | 0 – 65535 | TRIP since last reset/power-cycle. Reversing **adds** distance. Saturates at 65535, no wrap. **Reset-relative — see `odo_m`.** |
| `odo_m` | **5–7** | **u24** | 1 | **m** | 0 – 16 777 215 | **LIFETIME odometer. PERSISTENT — survives resets and power-cycles.** Held in the Tiva's internal EEPROM, written every 10 m, restored on boot. Reversing **adds**. Saturates, no wrap. |

**TRIP vs ODO — the question that keeps coming up.** They are different on purpose:

| | `trip_m` | `odo_m` |
|---|---|---|
| lifetime | **reset-relative** | **permanent** |
| zeroed by `0x140 ResetCommand` | **yes** | **no** |
| survives a power-cycle | no | **yes** (EEPROM) |
| width / ceiling | u16, 65.5 km | u24, 16 777 km |

`odo_m` did not exist before 2026-08-05 — that is why it was missing, not an
oversight in the decode. There is deliberately **no CAN path that can clear it**.
Worst case it loses **≤ 10 m** on a sudden power-off (the EEPROM write interval).
`0x200` is now **full** — all 8 bytes allocated.

---

## 0x210 BatteryStatus — 10 Hz

| Signal | Bytes | Type | Scale | Unit | Real range you'll see | Notes |
|---|---|---|---|---|---|---|
| `voltage` | 0 | u8 | 0.1 | **V** | **9.0 – 12.6** | 3S Li-ion. 0–25.5 V span. (mV precision stays on Tiva UART, not the wire.) |
| `current` | 1–2 | i16 | 0.01 | **A** | ~ −8 … +8 | **+ = discharge, − = charge.** Pack max 8 A. |
| `soc` | 3 | u8 | 1 | **%** | 0 – 100 | Estimate. Render as-is. |
| `power` | 4–5 | i16 | 1 | **W** | ~ −100 … +250 | = V×I. **+ = discharge, − = charge (regen).** This is what drives the POWER/REGEN strip. |
| `range` | 6–7 | u16 | 1 | **m** | 0 – 65535 | Estimated remaining range. **`0` = not estimable** (stopped or charging), NOT empty. |

**Battery is "Healthy" whenever the frame arrives at rate.** SoC/power/range are estimates — show them
as gauges, not lab values.

---

## 0x130 SteeringFeedback — 100 Hz

| Signal | Bits | Type | Scale | Unit | Real range you'll see | Notes |
|---|---|---|---|---|---|---|
| `steeringAngle` | 0–31 | float | — | **rad** | **−0.286 … +0.286** | **+ = LEFT** (REP-103). This is the real travel: ~ **−16.4° … +16.4°**, NOT a full wheel turn. **Recalibrated 2026-08-16 — see the warning below.** |
| `at_target` | b32 | bit | — | — | 0/1 | reached the (clamped) setpoint |
| `pot_fault` | b33 | bit | — | — | 0/1 | pot reading bad → **grey out the steering display** when set |
| `out_of_range` | b34 | bit | — | — | 0/1 | request exceeded travel, was clamped |
| `saturated` | b35 | bit | — | — | 0/1 | at the mechanical limit |

**Reading the status bits (important):**
- **Read `at_target` ONLY together with `out_of_range`.** `at_target` means "reached the *clamped*
  setpoint". `at_target` + `out_of_range` together = the request was beyond travel, clamped, and that
  limit was reached — not "we're where you asked".
- When `pot_fault` is set, `at_target` and `saturated` are suppressed (untrustworthy) — grey out.
- Rendering the wheel icon: map the ±angle to your icon rotation. Expect small travel (±~16°), and
  **left-endpoint angle is noisier than right** (mechanical, known) — light smoothing on the icon helps.

> ### ⚠️ `steeringAngle` WAS RECALIBRATED ON 2026-08-16 — re-check your gauge scaling
> **If your display was scaled against the old range, it now under-reads.**
>
> The old figures (−0.3037 … +0.2421) came from a CAD design angle and from the steering
> potentiometer. **Neither had ever been checked against the road wheels.** The pot sits
> upstream, on the servo horn / linkage, so pot travel and wheel angle differ by the linkage
> ratio — and that ratio was never accounted for. Measured against the wheels by driving
> circles and tape-measuring them, `0x130` was under-reporting by **2.31× on the left** and
> **1.91× on the right**.
>
> **`0x130` now carries the true wheel angle.** Nothing about the frame layout, units, sign
> convention or status bits changed — only the numbers are now correct. The endpoints are
> also symmetric now, where they used to be asymmetric.
>
> Small transient overshoot slightly past ±0.286 is normal (the clamp bounds the *command*,
> and the pot reports where the wheels actually are). Treat `out_of_range` / `saturated` as
> the authority on "at the limit", not the raw magnitude.

---

## Quick reference — steering angle endpoints
| | rad | degrees | |
|---|---|---|---|
| full LEFT | **+0.286** | **+16.4°** | was +0.2421 / +13.9° before 2026-08-16 |
| centre | 0.0 | 0° | |
| full RIGHT | **−0.286** | **−16.4°** | was −0.3037 / −17.4° before 2026-08-16 |

---

### Notes for the cluster
- **All frames Tiva→Cluster. Nothing here is a command** — display only.
- **A signal missing for a few frames = hold last value + optionally a stale indicator.** Don't blank on
  one drop.
- **These are the ONLY real data sources.** Anything else on the mockup (speed-limit sign, ambient temp,
  trip-energy graph, engine temp, drive modes) has **no Tiva backend** — decorative or drop it.
- Units were sized for THIS robot (m/min, W, m) on 2026-08-05 — make sure your decoder uses the v2 DBC.
