# TM4C123GH6PM Pinout — Car-Like Robot

Authoritative source: `src/PORT_PBCFG.c` (post-build pin config applied by
`Port_Init(&Port_Configuration)`), cross-checked against `include/Motor.h`,
`include/pwm_cfg.h`, `include/qei_cfg.h`, `include/servo_cfg.h`,
`include/can_cfg.h`, and `include/uart_cfg.h`.

Board: **TI LaunchPad EK-TM4C123GXL** (TM4C123GH6PM, Cortex-M4 @ 80 MHz).
Any pin not listed below is left as default digital input (`PORT_DIGITAL_IO`,
`PORT_PIN_IN`) and is unused by the firmware.

---

## Active signal pins

| Pin  | Signal / Net           | Peripheral (mode)        | Dir | Init | Subsystem | Driver |
|------|------------------------|--------------------------|-----|------|-----------|--------|
| PB6  | Right-motor PWM        | `M0PWM0` (alt 4)         | OUT | LOW  | Motor R   | `PWM.c` / `Motor.c` |
| PD2  | Right-motor DIRECTION  | Digital GPIO             | OUT | HIGH | Motor R   | `Motor.c` |
| PB4  | Left-motor PWM         | `M0PWM2` (alt 4)         | OUT | LOW  | Motor L   | `PWM.c` / `Motor.c` |
| PD3  | Left-motor DIRECTION   | Digital GPIO             | OUT | HIGH | Motor L   | `Motor.c` |
| PD6  | Right encoder Phase A  | `PhA0` (alt 6)           | IN  | LOW  | QEI0 = RIGHT (`QEI_CHANNEL_0`) | `qei.c` |
| PD7  | Right encoder Phase B  | `PhB0` (alt 6)           | IN  | LOW  | QEI0 = RIGHT (`QEI_CHANNEL_0`) | `qei.c` |
| PC5  | Left encoder Phase A   | `PhA1` (alt 6)           | IN  | LOW  | QEI1 = LEFT (`QEI_CHANNEL_1`)  | `qei.c` |
| PC6  | Left encoder Phase B   | `PhB1` (alt 6)           | IN  | LOW  | QEI1 = LEFT (`QEI_CHANNEL_1`)  | `qei.c` |
| PC4  | Steering-servo PWM     | `WT0CCP0` (alt 7)        | OUT | LOW  | Servo (50 Hz) | `timer_pwm.c` / `servo.c` |
| PE3  | Steering-pot feedback  | `AIN0` (analog)          | IN  | —    | ADC       | `adc.c` |
| PE4  | CAN0 RX                | `CAN0Rx` (alt 8)         | IN  | LOW  | CAN0 500 kbps | `can.c` |
| PE5  | CAN0 TX                | `CAN0Tx` (alt 8)         | OUT | LOW  | CAN0 500 kbps | `can.c` |
| PB0  | UART1 RX (from Pi/Jetson) | `U1Rx` (alt 1)        | IN  | LOW  | UART1 115200 8N1 | `uart.c` |
| PB1  | UART1 TX (to Pi/Jetson)   | `U1Tx` (alt 1)        | OUT | LOW  | UART1 115200 8N1 | `uart.c` |
| PB2  | INA226 SCL             | `I2C0SCL` (alt 3)        | OUT | —    | I2C0 (INA226 current sensor) | `i2c.c` |
| PB3  | INA226 SDA             | `I2C0SDA` (alt 3)        | OD  | —    | I2C0 (INA226 current sensor) | `i2c.c` |
| PA6  | MPU6050 SCL            | `I2C1SCL` (alt 3)        | OUT | —    | I2C1 (MPU6050 IMU) | `i2c.c` / `mpu6050.c` |
| PA7  | MPU6050 SDA            | `I2C1SDA` (alt 3)        | OD  | —    | I2C1 (MPU6050 IMU) | `i2c.c` / `mpu6050.c` |
| PB5  | MPU6050 INT (data-rdy) | Digital GPIO             | IN  | LOW  | I2C1 (MPU6050 IMU) | **wired in HW, no firmware use yet** |
| PF1  | Fault / status LED (RED)   | Digital GPIO         | OUT | LOW  | On-board RGB LED | app (e.g. `staircase_can_test.c` abort) |
| PF2  | Status LED (BLUE)          | Digital GPIO         | OUT | LOW  | On-board RGB LED | (available) |
| PF3  | Status LED (GREEN)         | Digital GPIO         | OUT | LOW  | On-board RGB LED | (available) |

---

## Subsystem detail

### Motors (differential drive)
- **Right:** PWM `PB6` (`M0PWM0`), direction `PD2`. `MOTOR_RIGHT` / `PWM_CHANNEL_MOTOR_A`.
- **Left:**  PWM `PB4` (`M0PWM2`), direction `PD3`. `MOTOR_LEFT`  / `PWM_CHANNEL_MOTOR_B`.
- Both PWMs come from the **M0PWM** module. Direction pins init **HIGH** (`STD_HIGH`).
- The motors are mounted mirror-imaged, so `Motor.c` writes the **right DIR pin inverted**
  relative to the left, so `MOTOR_DIR_FORWARD` on both propels the chassis straight
  (see `Motor.h` header comment).

### Encoders (hardware QEI)
- **QEI0 = RIGHT motor** → `PD6`/`PD7` (`PhA0`/`PhB0`), `QEI_CHANNEL_0`.
  `QEI0_SWAP_SIGNALS = 1` (mirror-mount compensation) so forward chassis motion
  reads as +position/FORWARD, matching the left channel.
- **QEI1 = LEFT motor** → `PC5`/`PC6` (`PhA1`/`PhB1`), `QEI_CHANNEL_1`.
  `QEI1_SWAP_SIGNALS = 0`.
- x4 quadrature decode; CPR = 2464 counts/output-rev (11 PPR × 56 gear × 4).

### Steering servo
- 50 Hz servo PWM on `PC4` via **Wide Timer 0 CCP0** (`WT0CCP0`, alt 7).
- Center = 1650 µs (`SERVO_CENTER_PULSE_US`); travel 1020 µs (full LEFT) … 2480 µs (full RIGHT).
- Feedback pot on `PE3` (`AIN0`) read by the ADC (`adc.c`).

### CAN (CAN0, 500 kbps)
- `PE4` = `CAN0Rx`, `PE5` = `CAN0Tx`, both alt-func **mode 8**.
- Pins are muxed by the PORT driver (Option B); `can.c` touches no GPIO.
- 500 kbps @ 16 MHz CAN clock (`CANBIT = 0x3A41`, 75 % sample point).

### UART1 (host link — Raspberry Pi / Jetson)
- `PB0` = `U1Rx`, `PB1` = `U1Tx`, 115200 8N1.
- Used by the distance/velocity app stacks and for diagnostics/boot banners.

### I2C buses
Unlike the other peripherals, I2C pins are **muxed inside `i2c.c`** (the
`I2C_Module[]` table), *not* in `src/PORT_PBCFG.c` — so PB2/PB3 (and any future I2C
pins) still read as `PORT_DIGITAL_IO` in the PORT config. Enable flags live in
`include/i2c_cfg.h`.

- **I2C0 — INA226 current/voltage sensor (LIVE):** `PB2` = `I2C0SCL`, `PB3` =
  `I2C0SDA`, alt-func mode 3, addr `0x40`, Fast mode 400 kHz. Enabled
  (`I2C0_ENABLED=1`, `i2c_cfg.h`); pin table at `i2c.c` `I2C_ID_0`. SDA is
  open-drain; the driver enables only weak internal pull-ups — external pull-ups
  are on the sensor breakout.
- **I2C1 — MPU6050 IMU (ACTIVE, hardware-verified 2026-08-01):** `PA6` = `I2C1SCL`,
  `PA7` = `I2C1SDA`, alt-func mode 3, at **400 kHz**; plus **`PB5`** as the
  data-ready **INT** GPIO. Separate bus from the INA226 (I2C0). `I2C1_ENABLED=1`
  (`i2c_cfg.h`); pin table at `i2c.c` `I2C_ID_1`, which self-muxes PA6/PA7 — they
  are plain `PORT_DIGITAL_IO` in `PORT_PBCFG.c:130-148` and the I2C driver
  overrides them at `I2C_Init()`, exactly as I2C0 does for PB2/PB3.
  - **External pull-ups are required and present** — measured on hardware: with
    the pins released from the peripheral and an internal pull-DOWN engaged, both
    PA6 and PA7 still read HIGH (30/30), i.e. the breakout's pull-ups win. Do not
    rely on the MCU's weak internal pull-ups alone.
  - **400 kHz is not optional here.** `I2C_CAP_DEFAULT_TICKS` is 200 us, sized in
    `i2c.h` as "~4.4x a healthy ~45 us single-byte command AT 400 kHz". At the old
    100 kHz the same command takes ~180 us, leaving ~1.1x margin, and the transfer
    returned `I2C_ERROR_TIMEOUT`. The MPU6050 datasheet specifies Fast-mode 400 kHz
    for all registers, so the bus runs at the speed the cap was designed for.
  - ⚠️ **`PB5` INT is WIRED IN HARDWARE BUT UNUSED BY FIRMWARE.** No ISR, no GPIO
    interrupt configuration, and `PORT_PBCFG.c:200-208` still has PB5 as a plain
    input (`PORT_DIGITAL_IO`/IN, no internal resistor). The bench polls on a 20 ms
    cadence instead. PB5 is genuinely free for this: none of its alt-funcs
    (`SSI2Fss`, `M0PWM3`, `T1CCP1`, `CAN0Tx`) is used on this board — CAN0Tx is on
    PE5 — it needs no `GPIOCR` unlock, and no GPIO-Port-B ISR exists today, so the
    port's interrupt vector is uncontended. **INT was previously documented as PE0
    (and before that PA5); the hardware is on PB5.** See `MPU6050_PIN_AUDIT.md` for
    the original bus-selection rationale (why I2C1/PA6-PA7 over I2C3/PD0-PD1) —
    note its INT-pin conclusion is superseded by this entry.
- **I2C2 (PE4/PE5) is unavailable** — those pins are committed to CAN0; enabling
  I2C2 is a hard compile `#error` (`i2c_cfg.h`).

### On-board RGB LED (PF1/PF2/PF3)
- All three configured as digital outputs, init LOW.
- `PF1` (red) is the **fault indicator** — e.g. the `staircase_can` CAN-dump abort
  drives it HIGH. PF2 (blue) / PF3 (green) are available.

---

## Planned / proposed additions (NOT yet wired in firmware)

_The MPU6050 IMU has moved to the **Active signal pins** table above — I2C1 on
PA6/PA7 is live and hardware-verified. The only part still outstanding is the
**PB5 INT** line: it is physically wired but no firmware configures or services
it (the driver polls instead), so it is listed there with that caveat rather
than here._


---

## Reserved / debug pins (not in `PORT_PBCFG.c`, fixed by hardware)
| Pin | Function |
|-----|----------|
| PA0 / PA1 | UART0 → USB debug (ICDI virtual COM), used by the bootloader/debugger |
| PC0–PC3   | JTAG/SWD (TCK/TMS/TDI/TDO) — debug/flash via TI ICDI |
| PF0 / PF4 | On-board push-buttons SW2/SW1 (left as inputs; not used by firmware) |

---

## Per-build activity

Only one application stack runs at a time; lower layers (Motor/QEI/UART) are shared.

| Pin group | `lptm4c123gh6pm` (production) | `staircase_can` (on-ground System-ID) |
|-----------|:---:|:---:|
| Motors PB6/PD2, PB4/PD3 | ✅ | ✅ (both driven, same duty) |
| Encoders PD6/PD7, PC5/PC6 | ✅ | ✅ (both logged) |
| Servo PC4 | ✅ | ✅ (held at 1650 µs center) |
| Pot ADC PE3 | ✅ | ❌ (no ADC in this env) |
| CAN PE4/PE5 | ✅ | ✅ (sample dump, IDs 0x7F0/0x7F1) |
| UART1 PB0/PB1 | ✅ | ✅ (boot banner + abort only) |
| Fault LED PF1 | app-dependent | ✅ (dump-abort indicator) |
