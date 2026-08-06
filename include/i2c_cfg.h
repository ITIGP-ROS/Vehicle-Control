#ifndef I2C_CFG_H_
#define I2C_CFG_H_

/*******************************************************************************
 *                          System Configuration                               *
 *******************************************************************************/

/* System clock frequency in Hz - independent copy of the same 16MHz
 * assumption duplicated in pwm_cfg.h/systick_cfg.h/uart_cfg.h/qei_cfg.h/
 * wdt_cfg.h (pwm_cfg.h:19-22 already flags this as a known, deferred,
 * repo-wide cleanup - not addressed by this driver alone). */
#define I2C_SYSTEM_CLOCK           (16000000UL)

/*******************************************************************************
 *                          I2C Module Enable                                  *
 *******************************************************************************/

/* Set to 1 to enable, 0 to disable */
/* By default, we will enable I2C0 which is PB2/PB3 on the launchpad */
#define I2C0_ENABLED               (1U)
#define I2C1_ENABLED               (1U)   /* MPU6050 IMU on PA6/PA7 */
#define I2C2_ENABLED               (0U)
#define I2C3_ENABLED               (0U)

/* I2C2 (PE4/PE5) is ELECTRICALLY UNAVAILABLE on this board - those pins are
 * committed to CAN0Rx/CAN0Tx (src/PORT_PBCFG.c:431-449), which is live in
 * production src/main.c. This is a hard compile-time block, not just a
 * comment, so enabling I2C2 by mistake fails the build instead of silently
 * fighting CAN for the same pins at runtime. See
 * REDAMEs/I2C_MCAL_DESIGN_PREP_AUDIT.md SS2 for the full pin-conflict
 * analysis (I2C0/I2C1/I2C3 are free, I2C2 is not, and there is no alternate
 * pin pair for I2C2 on this part). */
#if (I2C2_ENABLED == 1U)
#error "I2C2 (PE4/PE5) is blocked by CAN0Rx/CAN0Tx on this board - use I2C0 (PB2/PB3), I2C1 (PA6/PA7), or I2C3 (PD0/PD1) instead. See I2C_MCAL_DESIGN_PREP_AUDIT.md SS2."
#endif

/*******************************************************************************
 *                     Per-module target SCL frequency (Hz)                    *
 *  Plain Hz literals (NOT the I2C_SpeedModeType enum) so these are usable in *
 *  preprocessor arithmetic below - enum constants are not visible to the     *
 *  preprocessor, only to the compiler proper, so #if/#error guards on them   *
 *  would silently misevaluate.                                              *
 *                                                                            *
 *  Defined UNCONDITIONALLY for all 4 modules (not gated by I2Cn_ENABLED):    *
 *  I2C_InitWithConfig() is parameterized by a RUNTIME i2cId and picks its    *
 *  TPR via a switch() over all 4 IDs, so every ID needs a valid compile-time *
 *  constant to switch on even if that particular module is disabled this    *
 *  build - I2Cn_ENABLED only controls whether I2C_Init() brings a module up *
 *  automatically, not whether its constants exist. I2C2's value is an inert *
 *  placeholder (I2C2_ENABLED is compile-blocked above; this is never reached*
 *  through the normal I2C_Init() path). *
 *******************************************************************************/
/* INA226 (in hand) supports Fast mode (400 kHz) - matches the previous
 * I2C0_SPEED_MODE = I2C_SPEED_FAST default. */
#define I2C0_TARGET_HZ             (400000UL)

/* MPU6050 IMU. Raised 100k -> 400k on 2026-08-01 now the part is known: the
 * datasheet specifies Fast-mode "I2C Operating Frequency, All registers ...
 * 400 kHz" and "fSCL, SCL Clock Frequency ... 400 kHz"
 * (Tiva_DataSheet/MPU6050-DataSheet.pdf).
 *
 * This is not just "because we can": the shared per-command budget
 * I2C_CAP_DEFAULT_TICKS is 200 us, sized in i2c.h as "~4.4x a healthy ~45 us
 * single-byte command AT 400 kHz". At 100 kHz the same command takes ~180 us,
 * leaving only ~1.1x margin, so any clock-stretch by the slave trips the cap
 * and the transfer returns I2C_ERROR_TIMEOUT. Running the bus at the speed the
 * cap was designed for restores that margin. */
#define I2C1_TARGET_HZ             (400000UL)

/* Placeholder only - I2C2 is compile-blocked above (PE4/PE5 taken by CAN). */
#define I2C2_TARGET_HZ             (100000UL)

/* Reserved for future EEPROM - default conservative Standard mode
 * (100 kHz); most I2C EEPROMs support Fast mode but confirm against the
 * specific part's datasheet before raising this. */
#define I2C3_TARGET_HZ             (100000UL)

/*******************************************************************************
 *  Compile-time I2CMTPR (Timer Period Register) derivation.                  *
 *  Formula (datasheet "I2C Master Timer Period (I2CMTPR)", offset 0x00C,     *
 *  p.1025, SCL_LP/SCL_HP fixed at 6/4 per the same page):                    *
 *    SCL_PERIOD = 2 x (1 + TPR) x (SCL_LP + SCL_HP) x CLK_PRD                *
 *    => TPR = Fsys / (2 x (SCL_LP+SCL_HP) x Ftarget) - 1 = Fsys/(20*Ftarget)-1*
 *  Computed here at COMPILE TIME (mirrors pwm_cfg.h's PWM_PERIOD_LOAD        *
 *  pattern) so I2C_InitWithConfig() only ever writes a precomputed constant  *
 *  to I2CMTPR - no runtime division. TPR is a 7-bit field (range 1-127 per   *
 *  the same register description) - the #error guards below catch a         *
 *  target/clock combination that would over/underflow it at BUILD time      *
 *  rather than silently producing a wrong SCL frequency on hardware. At      *
 *  Fsys=16MHz: I2C0 (400kHz) -> TPR=1 (exact); I2C1/I2C3 (100kHz) -> TPR=7   *
 *  (exact) - see I2C_MCAL_DESIGN_PREP_AUDIT.md Step A for the worked math.  *
 *******************************************************************************/
#define I2C0_TPR   ((I2C_SYSTEM_CLOCK / (20UL * I2C0_TARGET_HZ)) - 1UL)
#define I2C1_TPR   ((I2C_SYSTEM_CLOCK / (20UL * I2C1_TARGET_HZ)) - 1UL)
#define I2C2_TPR   ((I2C_SYSTEM_CLOCK / (20UL * I2C2_TARGET_HZ)) - 1UL)
#define I2C3_TPR   ((I2C_SYSTEM_CLOCK / (20UL * I2C3_TARGET_HZ)) - 1UL)

#if ((I2C0_TPR < 1UL) || (I2C0_TPR > 127UL))
#error "I2C0_TPR out of the I2CMTPR 7-bit range [1,127] - check I2C_SYSTEM_CLOCK / I2C0_TARGET_HZ"
#endif
#if ((I2C1_TPR < 1UL) || (I2C1_TPR > 127UL))
#error "I2C1_TPR out of the I2CMTPR 7-bit range [1,127] - check I2C_SYSTEM_CLOCK / I2C1_TARGET_HZ"
#endif
#if ((I2C2_TPR < 1UL) || (I2C2_TPR > 127UL))
#error "I2C2_TPR out of the I2CMTPR 7-bit range [1,127] - check I2C_SYSTEM_CLOCK / I2C2_TARGET_HZ"
#endif
#if ((I2C3_TPR < 1UL) || (I2C3_TPR > 127UL))
#error "I2C3_TPR out of the I2CMTPR 7-bit range [1,127] - check I2C_SYSTEM_CLOCK / I2C3_TARGET_HZ"
#endif

/*******************************************************************************
 *                          Transaction timeout                                *
 *  REMOVED: I2C_TRANSACTION_TIMEOUT_MS (was 5 ms, SysTick-based). The BUSY-    *
 *  clear wait is now bounded by a per-command RAW TIMER0 tick budget passed   *
 *  in by the caller as `cap_ticks` (see the contract + I2C_CAP_DEFAULT_TICKS  *
 *  in i2c.h). This is self-clocked (no SysTick/ISR dependency) and can        *
 *  express a sub-millisecond bound, which the 1 ms-quantized SysTick timeout  *
 *  could not. The old macro was referenced only by the Phase-2 wait in        *
 *  i2c.c and is now unused, so it is deleted rather than left dangling.        *
 *******************************************************************************/

#endif /* I2C_CFG_H_ */
