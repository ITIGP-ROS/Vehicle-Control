/******************************************************************************
 *
 * Module: I2C (MCAL)
 *
 * File Name: i2c.c
 *
 * Description: Multi-module (I2C0/I2C1/I2C2/I2C3), multi-device (slave
 *              address per transaction) polling I2C master driver.
 *
 * POLLING, NOT INTERRUPT-DRIVEN - deliberate for this pass. Traffic is
 * low-rate (INA226 polled at 10 Hz today; a future IMU/EEPROM would still be
 * well under saturating even Standard-mode I2C), and this project's other
 * time-critical loops (velocity control @20ms, systick) are unaffected by a
 * few hundred microseconds of blocking per I2C transaction. Interrupt/DMA
 * support is explicitly OUT OF SCOPE for this pass - see
 * REDAMEs/I2C_MCAL_DESIGN_PREP_AUDIT.md.
 *
 * Every register macro and bit offset used below was re-confirmed against
 * Tiva_DataSheet/DATASHEET ARM M4.pdf (titled TM4C123GE6PM - same silicon/
 * pinout/peripheral map as this board's GH6PM, differs only in Flash size,
 * per TI's part-number key) immediately before this file was written - see
 * the register-map confirmation this implementation was reviewed against.
 *
 ******************************************************************************/

#include "i2c.h"
#include "tm4c123gh6pm_registers.h"
#include "PORT_REGS.h"     /* GPIO_PORTx_BASE_ADDRESS - named, for bus-recovery
                             * per-pin GPIODATA bit-band addressing only */
#include "timer0.h"         /* Timer0_GetTicks()/Timer0_ElapsedTicks() - raw
                             * TIMER0 tick basis for the per-command timeout cap.
                             * Replaced SysTick: self-clocked (no ISR dependency,
                             * closes the SysTick-masked-caller hang) and can
                             * express a sub-millisecond bound. REQUIRES
                             * Timer0_FreeRunning_Init() before the first I2C op. */

/*******************************************************************************
 *                          Private Macros                                     *
 *******************************************************************************/

/* I2CMCS status bits (read side) - datasheet "I2C Master Control/Status
 * (I2CMCS)", offset 0x004, p.1019-1020. */
#define I2C_MCS_CLKTO       0x80    /* bit7 - clock timeout (hw feature unused, not armed) */
#define I2C_MCS_BUSBSY      0x40    /* bit6 - I2C bus busy (tracks START/STOP on the bus) */
#define I2C_MCS_IDLE        0x20    /* bit5 - controller idle */
#define I2C_MCS_ARBLST      0x10    /* bit4 - arbitration lost */
#define I2C_MCS_DATACK      0x08    /* bit3 - data byte NOT acknowledged (1=NACK) */
#define I2C_MCS_ADRACK      0x04    /* bit2 - address NOT acknowledged (1=NACK) */
#define I2C_MCS_ERROR       0x02    /* bit1 - error on last operation */
#define I2C_MCS_BUSY        0x01    /* bit0 - controller busy; "other status bits
                                      * not valid" while set (datasheet p.1020) */

/* I2CMCS command bits (write side) - same offset, p.1021-1022. */
#define I2C_MCS_HS          0x10    /* bit4 - High-Speed enable (unused, Standard/Fast only) */
#define I2C_MCS_ACK         0x08    /* bit3 - auto-ACK the received byte */
#define I2C_MCS_STOP        0x04    /* bit2 - generate STOP */
#define I2C_MCS_START       0x02    /* bit1 - generate START (or REPEATED START if the
                                      * bus was never STOPped since the last START -
                                      * datasheet Table 16-5, p.1022-1023) */
#define I2C_MCS_RUN         0x01    /* bit0 - master enabled to transmit/receive */

/* Command sequences - bit-for-bit cross-checked against Table 16-5 "Write
 * Field Decoding for I2CMCS[3:0]" (p.1022-1023) in the Step A register-map
 * review; every value below matches a named row of that table. */
#define I2C_CMD_SINGLE_SEND          (I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN)  /* 0x07 */
#define I2C_CMD_SINGLE_RECEIVE       (I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN)  /* 0x07, ACK=0 */
#define I2C_CMD_BURST_SEND_START     (I2C_MCS_START | I2C_MCS_RUN)                 /* 0x03, NO STOP */
#define I2C_CMD_BURST_SEND_CONT      (I2C_MCS_RUN)                                 /* 0x01 */
#define I2C_CMD_BURST_SEND_FINISH    (I2C_MCS_STOP | I2C_MCS_RUN)                  /* 0x05 */
#define I2C_CMD_BURST_RECEIVE_START  (I2C_MCS_ACK | I2C_MCS_START | I2C_MCS_RUN)   /* 0x0B */
#define I2C_CMD_BURST_RECEIVE_CONT   (I2C_MCS_ACK | I2C_MCS_RUN)                   /* 0x09 */
#define I2C_CMD_BURST_RECEIVE_FINISH (I2C_MCS_STOP | I2C_MCS_RUN)                  /* 0x05, ACK=0 */

/* BUSY-ASSERT bound: after a command word is written to I2CMCS, BUSY does not
 * go high for ~4-8 SYSCLK cycles (the write has to cross the APB bridge and
 * the I2C master state machine has to actually start the transaction) - see
 * I2C_WaitForCommandComplete() below for the full explanation. This bound
 * only needs to be "a few dozen instructions" worth of spins, which even at
 * the slowest supported SYSCLK is microseconds - deliberately generous so it
 * never gates on build/optimization level, while still failing fast (not
 * hanging) if BUSY genuinely never asserts (dead/unclocked peripheral). */
#define I2C_BUSY_ASSERT_SPINS   (1000UL)

/* AUTO-ABORT cap: after a command times out we assert STOP and wait (capped)
 * for BUSBSY to clear. The STOP itself can hang if a wedged slave holds SDA
 * low, so this wait MUST be TIMER0-capped - never a blocking BUSBSY spin.
 * Provisional 200 us; retune this named constant, do not bury the literal. */
#define I2C_ABORT_CAP_US        (200U)
#define I2C_ABORT_CAP_TICKS     TIMER0_US_TO_TICKS(I2C_ABORT_CAP_US)   /* = 3200 */

/* SCL-toggle recovery half-period settle. >= the 400 kHz half-period (1.25 us);
 * 5 us also covers 100 kHz (5 us) and gives a wedged slave margin to release.
 * TIMER0-timed (NOT a NOP loop - NOP timing is -O dependent).
 *
 * OPEN-DRAIN RISE TIME: during recovery SCL and SDA are open-drain, so a line
 * going "high" is an RC RISE via the external pull-up, NOT a driven edge. This
 * half-period MUST be >= the worst-case pull-up rise time of THIS bus before
 * any line is sampled (RecoverStep) or a STOP edge is emitted (RecoverEnd),
 * otherwise a still-rising line is misread. 5 us is generous for a typical
 * ~4.7k pull-up / few-hundred-pF bus, but the exact rise is board-specific ->
 * HARDWARE-VERIFY item (scope the SCL rise). Do NOT shorten this. */
#define I2C_RECOVER_SCL_HALF_US (5U)
#define I2C_RECOVER_SCL_HALF_TICKS  TIMER0_US_TO_TICKS(I2C_RECOVER_SCL_HALF_US)  /* = 80 */

/* Reset-propagation settle after asserting SRI2C (a few clocks per datasheet).
 * TIMER0-timed so the recovery path (RecoverEnd -> I2C_Reset) has NO NOP loop. */
#define I2C_RESET_SETTLE_US     (10U)
#define I2C_RESET_SETTLE_TICKS  TIMER0_US_TO_TICKS(I2C_RESET_SETTLE_US)  /* = 160 */

/*******************************************************************************
 *                          Per-module descriptor table                        *
 *  One row per I2C_IdType value - the SINGLE source of truth for "everything  *
 *  about module N": its own MSA/MCS/MDR/MTPR/MCR registers, its precomputed   *
 *  (compile-time, i2c_cfg.h) TPR, and its SCL/SDA GPIO pin wiring. Replaces   *
 *  the previous per-register switch() helpers AND the masked-pointer-and-hex- *
 *  offset GPIO addressing with named macros only (tm4c123gh6pm_registers.h / *
 *  PORT_REGS.h) - matches how PORT.c resolves GPIO base addresses.           *
 *******************************************************************************/

typedef struct
{
    volatile uint32 *msa;
    volatile uint32 *mcs;
    volatile uint32 *mdr;
    volatile uint32 *mtpr;
    volatile uint32 *mcr;
    uint32            tpr;          /* precomputed I2CMTPR value, from i2c_cfg.h */

    volatile uint32 *gpioAfsel;
    volatile uint32 *gpioOdr;
    volatile uint32 *gpioDen;
    volatile uint32 *gpioPur;
    volatile uint32 *gpioPctl;
    volatile uint32 *gpioDir;       /* for I2C_RecoverBus() manual bit-banging */
    uint32            gpioBaseAddr; /* for I2C_RecoverBus() per-pin GPIODATA aliasing */
    uint32            gpioRcgcBit;  /* SYSCTL_RCGCGPIO_REG bit for this port (0=A..5=F) */
    uint8             sclPin;
    uint8             sdaPin;
} I2C_ModuleDescType;

/* SCL/SDA pin assignments and PCTL=3 encoding: datasheet Table 16-1 "I2C
 * Signals (64LQFP)", p.~1010 (re-confirmed in Step A / prior audit SS1).
 * PCTL encoding is written as a literal 3 directly in I2C_InitWithConfig()
 * (all four modules use the same encoding) rather than stored per-row here. */
static const I2C_ModuleDescType I2C_Module[I2C_ID_MAX] =
{
    /* I2C_ID_0: PB2=SCL, PB3=SDA */
    {
        &I2C0_MSA_REG, &I2C0_MCS_REG, &I2C0_MDR_REG, &I2C0_MTPR_REG, &I2C0_MCR_REG, I2C0_TPR,
        &GPIO_PORTB_AFSEL_REG, &GPIO_PORTB_ODR_REG, &GPIO_PORTB_DEN_REG, &GPIO_PORTB_PUR_REG,
        &GPIO_PORTB_PCTL_REG, &GPIO_PORTB_DIR_REG, GPIO_PORTB_BASE_ADDRESS, 1UL, 2U, 3U
    },
    /* I2C_ID_1: PA6=SCL, PA7=SDA */
    {
        &I2C1_MSA_REG, &I2C1_MCS_REG, &I2C1_MDR_REG, &I2C1_MTPR_REG, &I2C1_MCR_REG, I2C1_TPR,
        &GPIO_PORTA_AFSEL_REG, &GPIO_PORTA_ODR_REG, &GPIO_PORTA_DEN_REG, &GPIO_PORTA_PUR_REG,
        &GPIO_PORTA_PCTL_REG, &GPIO_PORTA_DIR_REG, GPIO_PORTA_BASE_ADDRESS, 0UL, 6U, 7U
    },
    /* I2C_ID_2: PE4=SCL, PE5=SDA - ELECTRICALLY BLOCKED (CAN0Rx/CAN0Tx own
     * these pins on this board, src/PORT_PBCFG.c:431-449). Row present only
     * for array completeness; i2c_cfg.h compile-blocks I2C2_ENABLED so
     * I2C_Init() can never bring this module up. */
    {
        &I2C2_MSA_REG, &I2C2_MCS_REG, &I2C2_MDR_REG, &I2C2_MTPR_REG, &I2C2_MCR_REG, I2C2_TPR,
        &GPIO_PORTE_AFSEL_REG, &GPIO_PORTE_ODR_REG, &GPIO_PORTE_DEN_REG, &GPIO_PORTE_PUR_REG,
        &GPIO_PORTE_PCTL_REG, &GPIO_PORTE_DIR_REG, GPIO_PORTE_BASE_ADDRESS, 4UL, 4U, 5U
    },
    /* I2C_ID_3: PD0=SCL, PD1=SDA */
    {
        &I2C3_MSA_REG, &I2C3_MCS_REG, &I2C3_MDR_REG, &I2C3_MTPR_REG, &I2C3_MCR_REG, I2C3_TPR,
        &GPIO_PORTD_AFSEL_REG, &GPIO_PORTD_ODR_REG, &GPIO_PORTD_DEN_REG, &GPIO_PORTD_PUR_REG,
        &GPIO_PORTD_PCTL_REG, &GPIO_PORTD_DIR_REG, GPIO_PORTD_BASE_ADDRESS, 3UL, 0U, 1U
    }
};

/* PCTL (PMCn) encoding for the I2C alternate function - datasheet Table 16-1,
 * "3" for every I2CnSCL/I2CnSDA signal on every module. */
#define I2C_PCTL_ENCODING   (3UL)

/*******************************************************************************
 *                          Private Variables                                  *
 *******************************************************************************/

/* Cache configs to allow for seamless hardware resets */
static I2C_ConfigType g_I2C_Configs[I2C_ID_MAX];

/* Saved SCL/SDA pin state across a RecoverBegin..RecoverEnd window, so the exact
 * prior AF/GPIO config (incl. SDA open-drain ODR) is restored on RecoverEnd.
 * Only the SCL|SDA bits (and their PCTL nibbles) are stashed - surgical, never
 * clobbers other pins on the shared port. */
typedef struct
{
    uint32  afsel;      /* AFSEL bits for SCL|SDA                       */
    uint32  pctl;       /* PCTL nibbles for SCL|SDA                     */
    uint32  dir;        /* DIR bits for SCL|SDA                         */
    uint32  odr;        /* ODR bits for SCL|SDA (SDA open-drain)        */
    boolean active;     /* TRUE between RecoverBegin and RecoverEnd     */
} I2C_RecoverSaveType;

static I2C_RecoverSaveType g_I2C_RecoverSave[I2C_ID_MAX];

/*******************************************************************************
 *                          Private Helper Functions                           *
 *******************************************************************************/

/**
 * @brief  Bounded busy-wait of exactly `ticks` TIMER0 ticks. Always terminates
 *         (fixed interval), TIMER0-timed - the only kind of delay allowed in
 *         the recovery path (no NOP loops: their timing is -O dependent).
 */
static void I2C_DelayTicks(uint32 ticks)
{
    uint32 start = Timer0_GetTicks();
    while (Timer0_ElapsedTicks(start) < ticks) { }
}

/**
 * @brief  Local, best-effort bus cleanup after a command times out. Asserts a
 *         STOP (I2CMCS STOP bit = the TivaWare BURST_*_ERROR_STOP command) and
 *         waits - TIMER0-CAPPED - for BUSBSY to clear.
 * @return I2C_ERROR_TIMEOUT  if BUSBSY cleared within I2C_ABORT_CAP_TICKS
 *                            (bus is clean; caller/HAL just retries), or
 *         I2C_ERROR_BUS_STUCK if the cap was exceeded (SDA still held low; the
 *                            HAL must run SCL-toggle recovery).
 * @note   The STOP can itself hang on a wedged slave, which is exactly why the
 *         BUSBSY wait is capped and NOT a blocking spin.
 */
static I2C_StatusType I2C_AutoAbort(volatile uint32 *mcs_reg)
{
    uint32 startTicks;

    /* Assert STOP to release the bus locally (STOP bit alone = abort+STOP). */
    *mcs_reg = I2C_MCS_STOP;

    startTicks = Timer0_GetTicks();
    while ((*mcs_reg & I2C_MCS_BUSBSY) != 0U)
    {
        if (Timer0_ElapsedTicks(startTicks) >= I2C_ABORT_CAP_TICKS)
        {
            return I2C_ERROR_BUS_STUCK;   /* SDA stuck -> HAL runs SCL recovery */
        }
    }
    return I2C_ERROR_TIMEOUT;             /* bus clean -> simple retry */
}

/**
 * @brief  Wait for a just-issued I2CMCS command to complete, then decode its
 *         result.
 *
 * BUSY-ASSERT DELAY (highest-priority fix, TM4C123 I2C quirk): after a
 * command word is written to I2CMCS, the BUSY bit does NOT go high for ~4-8
 * SYSCLK cycles - the write has to cross the APB bridge and the I2C master
 * state machine has to actually latch the command and start driving the bus.
 * If this function read BUSY immediately after the write and found it 0, that
 * 0 could be STALE (left over from the end of the PREVIOUS transaction, not
 * yet updated for this one) - the caller would wrongly conclude a transaction
 * that hasn't even started yet is already done, then read I2CMDR too early
 * (stale/garbage data) or issue the next command while the bus is still
 * mid-byte.
 *
 * MECHANISM USED: spin until BUSY is OBSERVED to assert (bounded,
 * I2C_BUSY_ASSERT_SPINS iterations - microseconds even at the slowest
 * supported SYSCLK) BEFORE ever trusting a "not busy" reading; only once
 * BUSY has been seen high do we start the real (time-based) wait for it to
 * clear again. This self-adapts to core clock/build-optimization level,
 * unlike a fixed NOP-count delay, and correctly fails I2C_ERROR_TIMEOUT
 * (rather than silently reporting success) if BUSY never asserts at all -
 * without this explicit check, a "wait for BUSY==1 then wait for BUSY==0"
 * sequence would otherwise treat "never started" and "already finished
 * before we looked" identically.
 *
 * PHASE-2 TIMEOUT BASIS: the BUSY-clear wait is bounded by cap_ticks, a RAW
 * TIMER0 tick budget supplied by the caller (per command). This replaced the
 * old SysTick 1 ms-quantized timeout: TIMER0 is self-clocked (no ISR, so a
 * caller with interrupts masked can no longer hang here) and can express a
 * sub-millisecond bound. Comparison is raw ticks - NO division, NO microseconds.
 *
 * @param mcs_reg    the module's I2CMCS register.
 * @param cap_ticks  max TIMER0 ticks to wait for BUSY to clear (> 0, < 2^31);
 *                   see the contract in i2c.h. REQUIRES TIMER0 to be running.
 */
static I2C_StatusType I2C_WaitForCommandComplete(volatile uint32 *mcs_reg, uint32 cap_ticks)
{
    uint32 assertSpins = I2C_BUSY_ASSERT_SPINS;
    uint32 startTicks;

    /* Phase 1: wait for BUSY to assert - see function header comment. LEFT
     * AS-IS: already a bounded spin-count (I2C_BUSY_ASSERT_SPINS=1000, ~750 us
     * worst case), never was SysTick-dependent, so it is not part of this
     * change. */
    while (((*mcs_reg & I2C_MCS_BUSY) == 0U) && (assertSpins > 0U))
    {
        assertSpins--;
    }
    if (assertSpins == 0U)
    {
        /* BUSY never asserted at all - the command was never actually
         * accepted/started by the hardware (e.g. clock not really enabled).
         * Report this explicitly rather than falling through to phase 2,
         * where a never-asserted BUSY would look identical to "already
         * finished" and this function would wrongly return I2C_OK. */
        return I2C_ERROR_TIMEOUT;
    }

    /* Phase 2: wait for BUSY to clear - real transaction complete, bounded by
     * the caller's raw TIMER0 tick budget. Timer0_ElapsedTicks() is the
     * wrap-safe (now - start) & 0xFFFFFFFF, exact for any interval < 2^32 ticks
     * (268 s); cap_ticks << 2^31 keeps the unsigned compare unambiguous.
     *
     * ✅ T25-1 CLOSED - this comment used to end: "If TIMER0 is not running,
     * elapsed stays 0 and this NEVER times out - the caller's startup order must
     * guarantee Timer0_FreeRunning_Init() ran." That is no longer true in either
     * half. A not-running counter now makes Timer0_ElapsedTicks() return
     * TIMER0_ELAPSED_INVALID (0xFFFFFFFF), which exceeds any legal cap, so this
     * wait EXPIRES ON ITS FIRST EVALUATION instead of spinning forever - and
     * I2C_InitWithConfig() refuses to initialise on a dead timebase in the first
     * place, so this path is not normally reachable. The startup order still
     * matters; it is simply no longer enforced by prose alone. */
    startTicks = Timer0_GetTicks();
    while ((*mcs_reg & I2C_MCS_BUSY) != 0U)
    {
        if (Timer0_ElapsedTicks(startTicks) >= cap_ticks)
        {
            /* AUTO-ABORT on the cap-exceeded path, INSIDE the per-command wait:
             * this fires immediately on whichever of the (up to 3) read/write
             * commands timed out - not after the outer I2C_Read/Write returns -
             * leaving the bus as clean as local STOP can make it before we hand
             * back. Returns I2C_ERROR_TIMEOUT (clean, retry) or
             * I2C_ERROR_BUS_STUCK (SDA held -> HAL SCL recovery). */
            return I2C_AutoAbort(mcs_reg);
        }
    }

    /* Per datasheet p.1020: "When BUSY is set, the other status bits are not
     * valid" - only safe to inspect ERROR/ARBLST/etc. now that BUSY==0. */
    if ((*mcs_reg & I2C_MCS_ERROR) != 0U)
    {
        if ((*mcs_reg & I2C_MCS_ARBLST) != 0U)
        {
            return I2C_ERROR_ARBITRATION_LOST;
        }
        return I2C_ERROR_NO_ACK;
    }
    return I2C_OK;
}

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

I2C_StatusType I2C_InitWithConfig(I2C_IdType i2cId, const I2C_ConfigType *config)
{
    const I2C_ModuleDescType *mod;
    volatile uint32 dummy;

    if (i2cId >= I2C_ID_MAX) { return I2C_ERROR_INVALID_ID; }
    if (config == NULL_PTR)  { return I2C_ERROR_NULL_PTR; }

    /* T25-1: REFUSE TO COME UP ON A DEAD TIMEBASE.
     *
     * Every bounded wait in this driver is measured with Timer0_ElapsedTicks(),
     * so this module's entire "no operation can hang forever" guarantee - the
     * one REVIEW 22 signed off - is CONDITIONAL on TIMER0 running. Until now
     * that condition was enforced by nothing but comments in six files, and
     * violating it did not degrade: Timer0_GetTicks() bus-faulted, or (with the
     * clock on but the counter stopped) elapsed stayed 0 and every wait here
     * became silently unbounded.
     *
     * Checking ONCE here, rather than per-command, is deliberate: the property
     * is a startup-order property, it cannot change while we run, and the hot
     * path must not pay for it. Timer0 itself now fails fast per call as a
     * backstop (ElapsedTicks returns TIMER0_ELAPSED_INVALID, tripping any cap
     * immediately), so the two mechanisms cover each other - this one REPORTS
     * the fault at the right layer at the right moment, that one bounds the
     * damage if anything slips past. */
    if (Timer0_IsRunning() == FALSE)
    {
        return I2C_ERROR_NOT_INITIALIZED;
    }

    /* Store config so I2C_Reset can use it later */
    g_I2C_Configs[i2cId] = *config;

    mod = &I2C_Module[i2cId];

    /* 1. Enable I2C clock and GPIO clock. GPIO clocking is technically
     * already enabled globally by Port_Init() (SYSCTL_RCGCGPIO_REG |= 0x3F,
     * src/PORT.c:89) before any application code runs, so this is a
     * redundant-but-harmless OR that makes this module usable standalone
     * even if that init-ordering assumption ever changes. */
    SYSCTL_RCGCI2C_REG |= (1UL << (uint32)i2cId);
    SYSCTL_RCGCGPIO_REG |= (1UL << mod->gpioRcgcBit);

    /* Give hardware time to enable clocks (read-back delay, same pattern as
     * the driver this replaces). */
    dummy = SYSCTL_RCGCGPIO_REG;
    dummy = SYSCTL_RCGCI2C_REG;
    (void)dummy;

    /* 2. Configure pins - AFSEL, ODR (SDA only - datasheet p.56387-56402:
     * "the corresponding port pin should NOT be configured as open drain"
     * for SCL, which has a fixed active pull-up in hardware), DEN, PUR,
     * PCTL. All via NAMED per-register macros (I2C_Module table above) -
     * no pointer-mask-and-hex-offset arithmetic. */
    *(mod->gpioAfsel) |= (1UL << mod->sclPin) | (1UL << mod->sdaPin);
    *(mod->gpioOdr)   |= (1UL << mod->sdaPin);
    *(mod->gpioDen)   |= (1UL << mod->sclPin) | (1UL << mod->sdaPin);
    *(mod->gpioPur)   |= (1UL << mod->sclPin) | (1UL << mod->sdaPin);

    *(mod->gpioPctl) &= ~((0xFUL << (4UL * mod->sclPin)) | (0xFUL << (4UL * mod->sdaPin)));
    *(mod->gpioPctl) |= (I2C_PCTL_ENCODING << (4UL * mod->sclPin))
                      | (I2C_PCTL_ENCODING << (4UL * mod->sdaPin));

    /* 3. Initialize the I2C Master - MFE (Master Function Enable) only,
     * datasheet "I2C Master Configuration (I2CMCR)" offset 0x020, p.1030. */
    *(mod->mcr) = 0x00000010UL;

    /* 4. Clock speed - I2CMTPR is a PRECOMPUTED COMPILE-TIME constant from
     * i2c_cfg.h (I2C0_TPR..I2C3_TPR), NOT computed here at runtime. HS bit
     * (bit7) stays 0 - Standard/Fast only, no High-Speed mode used. */
    *(mod->mtpr) = mod->tpr;

    return I2C_OK;
}

I2C_StatusType I2C_Init(void)
{
    I2C_StatusType status = I2C_OK;
    I2C_ConfigType config;

#if (I2C0_ENABLED == 1U)
    config.speedMode = (I2C0_TARGET_HZ >= 400000UL) ? I2C_SPEED_FAST : I2C_SPEED_STANDARD;
    status = I2C_InitWithConfig(I2C_ID_0, &config);
    if (status != I2C_OK) return status;
#endif
#if (I2C1_ENABLED == 1U)
    config.speedMode = (I2C1_TARGET_HZ >= 400000UL) ? I2C_SPEED_FAST : I2C_SPEED_STANDARD;
    status = I2C_InitWithConfig(I2C_ID_1, &config);
    if (status != I2C_OK) return status;
#endif
    /* I2C2_ENABLED is compile-blocked to 0 in i2c_cfg.h (PE4/PE5 taken by
     * CAN) - intentionally no I2C_ID_2 branch here. */
#if (I2C3_ENABLED == 1U)
    config.speedMode = (I2C3_TARGET_HZ >= 400000UL) ? I2C_SPEED_FAST : I2C_SPEED_STANDARD;
    status = I2C_InitWithConfig(I2C_ID_3, &config);
    if (status != I2C_OK) return status;
#endif

    return status;
}

I2C_StatusType I2C_Write(I2C_IdType i2cId, uint8 slaveAddr, uint8 regAddr, const uint8 *data, uint16 length, uint32 cap_ticks)
{
    const I2C_ModuleDescType *mod;
    I2C_StatusType status;
    uint16 i;

    if (i2cId >= I2C_ID_MAX) { return I2C_ERROR_INVALID_ID; }
    if ((data == NULL_PTR) && (length > 0U)) { return I2C_ERROR_NULL_PTR; }

    mod = &I2C_Module[i2cId];

    /* Wait if bus is busy (a DIFFERENT master or a leftover STOP-less
     * transaction is on the bus right now - distinct from the per-command
     * BUSY check done after each I2CMCS write below). */
    if ((*(mod->mcs) & I2C_MCS_BUSBSY) != 0U) { return I2C_ERROR_BUS_BUSY; }

    /* I2CMSA = (7-bit slave address << 1) | R/S.  bit0 = R/S (0=write,
     * 1=read), bits[7:1] = the 7-bit address - datasheet "I2C Master Slave
     * Address (I2CMSA)", offset 0x000, p.1018. Example: INA226 at 0x40 ->
     * write address byte 0x80 (this line), read address byte 0x81
     * (I2C_Read below). This shift was already present and correct in the
     * driver being replaced - not a bug that needed fixing here, just
     * confirmed against the datasheet per the review request. */
    *(mod->msa) = ((uint32)slaveAddr << 1) | 0UL;

    /* Send register address */
    *(mod->mdr) = regAddr;

    if (length == 0U)
    {
        /* Single byte write (just the register address, e.g. a
         * command-trigger register with no payload). Complete, self-
         * contained transaction - STOP|START|RUN is correct here (nothing
         * follows this byte). */
        *(mod->mcs) = I2C_CMD_SINGLE_SEND;
        return I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
    }

    /* Burst write: register-address phase holds the bus open (START|RUN, NO
     * STOP) because data bytes follow - this was already correct in the
     * driver being replaced. */
    *(mod->mcs) = I2C_CMD_BURST_SEND_START;
    status = I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
    if (status != I2C_OK) return status;

    for (i = 0U; i < (length - 1U); i++)
    {
        *(mod->mdr) = data[i];
        *(mod->mcs) = I2C_CMD_BURST_SEND_CONT;
        status = I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
        if (status != I2C_OK) return status;
    }

    *(mod->mdr) = data[length - 1U];
    *(mod->mcs) = I2C_CMD_BURST_SEND_FINISH;
    return I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
}

I2C_StatusType I2C_Read(I2C_IdType i2cId, uint8 slaveAddr, uint8 regAddr, uint8 *data, uint16 length, uint32 cap_ticks)
{
    const I2C_ModuleDescType *mod;
    I2C_StatusType status;
    uint16 i;

    if (i2cId >= I2C_ID_MAX) { return I2C_ERROR_INVALID_ID; }
    if ((data == NULL_PTR) || (length == 0U)) { return I2C_ERROR_NULL_PTR; }

    mod = &I2C_Module[i2cId];

    if ((*(mod->mcs) & I2C_MCS_BUSBSY) != 0U) { return I2C_ERROR_BUS_BUSY; }

    /* --- Address phase: write the register pointer --- */
    *(mod->msa) = ((uint32)slaveAddr << 1) | 0UL;   /* R/S=0 (write), see I2C_Write comment above */
    *(mod->mdr) = regAddr;

    /* TRUE REPEATED-START, NOT STOP-THEN-START: this MUST be
     * I2C_CMD_BURST_SEND_START (START|RUN, bit pattern 0x03, NO STOP bit) -
     * NOT I2C_CMD_SINGLE_SEND (0x07), which the driver being replaced used
     * here and which asserts STOP, releasing the bus. Per datasheet Table
     * 16-5 (p.1022-1023), a repeated-START is simply issuing START=1 again
     * WITHOUT an intervening STOP - the hardware distinguishes "fresh START"
     * from "repeated START" purely by whether the bus was released since the
     * last START, not by a different bit encoding. If this line used
     * SINGLE_SEND (STOP=1), the STOP would fully end the transaction and the
     * data-phase START below would be a FRESH START on an idle bus, not a
     * true repeated-START - which breaks EEPROM random-address reads (the
     * device's internal address latch can reset on STOP) and can return a
     * stale/wrong INA226 register if another master (or noise) gets the bus
     * in between. Using BURST_SEND_START here holds the bus continuously
     * across the R/S flip below. */
    *(mod->mcs) = I2C_CMD_BURST_SEND_START;
    status = I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
    if (status != I2C_OK) return status;

    /* --- Data phase: re-address for READ (R/S=1), bus still held --- */
    *(mod->msa) = ((uint32)slaveAddr << 1) | 1UL;   /* R/S=1 (read) - e.g. INA226 0x40 -> 0x81 */

    /* RECEIVE ACK DISCIPLINE (datasheet Table 16-5, p.1022-1023): the LAST
     * byte of any read - whether it's the only byte or the final byte of a
     * burst - must be NACKed (ACK bit = 0) and then STOPped, or the slave is
     * left thinking more bytes are wanted and keeps driving the bus/holding
     * SCL, wedging the next transaction. Every byte BEFORE the last one must
     * be ACKed (ACK bit = 1) so the slave knows to keep sending. Concretely:
     *   length == 1 : single byte, NACK+STOP  -> SINGLE_RECEIVE   (0x07, ACK=0)
     *   length  > 1 : first byte,  ACK, no stop -> BURST_RECEIVE_START (0x0B, ACK=1)
     *                 middle bytes, ACK, no stop -> BURST_RECEIVE_CONT  (0x09, ACK=1)
     *                 last byte,    NACK+STOP    -> BURST_RECEIVE_FINISH(0x05, ACK=0)
     * This was already correct in the driver being replaced; kept as-is,
     * now just following directly from a genuine repeated-START instead of
     * a stop-then-restart. */
    if (length == 1U)
    {
        *(mod->mcs) = I2C_CMD_SINGLE_RECEIVE;
        status = I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
        if (status != I2C_OK) return status;
        data[0] = (uint8)(*(mod->mdr));
        return I2C_OK;
    }

    *(mod->mcs) = I2C_CMD_BURST_RECEIVE_START;
    status = I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
    if (status != I2C_OK) return status;
    data[0] = (uint8)(*(mod->mdr));

    for (i = 1U; i < (length - 1U); i++)
    {
        *(mod->mcs) = I2C_CMD_BURST_RECEIVE_CONT;
        status = I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
        if (status != I2C_OK) return status;
        data[i] = (uint8)(*(mod->mdr));
    }

    *(mod->mcs) = I2C_CMD_BURST_RECEIVE_FINISH;
    status = I2C_WaitForCommandComplete(mod->mcs, cap_ticks);
    if (status != I2C_OK) return status;
    data[length - 1U] = (uint8)(*(mod->mdr));

    return I2C_OK;
}

/*******************************************************************************
 *          Bus-recovery primitives (public; HAL sequences them)               *
 *                                                                             *
 * Decomposition of the retired blocking I2C_RecoverBus() into begin/step/end  *
 * so the HAL owns policy and NONE of the old for(delay<1000) NOP loops remain *
 * - every wait here is TIMER0-timed via I2C_DelayTicks(). Per-pin GPIODATA    *
 * aliasing (base + ((1<<pin)<<2)) is the TM4C's documented single-pin access  *
 * (datasheet "GPIO Data (GPIODATA)", offset 0x000, p.661: bits [9:2] of the   *
 * address bus act as a write/read mask).                                      *
 *******************************************************************************/

void I2C_RecoverBegin(I2C_IdType i2cId)
{
    const I2C_ModuleDescType *mod;
    uint32 sclMask, sdaMask, pinMask, pctlMask;
    volatile uint32 *sclData;

    if (i2cId >= I2C_ID_MAX) { return; }

    /* Begin/End contract (FIX 4): refuse a double-Begin. Overwriting the stash
     * while a recovery is already in progress would later restore a WRONG
     * AFSEL/PCTL/DIR/ODR and silently break the bus. Only stash when not already
     * active; a second Begin is ignored (stash + in-progress pin state kept).
     * (void API kept - "do not overwrite the stash" per the contract options.) */
    if (g_I2C_RecoverSave[i2cId].active != FALSE) { return; }

    mod = &I2C_Module[i2cId];

    sclMask  = (1UL << mod->sclPin);
    sdaMask  = (1UL << mod->sdaPin);
    pinMask  = sclMask | sdaMask;
    pctlMask = (0xFUL << (4UL * mod->sclPin)) | (0xFUL << (4UL * mod->sdaPin));

    /* GPIO clock (redundant-but-harmless, see I2C_InitWithConfig). */
    SYSCTL_RCGCGPIO_REG |= (1UL << mod->gpioRcgcBit);
    I2C_DelayTicks(I2C_RESET_SETTLE_TICKS);

    /* Stash the EXACT prior AF/GPIO config (SCL|SDA bits + their PCTL nibbles)
     * so RecoverEnd restores it byte-for-byte - surgical, never clobbers other
     * pins on the shared port. */
    g_I2C_RecoverSave[i2cId].afsel  = *(mod->gpioAfsel) & pinMask;
    g_I2C_RecoverSave[i2cId].pctl   = *(mod->gpioPctl)  & pctlMask;
    g_I2C_RecoverSave[i2cId].dir    = *(mod->gpioDir)   & pinMask;
    g_I2C_RecoverSave[i2cId].odr    = *(mod->gpioOdr)   & pinMask;
    g_I2C_RecoverSave[i2cId].active = TRUE;

    /* Take SCL/SDA off the I2C alternate function to plain GPIO. */
    *(mod->gpioAfsel) &= ~pinMask;
    *(mod->gpioPctl)  &= ~pctlMask;
    *(mod->gpioDen)   |=  pinMask;
    *(mod->gpioPur)   |=  pinMask;

    /* BOTH SCL and SDA OPEN-DRAIN (FIX 1): ODR SET on both. Writing 0 pulls the
     * line low; writing 1 RELEASES it to the external pull-up (does NOT actively
     * drive high). This is mandatory - a push-pull master driving SCL high
     * against a clock-stretching slave holding SCL low (the exact fault we
     * recover) is a direct contention/short through the pin drivers. SCL is a
     * GPIO OUTPUT (we toggle its level, open-drain); SDA starts as INPUT so it
     * is OBSERVED, not driven - detecting the slave RELEASING SDA is the whole
     * point; driving SDA defeats recovery. */
    *(mod->gpioOdr) |=  sclMask;   /* SCL open-drain (release-high, not driven) */
    *(mod->gpioOdr) |=  sdaMask;   /* SDA open-drain                            */
    *(mod->gpioDir) |=  sclMask;   /* SCL = output (open-drain)                 */
    *(mod->gpioDir) &= ~sdaMask;   /* SDA = input (observed)                    */

    /* Idle: RELEASE SCL high (RC rise via the pull-up, not a driven edge). The
     * half-period settle MUST cover the pull-up rise time before any sample -
     * see I2C_RECOVER_SCL_HALF_US (HARDWARE-VERIFY item). */
    sclData  = (volatile uint32 *)(mod->gpioBaseAddr + (sclMask << 2));
    *sclData = sclMask;            /* open-drain high = release to pull-up */
    I2C_DelayTicks(I2C_RECOVER_SCL_HALF_TICKS);
}

uint8 I2C_RecoverStep(I2C_IdType i2cId)
{
    const I2C_ModuleDescType *mod;
    uint32 sclMask, sdaMask;
    volatile uint32 *sclData;
    volatile uint32 *sdaData;

    /* Not begun / bad id: report "released" so the HAL's early-exit loop ends. */
    if (i2cId >= I2C_ID_MAX)                       { return 1U; }
    if (g_I2C_RecoverSave[i2cId].active == FALSE)  { return 1U; }

    mod     = &I2C_Module[i2cId];
    sclMask = (1UL << mod->sclPin);
    sdaMask = (1UL << mod->sdaPin);
    sclData = (volatile uint32 *)(mod->gpioBaseAddr + (sclMask << 2));
    sdaData = (volatile uint32 *)(mod->gpioBaseAddr + (sdaMask << 2));

    /* One SCL pulse, SCL open-drain: release-high -> settle -> pull-low ->
     * settle -> release-high. "high" writes RELEASE to the pull-up (RC rise),
     * they do not drive; each settle is TIMER0-timed and MUST cover the pull-up
     * rise (I2C_RECOVER_SCL_HALF_TICKS) so the last release is fully high before
     * SDA is sampled. */
    *sclData = sclMask;                              /* release high (RC rise) */
    I2C_DelayTicks(I2C_RECOVER_SCL_HALF_TICKS);
    *sclData = 0UL;                                  /* pull low               */
    I2C_DelayTicks(I2C_RECOVER_SCL_HALF_TICKS);
    *sclData = sclMask;                              /* release high (RC rise) */
    I2C_DelayTicks(I2C_RECOVER_SCL_HALF_TICKS);

    /* Sample SDA AFTER the pulse (SCL settled high): 1 = released (high),
     * 0 = still held low. */
    return ((*sdaData & sdaMask) != 0UL) ? 1U : 0U;
}

I2C_StatusType I2C_RecoverEnd(I2C_IdType i2cId)
{
    const I2C_ModuleDescType *mod;
    uint32 sclMask, sdaMask, pinMask, pctlMask;
    volatile uint32 *sclData;
    volatile uint32 *sdaData;
    I2C_StatusType status;

    if (i2cId >= I2C_ID_MAX) { return I2C_ERROR_INVALID_ID; }

    /* Begin/End contract (FIX 4): refuse an End with no matching Begin - there
     * is nothing to restore, and doing the STOP/reset blind could disturb a bus
     * that is in normal AF mode. */
    if (g_I2C_RecoverSave[i2cId].active == FALSE) { return I2C_ERROR_NOT_INITIALIZED; }

    mod = &I2C_Module[i2cId];

    sclMask  = (1UL << mod->sclPin);
    sdaMask  = (1UL << mod->sdaPin);
    pinMask  = sclMask | sdaMask;
    pctlMask = (0xFUL << (4UL * mod->sclPin)) | (0xFUL << (4UL * mod->sdaPin));
    sclData  = (volatile uint32 *)(mod->gpioBaseAddr + (sclMask << 2));
    sdaData  = (volatile uint32 *)(mod->gpioBaseAddr + (sdaMask << 2));

    /* MANUAL STOP over GPIO, BEFORE re-muxing (the wedged hardware FSM may never
     * emit one): SDA low -> SCL high -> SDA high, i.e. SDA rising while SCL is
     * high = STOP.
     *
     * FIX 2 - SDA (and SCL) stay OPEN-DRAIN throughout: re-assert ODR on both
     * before switching SDA to output, so `*sdaData=0` pulls SDA low and
     * `*sdaData=sdaMask` RELEASES SDA to the pull-up (the STOP rising edge) -
     * never a driven push-pull high against a slave that might still hold SDA
     * low. */
    *(mod->gpioOdr) |= (sclMask | sdaMask);         /* re-assert open-drain on both */
    *(mod->gpioDir) |= sdaMask;                     /* SDA -> output (open-drain)   */

    *sclData = 0UL;                                  /* SCL pull low  */
    I2C_DelayTicks(I2C_RECOVER_SCL_HALF_TICKS);
    *sdaData = 0UL;                                  /* SDA pull low  */
    I2C_DelayTicks(I2C_RECOVER_SCL_HALF_TICKS);
    *sclData = sclMask;                             /* SCL RELEASE high (RC rise) */
    /* This inter-write delay MUST be >= the SCL pull-up rise time: SCL has to
     * actually reach high BEFORE SDA is released, or the SDA rising edge does
     * not land inside an SCL-high window and no valid STOP is generated. */
    I2C_DelayTicks(I2C_RECOVER_SCL_HALF_TICKS);
    *sdaData = sdaMask;                             /* SDA RELEASE high while SCL high = STOP */
    I2C_DelayTicks(I2C_RECOVER_SCL_HALF_TICKS);

    /* Restore the exact prior AF/GPIO config (active==TRUE is guaranteed by the
     * guard above). ODR (SDA open-drain) is restored here because the PORT layer
     * has no ODR mechanism. AFSEL restored LAST so PCTL/ODR/DIR are already
     * correct when the pins flip back to the I2C AF. */
    *(mod->gpioDir)   = (*(mod->gpioDir)   & ~pinMask)  | g_I2C_RecoverSave[i2cId].dir;
    *(mod->gpioOdr)   = (*(mod->gpioOdr)   & ~pinMask)  | g_I2C_RecoverSave[i2cId].odr;
    *(mod->gpioPctl)  = (*(mod->gpioPctl)  & ~pctlMask) | g_I2C_RecoverSave[i2cId].pctl;
    *(mod->gpioAfsel) = (*(mod->gpioAfsel) & ~pinMask)  | g_I2C_RecoverSave[i2cId].afsel;
    g_I2C_RecoverSave[i2cId].active = FALSE;

    /* Re-init the peripheral FSM clean (TIMER0-timed, no NOP loops). */
    status = I2C_Reset(i2cId);
    if (status != I2C_OK) { return status; }

    /* Verify: BUSBSY clear = bus idle/recovered; still set = hardware fault
     * beyond SCL recovery -> HAL escalates to permanent FAULT. */
    if ((*(mod->mcs) & I2C_MCS_BUSBSY) != 0U) { return I2C_ERROR_BUS_STUCK; }
    return I2C_OK;
}

I2C_StatusType I2C_Reset(I2C_IdType i2cId)
{
    if (i2cId >= I2C_ID_MAX) { return I2C_ERROR_INVALID_ID; }

    /* 1. Assert peripheral reset for this I2C module. */
    SYSCTL_SRI2C_REG |= (1UL << (uint32)i2cId);

    /* Let the reset propagate - TIMER0-timed (a few clocks per datasheet), NOT
     * a NOP loop: RecoverEnd runs this on the recovery path, which must contain
     * no -O-dependent NOP delays. */
    I2C_DelayTicks(I2C_RESET_SETTLE_TICKS);

    /* 2. Deassert peripheral reset. */
    SYSCTL_SRI2C_REG &= ~(1UL << (uint32)i2cId);

    /* 3. Re-initialize the peripheral using the cached configuration. NOTE: the
     * old blocking bit-bang I2C_RecoverBus() call (audit §3, ~6-7 ms of
     * for(delay<1000) loops) was REMOVED. SCL-toggle recovery is now the
     * caller's job via the TIMER0-capped I2C_RecoverBegin/Step/End primitives,
     * so I2C_Reset is pure peripheral re-init with no NOP-loop delays. */
    return I2C_InitWithConfig(i2cId, &g_I2C_Configs[i2cId]);
}
