#ifndef I2C_H_
#define I2C_H_

#include "i2c_types.h"
#include "i2c_cfg.h"
#include "timer0.h"     /* TIMER0_US_TO_TICKS - the per-command timeout is a raw
                         * TIMER0 tick cap passed in by the caller (see below). */

/*******************************************************************************
 *                    Per-command timeout budget (caller-owned)               *
 *                                                                             *
 * I2C_Write/I2C_Read take a `cap_ticks` argument: the maximum TIMER0 ticks a  *
 * SINGLE hardware command (one byte / START / STOP) may spend waiting for     *
 * BUSY to clear before returning I2C_ERROR_TIMEOUT. It is a RAW tick count -  *
 * the MCAL never converts to microseconds and never decides policy; the       *
 * caller owns the budget. A multi-byte transfer applies this cap PER command  *
 * (each byte gets its own fresh budget), matching the MCAL's per-command      *
 * granularity.                                                                *
 *                                                                             *
 * Contract:                                                                    *
 *   - cap_ticks MUST be > 0. A value of 0 makes every command time out        *
 *     immediately (loud, obvious failure - not a silent hang); the MCAL does  *
 *     NOT clamp it, because clamping would be the transport second-guessing a  *
 *     caller-owned budget. Pass one of the named constants below (or an        *
 *     RT-slot constant), never a bare literal.                                *
 *   - cap_ticks MUST stay < 2^31 so the wrap-safe unsigned compare in         *
 *     Timer0_ElapsedTicks() is unambiguous. At microsecond scale this is       *
 *     never approached (I2C_CAP_DEFAULT_TICKS = 3200 << 2^31).                *
 *   - REQUIRES Timer0_FreeRunning_Init() to have run (and RETURN E_OK) before *
 *     I2C_Init(). This is the one invariant the caller's startup order MUST   *
 *     guarantee - but it is now ENFORCED, not merely documented (T25-1):      *
 *       * I2C_InitWithConfig() checks Timer0_IsRunning() and returns          *
 *         I2C_ERROR_NOT_INITIALIZED on a dead timebase, so the module refuses *
 *         to come up rather than coming up unbounded; and                     *
 *       * Timer0_ElapsedTicks() returns TIMER0_ELAPSED_INVALID when the       *
 *         counter is not running, which trips any cap immediately.            *
 *     Previously a stopped TIMER0 froze GPTMTAR, elapsed stayed 0, the cap    *
 *     never elapsed, and every wait here hung forever - silently.             *
 *******************************************************************************/

/* Provisional healthy-command budget for non-RT callers (bench + general use).
 * 200 us ~= 4.4x a healthy ~45 us single-byte command at 400 kHz. PROVISIONAL:
 * retune this one named constant after a real healthy-command measurement - do
 * not scatter the literal at call sites. */
#define I2C_CAP_PROVISIONAL_US    (200U)
#define I2C_CAP_DEFAULT_TICKS     TIMER0_US_TO_TICKS(I2C_CAP_PROVISIONAL_US)  /* = 3200 */

/*******************************************************************************
 *                          Function Prototypes                                *
 *******************************************************************************/

/**
 * @brief Initialize all enabled I2C modules configured in i2c_cfg.h
 */
I2C_StatusType I2C_Init(void);

/**
 * @brief Initialize a specific I2C module with custom configuration
 */
I2C_StatusType I2C_InitWithConfig(I2C_IdType i2cId, const I2C_ConfigType *config);

/**
 * @brief Write data to an I2C slave device
 * @param i2cId: ID of the I2C module
 * @param slaveAddr: 7-bit slave address
 * @param regAddr: Internal register address to write to
 * @param data: Pointer to data buffer to write
 * @param length: Number of bytes to write
 * @param cap_ticks: per-command TIMER0-tick timeout budget (see header block;
 *        pass I2C_CAP_DEFAULT_TICKS unless an RT slot supplies its own)
 */
I2C_StatusType I2C_Write(I2C_IdType i2cId, uint8 slaveAddr, uint8 regAddr, const uint8 *data, uint16 length, uint32 cap_ticks);

/**
 * @brief Read data from an I2C slave device. Register-addressed: writes
 *        regAddr, then reads `length` bytes using a TRUE repeated-START
 *        (no STOP between the address phase and the data phase) - required
 *        for EEPROM random-address reads and to avoid returning a stale
 *        INA226 register on the next poll. length==1 uses a plain
 *        repeated-START + single-byte receive; length>1 uses a burst
 *        receive (first byte ACKed, middle bytes ACKed, last byte NACKed
 *        then STOPped) - see the sequencing comment in I2C_Read()'s
 *        implementation (src/i2c.c) for the exact command values.
 * @param i2cId: ID of the I2C module
 * @param slaveAddr: 7-bit slave address
 * @param regAddr: Internal register address to read from
 * @param data: Pointer to data buffer to store received bytes
 * @param length: Number of bytes to read (must be >= 1)
 * @param cap_ticks: per-command TIMER0-tick timeout budget (see header block;
 *        pass I2C_CAP_DEFAULT_TICKS unless an RT slot supplies its own)
 */
I2C_StatusType I2C_Read(I2C_IdType i2cId, uint8 slaveAddr, uint8 regAddr, uint8 *data, uint16 length, uint32 cap_ticks);

/**
 * @brief Forcefully resets and re-initializes the I2C peripheral hardware
 * @param i2cId: ID of the I2C module
 */
I2C_StatusType I2C_Reset(I2C_IdType i2cId);

/*******************************************************************************
 *              Bus-recovery primitives (HAL-sequenced, §3 closure)           *
 *                                                                             *
 * When a command returns I2C_ERROR_BUS_STUCK, auto-abort could not release    *
 * the bus (a wedged slave is holding SDA low). These three primitives are the *
 * electrical mechanics of the classic SCL-toggle recovery; the MCAL owns the  *
 * mechanics, the HAL owns ALL policy (how many steps, how many retries, when  *
 * to give up). They are transport/GPIO only - no app concepts.                *
 *                                                                             *
 * DISJOINT ERROR CLASSES (contract - do not blur):                            *
 *   - I2C_ERROR_BUS_STUCK is the ONLY condition that may enter these          *
 *     primitives. It means SDA is being held low - SCL toggling can clock a   *
 *     wedged slave into releasing it.                                         *
 *   - I2C_ERROR_TIMEOUT means auto-abort already asserted STOP and BUSBSY     *
 *     cleared: the bus is electrically CLEAN. A (repeated) TIMEOUT is NOT an  *
 *     SDA-stuck condition (it is clock-stretch beyond cap, too-tight cap, or  *
 *     a dead/disconnected slave). SCL toggling cannot revive a dead slave, so *
 *     a TIMEOUT must NEVER be escalated into this recovery path - the HAL     *
 *     soft-retries it (optionally with a widened cap) and otherwise FAULTs.   *
 *                                                                             *
 * Every wait inside them is TIMER0-capped/timed; there are NO NOP-loop delays *
 * and NO unbounded spins. REQUIRES Timer0 to be running (same dependency as   *
 * the command timeout). During recovery SCL and SDA are OPEN-DRAIN: a line    *
 * "high" is a pull-up release (RC rise), never a driven edge (no contention   *
 * against a slave holding the line low).                                      *
 *                                                                             *
 * HAL drives them ONLY on I2C_ERROR_BUS_STUCK (see the integration sketch):   *
 *   RecoverBegin(id);                                                          *
 *   for (n=0; n<I2C_RECOVER_MAX_STEPS; n++) if (RecoverStep(id)) break;        *
 *   status = RecoverEnd(id);   // I2C_OK = bus idle, else I2C_ERROR_BUS_STUCK  *
 *******************************************************************************/

/* Worst-case SCL pulses to clock a stuck slave free: 8 data bits + 1 ACK slot.
 * This is the HAL's step CAP, not a fixed count - the HAL stops early the first
 * time RecoverStep() reports SDA released. */
#define I2C_RECOVER_MAX_STEPS   (9U)

/**
 * @brief  Take PB2(SCL)/PB3(SDA) off the I2C alternate function to GPIO and set
 *         up bit-bang recovery: SCL as an OPEN-DRAIN output (idle released high
 *         via the pull-up, never actively driven - avoids contention with a
 *         clock-stretching slave), SDA as an INPUT (observed, never driven - the
 *         point is to detect when the slave releases it; driving SDA defeats
 *         recovery). Stashes the exact prior AFSEL/PCTL/DIR/ODR bits so
 *         RecoverEnd can restore them. No-op if a recovery is already active
 *         (double-Begin refused - the stash is not overwritten).
 * @param  i2cId: ID of the I2C module.
 */
void I2C_RecoverBegin(I2C_IdType i2cId);

/**
 * @brief  Emit ONE SCL pulse (high -> settle -> low -> settle -> high, all
 *         TIMER0-timed >= 400 kHz half-period) and sample SDA afterwards.
 * @param  i2cId: ID of the I2C module.
 * @return SDA line state after the pulse: 1 = released (high, bus freeing),
 *         0 = still held low. HAL stops pulsing the first time this returns 1.
 */
uint8 I2C_RecoverStep(I2C_IdType i2cId);

/**
 * @brief  Emit a MANUAL STOP over GPIO (SDA low -> SCL high -> SDA high, i.e.
 *         SDA rising while SCL high) BEFORE re-muxing - the wedged hardware FSM
 *         may never emit one. Then restore PB2/PB3 to the I2C AF (incl. SDA
 *         open-drain ODR, which the PORT layer cannot set) and I2C_Reset() the
 *         peripheral FSM.
 * @param  i2cId: ID of the I2C module.
 * @return I2C_OK if the bus is idle afterwards (BUSBSY clear); otherwise
 *         I2C_ERROR_BUS_STUCK (hardware fault beyond SCL recovery - HAL
 *         escalates to a permanent FAULT).
 */
I2C_StatusType I2C_RecoverEnd(I2C_IdType i2cId);

#endif /* I2C_H_ */
