/******************************************************************************
 *
 * Module: EEPROM (MCAL)
 *
 * File Name: eeprom.h
 *
 * Description: Word access to the TM4C123GH6PM's 2 KB internal EEPROM.
 *
 *  Geometry: 512 words of 32 bits (32 blocks x 16 words). Word-addressable,
 *  NO erase-before-write, ~500 000 write cycles per word. This is a genuinely
 *  different memory from the program flash - writing it does not disturb code.
 *
 * -----------------------------------------------------------------------------
 *  ⚠️ THE WRITE PATH IS NON-BLOCKING, AND THAT IS THE WHOLE DESIGN
 * -----------------------------------------------------------------------------
 *  A TM4C EEPROM word program is NOT fast. In the common case it is tens of
 *  microseconds, but when the block's copy buffer fills, the hardware performs
 *  an internal copy-and-erase and the operation can take MILLISECONDS.
 *
 *  This node runs a 1 ms cooperative super-loop with CAN transmit slots. A
 *  blocking write would blow that budget outright - a multi-millisecond stall
 *  would delay or drop scheduled frames, and it would do so RARELY and
 *  UNPREDICTABLY (only when the copy buffer happens to fill), which is the
 *  worst possible failure signature to debug.
 *
 *  So the write is SPLIT: Eeprom_WriteWordStart() issues the operation and
 *  returns immediately, having spun on nothing, and Eeprom_IsBusy() reports
 *  completion. The caller polls across super-loop iterations instead of
 *  waiting. NOTHING in the runtime path ever spins on this peripheral.
 *
 *  Eeprom_Init() and Eeprom_ReadWord() DO use bounded spins - both run at boot,
 *  where blocking is acceptable and there is nothing to starve.
 * -----------------------------------------------------------------------------
 *
 ******************************************************************************/

#ifndef EEPROM_H_
#define EEPROM_H_

#include "Platform_Types.h"
#include "Std_Types.h"

/** Number of 32-bit words in the internal EEPROM (2 KB / 4). */
#define EEPROM_TOTAL_WORDS      (512U)

/**
 * @brief  Bring up the EEPROM block, including the mandatory post-reset
 *         recovery check.
 * @return E_OK      the block is present, recovered and usable.
 *         E_NOT_OK  the clock never came ready, a bounded wait expired, or the
 *                   block reported PRETRY/ERETRY - meaning its internal
 *                   recovery did NOT complete and it must not be written.
 *
 * @note   ⚠️ CHECK THE RETURN AND DEGRADE, DO NOT HALT. A dead EEPROM must not
 *         ground the vehicle - it costs a persistent odometer, nothing more.
 * @note   The PRETRY/ERETRY check is not optional book-keeping: those bits mean
 *         a program or erase was interrupted (typically by power loss mid-write)
 *         and the block is in an indeterminate state. Writing anyway risks
 *         corrupting neighbouring words.
 */
Std_ReturnType Eeprom_Init(void);

/** @brief TRUE once Eeprom_Init() has returned E_OK. */
boolean Eeprom_IsReady(void);

/**
 * @brief  Read one 32-bit word. Bounded-spins if an operation is in flight.
 * @param  wordIndex 0 .. EEPROM_TOTAL_WORDS-1
 * @param  out       destination, must not be NULL_PTR
 * @return E_OK, or E_NOT_OK on a bad index / NULL / not ready / wait expiry.
 * @note   Boot-path use. Do not call from a timing-critical slot.
 */
Std_ReturnType Eeprom_ReadWord(uint16 wordIndex, uint32 *out);

/**
 * @brief  ISSUE a word write and return IMMEDIATELY - does not wait for it.
 * @return E_OK      the write was accepted and is now in progress.
 *         E_NOT_OK  bad index, not ready, or a previous write is still running.
 * @note   Poll Eeprom_IsBusy() on later iterations; do not issue another
 *         operation until it reports FALSE. See the header note above for why
 *         this is split rather than blocking.
 */
Std_ReturnType Eeprom_WriteWordStart(uint16 wordIndex, uint32 value);

/** @brief TRUE while a write issued by Eeprom_WriteWordStart() is in progress. */
boolean Eeprom_IsBusy(void);

/**
 * @brief  Sticky error count: bounded-wait expiries plus hardware error bits
 *         (WKERASE/WKCOPY/NOPERM/WRBUSY) latched after a write.
 * @return 0 in a healthy system. Same "a guard that gives up COUNTS it" idiom
 *         as Can_GetIfTimeoutCount() and Timer0_GetInitExpiryCount().
 */
uint32 Eeprom_GetErrorCount(void);

#endif /* EEPROM_H_ */
