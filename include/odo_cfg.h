/******************************************************************************
 *
 * Module: Odo (service)
 *
 * File Name: odo_cfg.h
 *
 * Description: Compile-time configuration for the persistent odometer.
 *
 ******************************************************************************/

#ifndef ODO_CFG_H_
#define ODO_CFG_H_

#include "Platform_Types.h"

/*-----------------------------------------------------------------------------
 *  WRITE POLICY - the endurance calculation, stated so it can be re-checked
 *
 *  The TM4C123 EEPROM is rated ~500 000 write cycles PER WORD. Writing the
 *  odometer on every update would destroy a word in minutes, so writes are
 *  throttled by DISTANCE and spread across a ring of slots:
 *
 *      10 m per write  x  500 000 cycles              =  5 000 km per slot
 *      5 000 km        x  ODO_RING_ENTRIES (8)        = 40 000 km lifetime
 *
 *  40 000 km is decades for a test robot that moves at ~0.5 m/s, and it is
 *  reached only if the vehicle actually drives that far - the counter advances
 *  with distance, not with time, so a parked robot writes nothing at all.
 *
 *  The cost of the throttle is the WORST-CASE LOSS ON SUDDEN POWER-OFF: up to
 *  one write interval, i.e. 10 m. That is the deliberate trade.
 *---------------------------------------------------------------------------*/

/** Distance travelled between EEPROM writes, metres. */
#define ODO_WRITE_INTERVAL_M        (10.0f)

/** Slots in the wear-levelling ring. Each write rotates to the next slot.
 *  MUST be < 256 for the newest-entry search to be unambiguous (see odo.c). */
#define ODO_RING_ENTRIES            (8U)

/** Words per ring entry: [0] = odometer metres, [1] = magic|sequence. */
#define ODO_WORDS_PER_ENTRY         (2U)

/** First EEPROM word used by the ring. The ring occupies
 *  ODO_RING_ENTRIES * ODO_WORDS_PER_ENTRY = 16 words (64 bytes) of the 512
 *  available, leaving the rest of the EEPROM free for future use. */
#define ODO_RING_BASE_WORD          (0U)

/*-----------------------------------------------------------------------------
 *  ENTRY FORMAT (one 32-bit word each)
 *
 *  word[0] = odometer, metres, 0 .. ODO_MAX_M
 *  word[1] = (ODO_SEQ_MAGIC << 24) | (sequence & 0x00FFFFFF)
 *
 *  ⚠️ THE MAGIC IS LOAD-BEARING, not decoration. A blank/erased EEPROM word
 *  reads as an all-ones or all-zeros pattern, and BOTH are otherwise legal
 *  entry values (sequence 0 with odometer 0 is exactly what a first boot should
 *  look like). Without a magic, a blank device is indistinguishable from a
 *  valid entry and the ring cannot tell "never written" from "written zero".
 *
 *  ⚠️ THE SEQUENCE IS 24-BIT SO IT CANNOT WRAP WITHIN THE DEVICE'S LIFE:
 *  16 777 216 writes x 10 m = 167 000 km, which is four times the 40 000 km
 *  endurance ceiling above. The ring therefore never has to reason about
 *  modular sequence comparison - a plain > works.
 *---------------------------------------------------------------------------*/
#define ODO_SEQ_MAGIC               (0xA5UL)
#define ODO_SEQ_MAGIC_SHIFT         (24U)
#define ODO_SEQ_MASK                (0x00FFFFFFUL)

/** Odometer ceiling. EXACTLY the 24-bit DBC `odo_m` signal maximum, so the
 *  service and the wire saturate at the same value and the frame can never
 *  carry a wrapped number. 16 777 215 m = 16 777 km. */
#define ODO_MAX_M                   (16777215UL)

#endif /* ODO_CFG_H_ */
