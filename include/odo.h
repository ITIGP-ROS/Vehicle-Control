/******************************************************************************
 *
 * Module: Odo (service)
 *
 * File Name: odo.h
 *
 * Description: PERMANENT lifetime odometer, persisted in the TM4C's internal
 *              EEPROM so it survives resets and power cycles.
 *
 * -----------------------------------------------------------------------------
 *  ODO vs TRIP - two different things, deliberately
 * -----------------------------------------------------------------------------
 *    TRIP (cluster_comm, 0x200 trip_m, uint16)  RESET-RELATIVE. Zeroed by
 *         0x140 ResetCommand and by every power-cycle. Ceiling 65.5 km.
 *    ODO  (this module,  0x200 odo_m,  uint24)  LIFETIME. Survives resets and
 *         power cycles, and is NOT zeroed by ResetCommand - there is
 *         deliberately NO way to clear it over CAN. Ceiling 16 777 km.
 *
 *  Both accumulate TOTAL PATH LENGTH from the same |delta| samples, so
 *  reversing ADDS to both, exactly like a real odometer.
 * -----------------------------------------------------------------------------
 *
 *  ⚠️ DEGRADES, NEVER HALTS. If the EEPROM is missing or refuses to come up the
 *  odometer keeps counting IN RAM and simply stops being persistent -
 *  Odo_IsPersistent() reports FALSE. A dead EEPROM costs a saved number; it must
 *  never ground the vehicle.
 *
 ******************************************************************************/

#ifndef ODO_H_
#define ODO_H_

#include "Platform_Types.h"
#include "Std_Types.h"
#include "odo_cfg.h"

/**
 * @brief  Bring up the odometer: init the EEPROM, scan the wear-levelling ring
 *         and RESUME from the newest valid entry.
 * @return E_OK      persistent - a stored value was restored, or the ring was
 *                   blank and the odometer legitimately starts at 0.
 *         E_NOT_OK  the EEPROM is unusable. The odometer still works, in RAM
 *                   only, starting from 0.
 * @note   Call once at boot, BEFORE the first Odo_AddMetres().
 */
Std_ReturnType Odo_Init(void);

/**
 * @brief  Add a distance increment to the lifetime total.
 * @param  metres  a NON-NEGATIVE path-length increment. Callers pass |delta|:
 *                 an odometer measures distance travelled, not displacement, so
 *                 reversing must ADD. Negative input is ignored.
 * @note   Saturates at ODO_MAX_M - it never wraps. A wrapped odometer reads as
 *         a small plausible number, which is far worse than a stuck maximum.
 * @note   Does NOT touch the EEPROM. It only marks a write as due once
 *         ODO_WRITE_INTERVAL_M has accumulated; the write itself is issued by
 *         Odo_MainFunction().
 */
void Odo_AddMetres(float32 metres);

/**
 * @brief  Service the non-blocking EEPROM write state machine. Call from ONE
 *         super-loop slot.
 * @note   Issues at most one EEPROM word write per call and NEVER SPINS - a
 *         TM4C EEPROM program can take milliseconds when its copy buffer fills,
 *         which would blow the 1 ms slot budget. Completion is polled across
 *         later calls. See eeprom.h.
 */
void Odo_MainFunction(void);

/** @brief Lifetime distance in whole metres, saturating at ODO_MAX_M. This is
 *         the value packed into 0x200 odo_m. */
uint32 Odo_GetMetres(void);

/** @brief TRUE if the odometer is EEPROM-backed. FALSE means it is counting in
 *         RAM only and will be lost at the next reset. */
boolean Odo_IsPersistent(void);

/** @brief Completed EEPROM save cycles since boot. Useful to confirm the write
 *         throttle is behaving (expect one per ODO_WRITE_INTERVAL_M travelled). */
uint32 Odo_GetSaveCount(void);

/** @brief Ring slot the next save will use (0 .. ODO_RING_ENTRIES-1). Exposed
 *         so a test can watch the wear-levelling rotation. */
uint8 Odo_GetNextSlot(void);

#endif /* ODO_H_ */
