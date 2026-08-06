/******************************************************************************
 *
 * Module: EEPROM (MCAL)
 *
 * File Name: eeprom_private.h
 *
 * Description: Register offsets and bit definitions for the TM4C123GH6PM
 *              internal EEPROM. Included by eeprom.c only.
 *
 *              The EEPROM peripheral block (base 0x400AF000) is NOT in
 *              tm4c123gh6pm_registers.h - only the SYSCTL clock-gate/reset
 *              registers for it are - so the block's own registers are defined
 *              here. Datasheet chapter 8 (Internal Memory), section 8.2.4.
 *
 *              Geometry: 2 KB total = 512 words of 32 bits, organised as
 *              32 blocks x 16 words. Word-addressable, NO erase-before-write,
 *              ~500 000 write cycles per word.
 *
 ******************************************************************************/

#ifndef EEPROM_PRIVATE_H_
#define EEPROM_PRIVATE_H_

#include "Platform_Types.h"

/*******************************************************************************
 *                  EEPROM peripheral block - datasheet 8.5 / p.549-570        *
 *******************************************************************************/

#define EEPROM_PRIV_BASE            (0x400AF000UL)

#define EEPROM_EESIZE_REG           (*((volatile uint32 *)(EEPROM_PRIV_BASE + 0x000UL)))
#define EEPROM_EEBLOCK_REG          (*((volatile uint32 *)(EEPROM_PRIV_BASE + 0x004UL)))
#define EEPROM_EEOFFSET_REG         (*((volatile uint32 *)(EEPROM_PRIV_BASE + 0x008UL)))
#define EEPROM_EERDWR_REG           (*((volatile uint32 *)(EEPROM_PRIV_BASE + 0x010UL)))
#define EEPROM_EERDWRINC_REG        (*((volatile uint32 *)(EEPROM_PRIV_BASE + 0x014UL)))
#define EEPROM_EEDONE_REG           (*((volatile uint32 *)(EEPROM_PRIV_BASE + 0x018UL)))
#define EEPROM_EESUPP_REG           (*((volatile uint32 *)(EEPROM_PRIV_BASE + 0x01CUL)))

/* EEDONE - EEPROM Done Status, offset 0x018, p.560.
 * WORKING (bit 0) is set while an operation is in progress. The error bits are
 * sticky for the last operation. */
#define EEPROM_PRIV_EEDONE_WORKING  (0x00000001UL)  /* b0 operation in progress   */
#define EEPROM_PRIV_EEDONE_WKERASE  (0x00000004UL)  /* b2 erase required, failed  */
#define EEPROM_PRIV_EEDONE_WKCOPY   (0x00000008UL)  /* b3 copy required, failed   */
#define EEPROM_PRIV_EEDONE_NOPERM   (0x00000010UL)  /* b4 write without permission*/
#define EEPROM_PRIV_EEDONE_WRBUSY   (0x00000020UL)  /* b5 write busy              */
#define EEPROM_PRIV_EEDONE_ERR_MASK (EEPROM_PRIV_EEDONE_WKERASE | \
                                     EEPROM_PRIV_EEDONE_WKCOPY  | \
                                     EEPROM_PRIV_EEDONE_NOPERM  | \
                                     EEPROM_PRIV_EEDONE_WRBUSY)

/* EESUPP - EEPROM Support Control and Status, offset 0x01C, p.561.
 * PRETRY/ERETRY set after reset mean the EEPROM's internal recovery did not
 * complete: the block is UNUSABLE and must not be written. */
#define EEPROM_PRIV_EESUPP_ERETRY   (0x00000004UL)  /* b2 erase must be retried   */
#define EEPROM_PRIV_EESUPP_PRETRY   (0x00000008UL)  /* b3 program must be retried */
#define EEPROM_PRIV_EESUPP_FAIL     (EEPROM_PRIV_EESUPP_ERETRY | EEPROM_PRIV_EESUPP_PRETRY)

/*******************************************************************************
 *                  SYSCTL bits this driver touches                            *
 *******************************************************************************/

#define EEPROM_PRIV_RCGCEEPROM_MASK (0x00000001UL)  /* SYSCTL_RCGCEEPROM bit 0    */
#define EEPROM_PRIV_PREEPROM_MASK   (0x00000001UL)  /* SYSCTL_PREEPROM  bit 0     */
#define EEPROM_PRIV_SREEPROM_MASK   (0x00000001UL)  /* SYSCTL_SREEPROM  bit 0     */

/*******************************************************************************
 *                  Geometry + bounds                                          *
 *******************************************************************************/

#define EEPROM_PRIV_WORDS_PER_BLOCK (16U)
#define EEPROM_PRIV_NUM_BLOCKS      (32U)
#define EEPROM_PRIV_TOTAL_WORDS     (EEPROM_PRIV_WORDS_PER_BLOCK * EEPROM_PRIV_NUM_BLOCKS) /* 512 */

/* Bounded spin cap for the WORKING poll and the post-reset settle. Same
 * discipline as TIMER0_INIT_SPIN_CAP: a dead peripheral must fail fast, never
 * hang. Generous - a worst-case EEPROM word program (one that triggers an
 * internal copy/erase of the 2 KB block) is specified at up to ~10 ms, and this
 * cap is ~ms-class even at -O0, so it only expires if the block is truly dead.
 *
 * ⚠️ THIS CAP IS ONLY USED DURING Eeprom_Init() AND Eeprom_ReadWord(), BOTH OF
 * WHICH RUN AT BOOT. The runtime WRITE path never spins - see eeprom.h. */
#define EEPROM_PRIV_SPIN_CAP        (2000000UL)

/* Post-clock-enable settle. The datasheet requires a delay after setting
 * RCGCEEPROM before the block's registers may be touched (8.2.4.1 step 2). */
#define EEPROM_PRIV_SETTLE_LOOPS    (100U)

#endif /* EEPROM_PRIVATE_H_ */
