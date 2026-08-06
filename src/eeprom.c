/******************************************************************************
 *
 * Module: EEPROM (MCAL)
 *
 * File Name: eeprom.c
 *
 * Description: TM4C123GH6PM internal EEPROM word access. See eeprom.h for why
 *              the write path is non-blocking.
 *
 * Bring-up order per datasheet 8.2.4.1 ("EEPROM Initialization and
 * Configuration"), including the mandatory post-reset recovery pass:
 *   1. RCGCEEPROM bit0 = 1, then a settle delay before touching the block
 *   2. wait EEDONE.WORKING == 0        (internal power-on recovery)
 *   3. if EESUPP.PRETRY|ERETRY -> the block did NOT recover: FAIL, do not write
 *   4. software-reset the block via SREEPROM, settle again
 *   5. wait EEDONE.WORKING == 0
 *   6. re-check EESUPP -> still set means genuinely unusable: FAIL
 *
 * Steps 4-6 are not belt-and-braces. The EEPROM's own recovery runs on the
 * FIRST power-up after an interrupted program/erase, and the documented
 * sequence performs a reset-and-recheck so that a block which recovers on the
 * second pass is usable rather than being written off. Skipping them is the
 * classic way to get an EEPROM that "works until the first power cut".
 *
 ******************************************************************************/

#include "Platform_Types.h"
#include "tm4c123gh6pm_registers.h"
#include "eeprom.h"
#include "eeprom_private.h"

/*******************************************************************************
 *                          Module State                                       *
 *******************************************************************************/

static boolean g_Eeprom_Ready      = FALSE;
static uint32  g_Eeprom_ErrorCount = 0U;

static void Eeprom_CountError(void)
{
    if (g_Eeprom_ErrorCount < 0xFFFFFFFFUL)
    {
        g_Eeprom_ErrorCount++;
    }
}

/*---------------------------------------------------------------------------
 * Bounded wait for EEDONE.WORKING to clear. BOOT PATH ONLY - the runtime write
 * path never calls this (see eeprom.h).
 *-------------------------------------------------------------------------*/
static boolean Eeprom_WaitNotWorking(void)
{
    uint32 spins = EEPROM_PRIV_SPIN_CAP;

    while (((EEPROM_EEDONE_REG & EEPROM_PRIV_EEDONE_WORKING) != 0UL) && (spins > 0UL))
    {
        spins--;
    }

    if (spins == 0UL)
    {
        Eeprom_CountError();
        return FALSE;
    }
    return TRUE;
}

static void Eeprom_Settle(void)
{
    volatile uint32 i;
    for (i = 0U; i < EEPROM_PRIV_SETTLE_LOOPS; i++)
    {
        /* deliberate busy delay - boot only */
    }
}

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

Std_ReturnType Eeprom_Init(void)
{
    uint32 spins;

    g_Eeprom_Ready = FALSE;

    /* 1. Clock the block, then wait for it to report present, then settle. */
    SYSCTL_RCGCEEPROM_REG |= EEPROM_PRIV_RCGCEEPROM_MASK;

    spins = EEPROM_PRIV_SPIN_CAP;
    while (((SYSCTL_PREEPROM_REG & EEPROM_PRIV_PREEPROM_MASK) == 0UL) && (spins > 0UL))
    {
        spins--;
    }
    if (spins == 0UL)
    {
        Eeprom_CountError();
        return E_NOT_OK;            /* clock never came up - do not touch the block */
    }
    Eeprom_Settle();

    /* 2. Let the block finish its own power-on recovery. */
    if (Eeprom_WaitNotWorking() == FALSE)
    {
        return E_NOT_OK;
    }

    /* 3. First recovery verdict. */
    if ((EEPROM_EESUPP_REG & EEPROM_PRIV_EESUPP_FAIL) != 0UL)
    {
        /* Not fatal yet - the documented sequence resets and re-checks. */
        Eeprom_CountError();
    }

    /* 4. Software-reset the block and settle. */
    SYSCTL_SREEPROM_REG |= EEPROM_PRIV_SREEPROM_MASK;
    Eeprom_Settle();
    SYSCTL_SREEPROM_REG &= ~EEPROM_PRIV_SREEPROM_MASK;
    Eeprom_Settle();

    spins = EEPROM_PRIV_SPIN_CAP;
    while (((SYSCTL_PREEPROM_REG & EEPROM_PRIV_PREEPROM_MASK) == 0UL) && (spins > 0UL))
    {
        spins--;
    }
    if (spins == 0UL)
    {
        Eeprom_CountError();
        return E_NOT_OK;
    }
    Eeprom_Settle();

    /* 5. Wait again for the post-reset recovery. */
    if (Eeprom_WaitNotWorking() == FALSE)
    {
        return E_NOT_OK;
    }

    /* 6. Final verdict. Still set => the block is genuinely unusable; refuse to
     *    come up rather than writing into an indeterminate memory. */
    if ((EEPROM_EESUPP_REG & EEPROM_PRIV_EESUPP_FAIL) != 0UL)
    {
        Eeprom_CountError();
        return E_NOT_OK;
    }

    /* Sanity-check the geometry actually matches what this driver assumes.
     * EESIZE reports blocks in 31:16 and words-per-block in 15:0. A part that
     * disagrees would silently alias addresses. */
    {
        uint32 eesize = EEPROM_EESIZE_REG;
        uint32 blocks = (eesize >> 16) & 0xFFFFUL;
        uint32 words  =  eesize        & 0xFFFFUL;

        if ((blocks * words) < (uint32)EEPROM_TOTAL_WORDS)
        {
            Eeprom_CountError();
            return E_NOT_OK;
        }
    }

    g_Eeprom_Ready = TRUE;
    return E_OK;
}

boolean Eeprom_IsReady(void) { return g_Eeprom_Ready; }

boolean Eeprom_IsBusy(void)
{
    if (g_Eeprom_Ready == FALSE)
    {
        return FALSE;               /* nothing can be in flight */
    }
    return ((EEPROM_EEDONE_REG & EEPROM_PRIV_EEDONE_WORKING) != 0UL) ? TRUE : FALSE;
}

Std_ReturnType Eeprom_ReadWord(uint16 wordIndex, uint32 *out)
{
    if ((g_Eeprom_Ready == FALSE) || (out == NULL_PTR) ||
        (wordIndex >= (uint16)EEPROM_TOTAL_WORDS))
    {
        return E_NOT_OK;
    }

    /* A read must not overlap an in-flight write. Boot path, so a bounded spin
     * is acceptable here (unlike the write path). */
    if (Eeprom_WaitNotWorking() == FALSE)
    {
        return E_NOT_OK;
    }

    EEPROM_EEBLOCK_REG  = (uint32)(wordIndex / EEPROM_PRIV_WORDS_PER_BLOCK);
    EEPROM_EEOFFSET_REG = (uint32)(wordIndex % EEPROM_PRIV_WORDS_PER_BLOCK);
    *out = EEPROM_EERDWR_REG;

    return E_OK;
}

Std_ReturnType Eeprom_WriteWordStart(uint16 wordIndex, uint32 value)
{
    if ((g_Eeprom_Ready == FALSE) || (wordIndex >= (uint16)EEPROM_TOTAL_WORDS))
    {
        return E_NOT_OK;
    }

    /* DO NOT WAIT - refuse instead. This is the promise the header makes: this
     * function never spins. If a previous write is still running, the caller
     * retries on a later super-loop iteration. */
    if ((EEPROM_EEDONE_REG & EEPROM_PRIV_EEDONE_WORKING) != 0UL)
    {
        return E_NOT_OK;
    }

    /* Latch any error bits left by the PREVIOUS write before starting a new one
     * - EEDONE's error bits describe the last completed operation, so this is
     * the only moment they can be observed. */
    if ((EEPROM_EEDONE_REG & EEPROM_PRIV_EEDONE_ERR_MASK) != 0UL)
    {
        Eeprom_CountError();
    }

    EEPROM_EEBLOCK_REG  = (uint32)(wordIndex / EEPROM_PRIV_WORDS_PER_BLOCK);
    EEPROM_EEOFFSET_REG = (uint32)(wordIndex % EEPROM_PRIV_WORDS_PER_BLOCK);
    EEPROM_EERDWR_REG   = value;    /* this write STARTS the program operation */

    return E_OK;
}

uint32 Eeprom_GetErrorCount(void) { return g_Eeprom_ErrorCount; }
