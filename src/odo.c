/******************************************************************************
 *
 * Module: Odo (service)
 *
 * File Name: odo.c
 *
 * Description: Lifetime odometer over a wear-levelled EEPROM ring.
 *              See odo.h for the ODO-vs-TRIP distinction and odo_cfg.h for the
 *              endurance arithmetic behind the write policy.
 *
 ******************************************************************************/

#include "odo.h"
#include "eeprom.h"

/*******************************************************************************
 *                          Module State                                       *
 *******************************************************************************/

/* Whole metres, saturating. Kept as an integer, not a float: at 16 777 215 a
 * float32's ULP is 1.0, so accumulating small increments into a float total
 * would silently stop advancing near the ceiling. The fractional remainder
 * lives in its own accumulator below. */
static uint32  g_Odo_Metres      = 0UL;

/* Sub-metre remainder. Increments arrive as ~0.05 m at 10 Hz, so throwing away
 * the fraction on every sample would lose most of the distance. */
static float32 g_Odo_Fraction    = 0.0f;

/* Metres travelled since the last EEPROM save was STARTED. */
static float32 g_Odo_SinceSave   = 0.0f;

static boolean g_Odo_Persistent  = FALSE;
static uint32  g_Odo_Sequence    = 0UL;   /* sequence of the last saved entry   */
static uint8   g_Odo_NextSlot    = 0U;    /* ring slot the next save will use   */
static uint32  g_Odo_SaveCount   = 0UL;

/* Non-blocking save state machine. */
typedef enum
{
    ODO_SAVE_IDLE = 0,      /* nothing in flight                                */
    ODO_SAVE_DUE,           /* interval reached, waiting for the bus to be free  */
    ODO_SAVE_ODO_ISSUED,    /* word[0] (metres) written, awaiting completion     */
    ODO_SAVE_SEQ_ISSUED     /* word[1] (magic|seq) written, awaiting completion  */
} Odo_SaveStateType;

static Odo_SaveStateType g_Odo_SaveState = ODO_SAVE_IDLE;
static uint32 g_Odo_PendingMetres = 0UL;  /* snapshot saved by the current cycle */
static uint32 g_Odo_PendingSeq    = 0UL;

/*******************************************************************************
 *                          Ring helpers                                       *
 *******************************************************************************/

static uint16 Odo_EntryWord(uint8 slot, uint8 word)
{
    return (uint16)((uint16)ODO_RING_BASE_WORD +
                    ((uint16)slot * (uint16)ODO_WORDS_PER_ENTRY) + (uint16)word);
}

static boolean Odo_SeqWordValid(uint32 seqWord)
{
    return (((seqWord >> ODO_SEQ_MAGIC_SHIFT) & 0xFFUL) == ODO_SEQ_MAGIC) ? TRUE : FALSE;
}

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

Std_ReturnType Odo_Init(void)
{
    uint8   slot;
    boolean anyValid = FALSE;
    uint32  bestSeq  = 0UL;
    uint8   bestSlot = 0U;

    g_Odo_Metres     = 0UL;
    g_Odo_Fraction   = 0.0f;
    g_Odo_SinceSave  = 0.0f;
    g_Odo_Sequence   = 0UL;
    g_Odo_NextSlot   = 0U;
    g_Odo_SaveCount  = 0UL;
    g_Odo_SaveState  = ODO_SAVE_IDLE;
    g_Odo_Persistent = FALSE;

    if (Eeprom_Init() != E_OK)
    {
        /* RAM-only from here. Not an error the vehicle should care about: the
         * odometer keeps counting, it just will not survive the next reset. */
        return E_NOT_OK;
    }

    /* Scan every slot and take the highest VALID sequence. The sequence is
     * 24-bit and cannot wrap within the device's endurance (odo_cfg.h), so a
     * plain comparison is correct - no modular ordering needed. */
    for (slot = 0U; slot < (uint8)ODO_RING_ENTRIES; slot++)
    {
        uint32 metres  = 0UL;
        uint32 seqWord = 0UL;

        if (Eeprom_ReadWord(Odo_EntryWord(slot, 0U), &metres)  != E_OK) { continue; }
        if (Eeprom_ReadWord(Odo_EntryWord(slot, 1U), &seqWord) != E_OK) { continue; }

        if (Odo_SeqWordValid(seqWord) == FALSE)
        {
            continue;               /* blank or corrupt - never written */
        }

        {
            uint32 seq = seqWord & ODO_SEQ_MASK;
            if ((anyValid == FALSE) || (seq > bestSeq))
            {
                anyValid = TRUE;
                bestSeq  = seq;
                bestSlot = slot;
                g_Odo_Metres = (metres > ODO_MAX_M) ? ODO_MAX_M : metres;
            }
        }
    }

    if (anyValid != FALSE)
    {
        g_Odo_Sequence = bestSeq;
        /* Resume the rotation AFTER the newest entry, so the next save lands on
         * the oldest slot and wear stays even. */
        g_Odo_NextSlot = (uint8)((bestSlot + 1U) % (uint8)ODO_RING_ENTRIES);
    }
    else
    {
        /* Blank device / first boot. Starting at 0 is the correct answer, not a
         * failure - and it is distinguishable from "written zero" only because
         * of the magic (odo_cfg.h). */
        g_Odo_Metres   = 0UL;
        g_Odo_Sequence = 0UL;
        g_Odo_NextSlot = 0U;
    }

    g_Odo_Persistent = TRUE;
    return E_OK;
}

void Odo_AddMetres(float32 metres)
{
    /* An odometer measures PATH LENGTH: callers pass |delta| so that reversing
     * adds. A negative value means the caller got that wrong; ignore it rather
     * than letting the lifetime total run backwards. */
    if ((metres <= 0.0f) || (metres != metres))     /* also rejects NaN */
    {
        return;
    }

    g_Odo_Fraction  += metres;
    g_Odo_SinceSave += metres;

    /* Carry whole metres out of the fraction. A while-loop rather than a single
     * subtraction so an unusually large increment cannot leave >1 m stranded. */
    while (g_Odo_Fraction >= 1.0f)
    {
        g_Odo_Fraction -= 1.0f;
        if (g_Odo_Metres < ODO_MAX_M)
        {
            g_Odo_Metres++;
        }
        else
        {
            g_Odo_Fraction = 0.0f;      /* saturated - stop carrying */
            break;
        }
    }

    if ((g_Odo_SinceSave >= ODO_WRITE_INTERVAL_M) && (g_Odo_SaveState == ODO_SAVE_IDLE))
    {
        g_Odo_SaveState = ODO_SAVE_DUE;
    }
}

void Odo_MainFunction(void)
{
    if (g_Odo_Persistent == FALSE)
    {
        return;                         /* RAM-only: nothing to service */
    }

    /* NEVER SPIN. Each branch issues at most one word write and returns; the
     * hardware finishes in its own time and we look again next call. */
    if (Eeprom_IsBusy() != FALSE)
    {
        return;
    }

    switch (g_Odo_SaveState)
    {
        case ODO_SAVE_DUE:
            /* Snapshot what we are about to persist, so a distance increment
             * arriving mid-cycle cannot make word[0] and word[1] describe
             * different totals. */
            g_Odo_PendingMetres = g_Odo_Metres;
            g_Odo_PendingSeq    = g_Odo_Sequence + 1UL;

            if (Eeprom_WriteWordStart(Odo_EntryWord(g_Odo_NextSlot, 0U),
                                      g_Odo_PendingMetres) == E_OK)
            {
                g_Odo_SaveState = ODO_SAVE_ODO_ISSUED;
                g_Odo_SinceSave = 0.0f;   /* interval restarts when the save starts */
            }
            break;

        case ODO_SAVE_ODO_ISSUED:
            /* ⚠️ THE SEQUENCE WORD IS WRITTEN LAST, AND THAT IS WHAT MAKES THIS
             * TEAR-SAFE WITHOUT A CHECKSUM.
             *
             * If power fails between the two words, this slot ends up holding a
             * NEW odometer value against its OLD sequence. That slot is the one
             * the rotation just reached, i.e. the OLDEST - so its sequence is
             * the MINIMUM in the ring, and Odo_Init's "highest sequence wins"
             * scan can never select it. The half-written entry is inert and is
             * overwritten on the next pass. The cost of the lost write is
             * exactly the documented worst case: one interval, 10 m. */
            if (Eeprom_WriteWordStart(Odo_EntryWord(g_Odo_NextSlot, 1U),
                                      (ODO_SEQ_MAGIC << ODO_SEQ_MAGIC_SHIFT) |
                                      (g_Odo_PendingSeq & ODO_SEQ_MASK)) == E_OK)
            {
                g_Odo_SaveState = ODO_SAVE_SEQ_ISSUED;
            }
            break;

        case ODO_SAVE_SEQ_ISSUED:
            /* Both words are down and the hardware is idle: the entry is
             * committed. Only now advance the rotation and the sequence. */
            g_Odo_Sequence = g_Odo_PendingSeq;
            g_Odo_NextSlot = (uint8)((g_Odo_NextSlot + 1U) % (uint8)ODO_RING_ENTRIES);
            if (g_Odo_SaveCount < 0xFFFFFFFFUL)
            {
                g_Odo_SaveCount++;
            }
            g_Odo_SaveState = ODO_SAVE_IDLE;
            break;

        case ODO_SAVE_IDLE:
        default:
            break;
    }
}

uint32  Odo_GetMetres(void)     { return g_Odo_Metres;     }
boolean Odo_IsPersistent(void)  { return g_Odo_Persistent; }
uint32  Odo_GetSaveCount(void)  { return g_Odo_SaveCount;  }
uint8   Odo_GetNextSlot(void)   { return g_Odo_NextSlot;   }
