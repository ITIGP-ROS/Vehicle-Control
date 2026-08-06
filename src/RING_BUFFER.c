/*****************************************************************************
 * RING_BUFFER.c — SPSC Lock-Free Ring Buffer Implementation
 *                  TM4C123GH6PM CAN0 RX FIFO (Can_FrameType)
 *
 * ═══════════════════════════════════════════════════════════════════════
 * SPSC INVARIANT — CRITICAL
 * ═══════════════════════════════════════════════════════════════════════
 *
 * RB-C01: HEAD written ONLY by producer (RingBuffer_Push / CAN0 ISR).
 *         TAIL written ONLY by consumer (RingBuffer_Pop / main loop).
 *         No other code may write head or tail.
 *
 * RB-C07: DMB instruction on Cortex-M4:
 *         __asm volatile ("dmb" ::: "memory");
 *         The "memory" clobber prevents the COMPILER from reordering.
 *         The "dmb" instruction prevents the CPU from reordering.
 *         Both are needed — compiler barrier alone is insufficient.
 *
 * RB-C08: No interrupt disabling anywhere in this module.
 *
 *****************************************************************************/

#include "RING_BUFFER.h"

/* ================================================================
 *  Internal Storage Struct — NOT exposed in header
 * ================================================================ */

/* volatile on head and tail:
 *   head is written by ISR (producer), read by main (consumer).
 *   tail is written by main (consumer), read by ISR (producer).
 *   Both MUST be volatile to prevent the compiler from caching
 *   their values in a register across loop iterations.
 *
 * buf[] is NOT volatile:
 *   Whole-element struct assignment is used; the volatile head/tail
 *   indices are the synchronisation mechanism, not the data array.  */
typedef struct
{
    Can_FrameType     buf[RING_BUFFER_CAPACITY];
    volatile uint32   head;   /* write index — producer only */
    volatile uint32   tail;   /* read  index — consumer only */
} RingBuffer_t;

/* ================================================================
 *  Static State — all file-scope, no dynamic allocation
 * ================================================================ */

/* Single global ring buffer instance (RX only).                   */
static RingBuffer_t ring_buffer_instance;

/* ================================================================
 *  Lifetime Statistics — never cleared, saturate at max
 * ================================================================ */

/* rb_push_count / rb_drop_count: written only by producer (ISR).
 * rb_pop_count:                   written only by consumer (main).
 * rb_max_fill:                    written only by producer (ISR).
 * Each is single-writer. volatile for reliable cross-context reads. */
static volatile uint32 rb_push_count = 0U;
static volatile uint32 rb_pop_count  = 0U;
static volatile uint32 rb_drop_count = 0U;
static volatile uint32 rb_max_fill   = 0U;   /* max(head-tail) ever observed */

/* ================================================================
 *  RingBuffer_Init
 * ================================================================ */

RingBuffer_Error_t RingBuffer_Init(void)
{
    uint32 i;
    Can_FrameType zero = { 0U, 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U } };

    /* ---- Reset head and tail indices ---- */
    ring_buffer_instance.head = 0U;
    ring_buffer_instance.tail = 0U;

    /* ---- Clear all slots (struct assignment) ---- */
    for (i = 0U; i < RING_BUFFER_CAPACITY; i++)
    {
        ring_buffer_instance.buf[i] = zero;
    }

    /* ---- Reset diagnostic counters ---- */
    rb_push_count = 0U;
    rb_pop_count  = 0U;
    rb_drop_count = 0U;
    rb_max_fill   = 0U;

    return RING_BUFFER_OK;
}

/* ================================================================
 *  RingBuffer_Push  (PRODUCER — ISR context)
 *
 *  RB-C05 sequence:
 *    1. NULL check
 *    2. Read local copies of head and tail
 *    3. Full check via subtraction (RB-C02)
 *    4. Whole-struct copy into buf[head & MASK]
 *    5. DMB barrier (RB-C07) — guarantees data visible before head++
 *    6. Increment head — makes slot visible to consumer
 *    7. Increment push count (saturating)
 *    8. Update max fill tracker
 * ================================================================ */

RingBuffer_Error_t RingBuffer_Push(const Can_FrameType *frame)
{
    uint32 local_head;
    uint32 local_tail;
    uint32 idx;
    uint32 fill;

    /* ---- 1) NULL check ---- */
    if (frame == NULL_PTR)
    {
        return RING_BUFFER_NULL_PTR;
    }

    /* ---- 2) Read local copies of head and tail ----
     *   local_head: owned by us (producer), reading our own index.
     *   local_tail: owned by consumer — volatile read gets latest value. */
    local_head = ring_buffer_instance.head;
    local_tail = ring_buffer_instance.tail;

    /* ---- 3) Full check (RB-C02) ----
     *   Subtraction-based: works correctly even when head wraps
     *   past UINT32_MAX because unsigned arithmetic wraps identically. */
    if ((local_head - local_tail) >= RING_BUFFER_CAPACITY)
    {
        /* Buffer full — DROP the newest, do not overwrite oldest, and
         * count it so the loss is VISIBLE. */
        if (rb_drop_count < 0xFFFFFFFFUL)
        {
            rb_drop_count++;
        }
        return RING_BUFFER_FULL;
    }

    /* ---- 4) Copy frame into buf[head & MASK] (buf[] is non-volatile) ---- */
    idx = local_head & RING_BUFFER_MASK;
    ring_buffer_instance.buf[idx] = *frame;

    /* ---- 5) DMB barrier (RB-C05, RB-C07) ----
     *   Ensures all buf[] writes are globally visible BEFORE head is
     *   incremented, so the consumer never reads a slot before its data. */
    __asm volatile ("dmb" ::: "memory");

    /* ---- 6) Increment head — makes slot visible to consumer ---- */
    ring_buffer_instance.head = local_head + 1U;

    /* ---- 7) Increment push count (saturating) ---- */
    if (rb_push_count < 0xFFFFFFFFUL)
    {
        rb_push_count++;
    }

    /* ---- 8) Update max fill tracker (producer side; single writer) ---- */
    fill = (local_head + 1U) - local_tail;
    if (fill > rb_max_fill)
    {
        rb_max_fill = fill;
    }

    return RING_BUFFER_OK;
}

/* ================================================================
 *  RingBuffer_Pop  (CONSUMER — main loop context)
 *
 *  RB-C06 sequence:
 *    1. NULL check
 *    2. Read local copies of head and tail
 *    3. Empty check (RB-C03)
 *    4. Whole-struct copy from buf[tail & MASK] into *frame
 *    5. DMB barrier (RB-C07) — guarantees read completes before tail++
 *    6. Increment tail — frees the slot for the producer
 *    7. Increment pop count (saturating)
 * ================================================================ */

RingBuffer_Error_t RingBuffer_Pop(Can_FrameType *frame)
{
    uint32 local_head;
    uint32 local_tail;
    uint32 idx;

    /* ---- 1) NULL check ---- */
    if (frame == NULL_PTR)
    {
        return RING_BUFFER_NULL_PTR;
    }

    /* ---- 2) Read local copies of head and tail ----
     *   local_tail: owned by us (consumer), reading our own index.
     *   local_head: owned by producer — volatile read gets latest value. */
    local_head = ring_buffer_instance.head;
    local_tail = ring_buffer_instance.tail;

    /* ---- 3) Empty check (RB-C03) ---- */
    if (local_head == local_tail)
    {
        return RING_BUFFER_EMPTY;
    }

    /* ---- 4) Copy frame from buf[tail & MASK] into *frame ---- */
    idx = local_tail & RING_BUFFER_MASK;
    *frame = ring_buffer_instance.buf[idx];

    /* ---- 5) DMB barrier (RB-C06, RB-C07) ----
     *   Ensures the buf[] read completes BEFORE tail is incremented, so
     *   the producer cannot overwrite the slot while we are still reading. */
    __asm volatile ("dmb" ::: "memory");

    /* ---- 6) Increment tail — frees the slot for the producer ---- */
    ring_buffer_instance.tail = local_tail + 1U;

    /* ---- 7) Increment pop count (saturating) ---- */
    if (rb_pop_count < 0xFFFFFFFFUL)
    {
        rb_pop_count++;
    }

    return RING_BUFFER_OK;
}

/* ================================================================
 *  Status Queries
 * ================================================================ */

uint32 RingBuffer_Count(void)
{
    return (ring_buffer_instance.head - ring_buffer_instance.tail);
}

uint8 RingBuffer_IsEmpty(void)
{
    uint8 result;

    if (ring_buffer_instance.head == ring_buffer_instance.tail)
    {
        result = 1U;
    }
    else
    {
        result = 0U;
    }

    return result;
}

uint8 RingBuffer_IsFull(void)
{
    uint8 result;

    if ((ring_buffer_instance.head - ring_buffer_instance.tail) >= RING_BUFFER_CAPACITY)
    {
        result = 1U;
    }
    else
    {
        result = 0U;
    }

    return result;
}

/* ================================================================
 *  Diagnostics
 * ================================================================ */

uint32 RingBuffer_GetPushCount(void)
{
    return rb_push_count;
}

uint32 RingBuffer_GetPopCount(void)
{
    return rb_pop_count;
}

uint32 RingBuffer_GetDropCount(void)
{
    return rb_drop_count;
}

uint32 RingBuffer_GetMaxFill(void)
{
    return rb_max_fill;
}
