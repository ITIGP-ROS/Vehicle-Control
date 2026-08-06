#ifndef RING_BUFFER_H
#define RING_BUFFER_H

/*****************************************************************************
 * RING_BUFFER.h — SPSC Lock-Free Ring Buffer for Can_FrameType
 *                  TM4C123GH6PM CAN0 RX FIFO
 *
 * ═══════════════════════════════════════════════════════════════════════
 * SPSC INVARIANT — READ THIS BEFORE MODIFYING ANYTHING
 * ═══════════════════════════════════════════════════════════════════════
 *
 * This ring buffer is Single-Producer Single-Consumer (SPSC):
 *   Producer: CAN0 ISR (IRQ 39) — calls RingBuffer_Push() ONLY
 *   Consumer: main loop          — calls RingBuffer_Pop()  ONLY
 *
 * RB-C01: HEAD is written ONLY by the producer (ISR).
 *         TAIL is written ONLY by the consumer (main).
 *         If any other code writes head or tail, the lock-free design
 *         breaks silently — data corruption with no visible error.
 *
 * RB-C08: No interrupt disabling. No critical sections. The SPSC
 *         invariant plus DMB barriers are the synchronisation mechanism.
 *         Disabling IRQs in Push would add latency to the CAN0 ISR.
 *
 *****************************************************************************/

#include "Std_Types.h"     /* uint8/uint32/boolean (Platform_Types.h) + NULL_PTR (Compiler.h) */
#include "can_frame.h"

/* ============================================================
 *  Capacity and Mask
 * ============================================================ */

/* ----------------------------------------------------------------
 * RING_BUFFER_CAPACITY — must be a power of 2 for masking to work.
 * 32 frames × ~16 bytes ≈ 512 bytes RAM.
 * At 500 kbps, back-to-back frames are ~250 us apart; 32 absorbs
 * ~8 ms of burst before drop — large margin over a single frame,
 * for almost no SRAM on the 32 KB TM4C123GH6PM.
 * ---------------------------------------------------------------- */
#define RING_BUFFER_CAPACITY   32U

/* Derived mask — used for index wrapping without modulo.
 * Modulo on non-power-of-2 generates a division instruction on
 * Cortex-M4 (no hardware divide for arbitrary divisors). Masking
 * compiles to a single AND instruction.                          */
#define RING_BUFFER_MASK       (RING_BUFFER_CAPACITY - 1U)

/* ============================================================
 *  Error Codes
 * ============================================================ */

typedef enum
{
    RING_BUFFER_OK       = 0x00U,  /* Operation successful              */
    RING_BUFFER_FULL     = 0x01U,  /* Push rejected — buffer full       */
    RING_BUFFER_EMPTY    = 0x02U,  /* Pop rejected — buffer empty       */
    RING_BUFFER_NULL_PTR = 0x03U,  /* NULL frame pointer passed         */
    RING_BUFFER_PARAM    = 0x04U   /* Invalid parameter passed          */
} RingBuffer_Error_t;

/* ============================================================
 *  Initialization
 * ============================================================ */

/**
 * @brief  Initialize the ring buffer — clears head, tail, buf[].
 *         Must be called once before any Push or Pop.
 *         Safe to call again to reset the buffer mid-session.
 * @note   Not ISR-safe — call only from main context before
 *         enabling the CAN0 interrupt (NVIC EN1).
 * @return RING_BUFFER_OK always.
 */
RingBuffer_Error_t RingBuffer_Init(void);

/* ============================================================
 *  Producer API (ISR context)
 * ============================================================ */

/**
 * @brief  Push one Can_FrameType into the ring buffer.
 *
 *         ISR-safe. Lock-free SPSC design — no interrupt disable.
 *         Uses DMB barrier to guarantee write ordering on Cortex-M4.
 *
 *         If the buffer is full, the frame is DROPPED and the
 *         drop counter is incremented. Oldest data is NOT overwritten.
 *         Rationale: dropping the newest with a visible counter is
 *         better than corrupt data from overwrite during a read.
 *
 * @param  frame  Pointer to Can_FrameType to copy in (must not be NULL)
 * @return RING_BUFFER_OK       on success
 *         RING_BUFFER_FULL     if no space — frame dropped
 *         RING_BUFFER_NULL_PTR if frame is NULL
 *
 * @note   SPSC invariant (RB-C01): ONLY this function writes head.
 *         Any other code writing head corrupts the buffer silently.
 */
RingBuffer_Error_t RingBuffer_Push(const Can_FrameType *frame);

/* ============================================================
 *  Consumer API (main loop context)
 * ============================================================ */

/**
 * @brief  Pop one Can_FrameType from the ring buffer.
 *
 *         Main-loop-safe. Lock-free SPSC design.
 *         Uses DMB barrier after read, before tail increment.
 *
 * @param  frame  Pointer to destination struct (must not be NULL)
 * @return RING_BUFFER_OK       on success
 *         RING_BUFFER_EMPTY    if no data available
 *         RING_BUFFER_NULL_PTR if frame is NULL
 *
 * @note   SPSC invariant (RB-C01): ONLY this function writes tail.
 *         Any other code writing tail corrupts the buffer silently.
 */
RingBuffer_Error_t RingBuffer_Pop(Can_FrameType *frame);

/* ============================================================
 *  Status Queries
 * ============================================================ */

/**
 * @brief  Return number of frames currently available to pop.
 *         Based on (head - tail) unsigned subtraction (RB-C04).
 *         Safe to call from either context.
 * @return frame count (0 = empty, RING_BUFFER_CAPACITY = full)
 */
uint32 RingBuffer_Count(void);

/**
 * @brief  Return 1U if buffer is empty, 0U otherwise.
 *         Empty condition (RB-C03): head == tail.
 * @return 1U = empty, 0U = has data
 */
uint8 RingBuffer_IsEmpty(void);

/**
 * @brief  Return 1U if buffer is full, 0U otherwise.
 *         Full condition (RB-C02): (head - tail) >= CAPACITY.
 * @return 1U = full, 0U = has space
 */
uint8 RingBuffer_IsFull(void);

/* ============================================================
 *  Diagnostics
 * ============================================================ */

/**
 * @brief  Return total number of successful pushes since Init.
 *         Saturates at 0xFFFFFFFFUL.
 * @return push count
 */
uint32 RingBuffer_GetPushCount(void);

/**
 * @brief  Return total number of successful pops since Init.
 *         Saturates at 0xFFFFFFFFUL.
 * @return pop count
 */
uint32 RingBuffer_GetPopCount(void);

/**
 * @brief  Return total frames dropped (push when full) since Init.
 *         Saturates at 0xFFFFFFFFUL. Non-zero means the main loop is
 *         too slow or the RX rate is too high.
 * @return drop count
 */
uint32 RingBuffer_GetDropCount(void);

/**
 * @brief  Return the maximum number of frames ever simultaneously
 *         present in the buffer since Init (high-water mark).
 * @return maximum fill level (0..RING_BUFFER_CAPACITY)
 */
uint32 RingBuffer_GetMaxFill(void);

#endif /* RING_BUFFER_H */
