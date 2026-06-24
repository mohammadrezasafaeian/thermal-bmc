#include "ring_buf.h"
#include <stddef.h>

/* ============================================================================
 * SPSC ring buffer — Single Producer (TIM2 ISR), Single Consumer (main loop)
 *
 * OWNERSHIP CONTRACT:
 *   head : written ONLY by the ISR  (Push side),  read by both
 *   tail : written ONLY by main loop (Pop side),  read by both
 *   One writer per index + naturally-atomic byte stores on Cortex-M
 *   => no disable-IRQ, no locks needed.
 * ========================================================================== */

#define RB_SIZE 16              /* power of 2: makes (x & RB_MASK) == (x % RB_SIZE) */
#define RB_MASK (RB_SIZE - 1)   /* 0b1111 — all-ones mask, 1-cycle AND vs costly %  */

typedef struct {
    Event_t buffer[RB_SIZE];
    volatile uint8_t head;      /* next slot to WRITE  */
    volatile uint8_t tail;      /* next slot to READ   */
} RingBuffer_t;

static RingBuffer_t q;

void RingBuffer_Init(void) { q.head = 0; q.tail = 0; }

bool RingBuffer_IsEmpty(void)
{
    return (q.head == q.tail);
}

bool RingBuffer_IsFull(void)
{
    /* Full = the slot AFTER head is where tail stands. Mask applied to the
     * successor, so the wrap (head=15 -> 0) is handled.
     * One slot stays sacrificed: if all 16 could fill, head==tail would be
     * both "empty" and "full" — ambiguous. Capacity is RB_SIZE-1 = 15.    */
    return (((q.head + 1) & RB_MASK) == q.tail);
}

bool RingBuffer_Push(Event_t evt)       /* ISR context only */
{
    if (RingBuffer_IsFull()) return false;   /* drop, never overwrite */

    /* ORDER MATTERS: data first, index second.
     * head is the consumer's "data ready" signal. If we advanced head first,
     * the main loop could run between the two lines (the ISR can't be
     * interrupted by main, but the main loop resumes the instant we return —
     * and with nested/multiple IRQs the gap is real), see the new head,
     * and Pop a slot whose data was never stored: a stale/garbage event.
     * Publishing the index LAST means the slot is complete before it is
     * visible.                                                            */
    q.buffer[q.head] = evt;
    q.head = (q.head + 1) & RB_MASK;
    return true;
}

bool RingBuffer_Pop(Event_t *out)       /* main loop only */
{
    if (out == NULL) return false;
    if (RingBuffer_IsEmpty()) return false;

    /* Mirror discipline: copy first, release the slot second.
     * tail is the producer's "slot free" signal. If we advanced tail first,
     * the ISR could fire between the two lines, see the freed slot, and
     * OVERWRITE buffer[old tail] while we are still copying from it.      */
    *out = q.buffer[q.tail];
    q.tail = (q.tail + 1) & RB_MASK;
    return true;
}
