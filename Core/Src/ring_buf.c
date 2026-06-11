#include "ring_buf.h"
#include <stddef.h>

/* ============================================================================
 * SPSC ring buffer (Single Producer = TIM2 ISR, Single Consumer = main loop)
 *
 * OWNERSHIP CONTRACT (this is the whole trick — fill in TODO 1):
 *   head : written ONLY by the main file , read by both
 *   tail : written ONLY by ISR, read by both
 *   Because each index has exactly one writer, no disable-IRQ, no locks.
 * ========================================================================== */

#define RB_SIZE 16              /* must be a power of 2 — see TODO 4 */
#define RB_MASK (RB_SIZE - 1)

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
    /* TODO 2: write the full-detection condition.
     * Constraint you must respect: with this scheme, the buffer can hold at
     * most RB_SIZE-1 events, NOT RB_SIZE. In a comment, answer: WHY do we
     * sacrifice one slot? What would go wrong if we allowed all 16 to fill?
     * we keep one slot for differentiating between full and empty
     * (Hint location, not answer: look at IsEmpty's condition.)            */
	if (q.head == q.tail-1)
		return true
		else:
		return false


}

bool RingBuffer_Push(Event_t evt)       /* called from ISR context only */
{
    if (RingBuffer_IsFull()) return false;   /* drop, never overwrite    */

    /* TODO 3a: two lines — (i) store evt into the buffer, (ii) advance head.
     * THE ORDER OF THESE TWO LINES IS THE EXAM. In a comment, answer:
     * if you advanced head FIRST and stored the data SECOND, what exact
     * sequence of events corrupts the consumer? Walk it: ISR does step (ii),
     * then... what can happen before step (i)?                            */
    q.buffer[q.head++] = evt;
    return true;
}

bool RingBuffer_Pop(Event_t *out)       /* called from main loop only */
{
    if (out == NULL) return false;
    if (RingBuffer_IsEmpty()) return false;

    /* TODO 3b: two lines — (i) copy buffer[tail] to *out, (ii) advance tail.
     * Same ordering discipline, mirrored. One-line comment: why is THIS
     * order the safe one on the consumer side?                            */
    &out = q.buffer[q.tail++]
    return true;
}

/* TODO 4 (comment only, 2 sentences): the index advance is
 *      idx = (idx + 1) & RB_MASK;
 * Why must RB_SIZE be a power of 2 for this to work — and what does this
 * buy over the alternative  idx = (idx + 1) % RB_SIZE  on a Cortex-M4?
 * (One of the two answers is about correctness, the other about cost.)    */
