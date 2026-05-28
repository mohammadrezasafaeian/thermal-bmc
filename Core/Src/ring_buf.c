#include "ring_buf.h"
#include <stddef.h>

#define RB_SIZE 16
#define RB_MASK (RB_SIZE - 1)

typedef struct {
    Event_t buffer[RB_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} RingBuffer_t;


static RingBuffer_t g_event_queue;

void RingBuffer_Init(void) {
    g_event_queue.head = 0;
    g_event_queue.tail = 0;
}

bool RingBuffer_IsEmpty(void) {
    // Empty if head catches up to tail
    return (g_event_queue.head == g_event_queue.tail);
}

bool RingBuffer_IsFull(void) {
    // Calculate where head WILL BE after next write
    uint8_t next_head = (g_event_queue.head + 1) & RB_MASK;
    // Full if next write position hits tail
    return (next_head == g_event_queue.tail);
}

bool RingBuffer_Push(Event_t evt) {
    // 1. CHECK FIRST: Are we full?
    if (RingBuffer_IsFull()) {
        return false; // Buffer full, drop event (or handle error)
    }

    // 2. Write data
    g_event_queue.buffer[g_event_queue.head] = evt;

    // 3. Advance head with wrap-around
    // Using local variable isn't strictly necessary here since we already checked,
    // but it's good practice for minimizing time with interrupts disabled if we added that later.
    g_event_queue.head = (g_event_queue.head + 1) & RB_MASK;

    return true;
}

bool RingBuffer_Pop(Event_t* out_evt) {
    if (out_evt == NULL) return false;

    // 1. CHECK FIRST: Are we empty?
    if (RingBuffer_IsEmpty()) {
        return false;
    }

    // 2. Read data
    *out_evt = g_event_queue.buffer[g_event_queue.tail];

    // 3. Advance tail with wrap-around
    g_event_queue.tail = (g_event_queue.tail + 1) & RB_MASK;

    return true;
}
