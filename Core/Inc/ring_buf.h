#ifndef RING_BUF_H
#define RING_BUF_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    EVT_NONE = 0,
    EVT_START_CMD,
    EVT_STOP_CMD,
    EVT_RESET_CMD,
    EVT_PID_TICK,
    EVT_FAULT_DETECTED,
    EVT_SERIAL_BYTE
} Event_t;

void RingBuffer_Init(void);
bool RingBuffer_IsEmpty(void);
bool RingBuffer_IsFull(void);
bool RingBuffer_Push(Event_t evt);
bool RingBuffer_Pop(Event_t *out_evt);

#endif
