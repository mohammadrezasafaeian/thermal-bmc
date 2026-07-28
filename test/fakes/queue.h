#ifndef FAKE_QUEUE_H
#define FAKE_QUEUE_H

#include "FreeRTOS.h"

typedef void *QueueHandle_t;

QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t item_size);
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait);
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *woken);
BaseType_t xQueueReceive(QueueHandle_t q, void *out, TickType_t wait);

void fake_queue_reset(void);

#endif
