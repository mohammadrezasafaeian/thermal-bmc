#ifndef FAKE_SEMPHR_H
#define FAKE_SEMPHR_H

#include "FreeRTOS.h"

typedef void *SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t wait);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);

/* Counts take/give so a test can assert the pairing is balanced - an
   unbalanced count is a real bug the single-threaded fake CAN catch. */
int fake_sem_balance(SemaphoreHandle_t s);
void fake_sem_reset(void);

#endif
