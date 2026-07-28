#ifndef FAKE_TASK_H
#define FAKE_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"   /* fake_tick_advance, HAL_GetTick */

typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

/* Records the task rather than starting it: the test calls the control
   functions itself, so nothing needs to run concurrently. */
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint16_t stack,
                       void *arg, UBaseType_t prio, TaskHandle_t *handle);

void vTaskDelay(TickType_t ticks);        /* advances the fake clock */
TickType_t xTaskGetTickCount(void);

int  fake_task_count(void);
void fake_task_reset(void);

#endif

/* CMSIS-RTOS shim used in a couple of places */
void osDelay(uint32_t ms);
