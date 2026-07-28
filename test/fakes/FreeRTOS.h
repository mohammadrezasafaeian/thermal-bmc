/* Host stand-in for FreeRTOS.
 *
 * The tests drive the control logic single-threaded, so there is no
 * concurrency to arbitrate: a mutex that always succeeds is an accurate
 * model of what actually happens in the harness.
 *
 * NOT MODELLED, deliberately: preemption, priority inversion, blocking,
 * deadlock, races. Those are real on the target and have to be found there.
 */
#ifndef FAKE_FREERTOS_H
#define FAKE_FREERTOS_H

#include <stdint.h>
#include <stddef.h>

typedef uint32_t TickType_t;
typedef int      BaseType_t;
typedef unsigned UBaseType_t;

#define pdTRUE   1
#define pdFALSE  0
#define pdPASS   1
#define pdFAIL   0
#define portMAX_DELAY  0xFFFFFFFFu
#define configTICK_RATE_HZ 1000u
#define pdMS_TO_TICKS(ms)  ((TickType_t)(ms))

/* No preemption in the harness, so a yield request is a no-op. */
#define portYIELD_FROM_ISR(x)  ((void)(x))

#endif
