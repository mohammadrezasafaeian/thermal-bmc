/* Host implementations of the FreeRTOS calls the firmware makes.
 *
 * Tests are single-threaded: the test itself replaces the scheduler and calls
 * the control functions directly. So there is no contention to arbitrate and
 * a mutex that always succeeds is accurate, not a shortcut.
 *
 * Not modelled: preemption, blocking, priority inversion, races.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>


/* ==========================================================================
 * TASKS  -  record the request, never run the function
 *
 * Task bodies are for(;;) loops, so calling one would never return. We just
 * note that init asked for it, so a test can assert "init created 3 tasks".
 * ======================================================================== */

#define FAKE_MAX_TASKS 8

static int s_task_count = 0;


BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint16_t stack,
                       void *arg, UBaseType_t prio, TaskHandle_t *handle)
{
    (void)fn; (void)name; (void)stack; (void)arg; (void)prio;

    if (s_task_count >= FAKE_MAX_TASKS) return pdFAIL;

    s_task_count++;
    if (handle) *handle = (TaskHandle_t)(intptr_t)s_task_count;

    return pdPASS;
}


void vTaskDelay(TickType_t ticks)
{
    fake_tick_advance((uint32_t)ticks);
}


TickType_t xTaskGetTickCount(void)
{
    return HAL_GetTick();
}


void osDelay(uint32_t ms)
{
    fake_tick_advance(ms);
}


int fake_task_count(void)
{
    return s_task_count;
}


void fake_task_reset(void)
{
    s_task_count = 0;
}


/* ==========================================================================
 * SEMAPHORES  -  always succeed, but count the depth
 *
 * With one thread a take can always succeed immediately. Counting take/give
 * costs almost nothing and makes an unbalanced pair visible: on the target
 * that is a deadlock, here it is just a number a test can assert on.
 *
 * Handles are 1-based so they are never NULL.
 * ======================================================================== */

#define FAKE_MAX_SEMS 8

static int s_bal[FAKE_MAX_SEMS];
static int s_sem_count = 0;


SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    if (s_sem_count >= FAKE_MAX_SEMS) return NULL;

    s_sem_count++;
    return (SemaphoreHandle_t)(intptr_t)s_sem_count;
}


/* Turn a handle back into an array index. Returns -1 if the handle is bad. */
static int sem_index(SemaphoreHandle_t s)
{
    int i = (int)(intptr_t)s - 1;
    return (i >= 0 && i < FAKE_MAX_SEMS) ? i : -1;
}


BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t wait)
{
    (void)wait;

    int i = sem_index(s);
    if (i < 0) return pdFALSE;

    s_bal[i]++;
    return pdTRUE;
}


BaseType_t xSemaphoreGive(SemaphoreHandle_t s)
{
    int i = sem_index(s);
    if (i < 0) return pdFALSE;

    s_bal[i]--;
    return pdTRUE;
}


int fake_sem_balance(SemaphoreHandle_t s)
{
    int i = sem_index(s);
    return (i < 0) ? 0 : s_bal[i];
}


void fake_sem_reset(void)
{
    memset(s_bal, 0, sizeof(s_bal));
    s_sem_count = 0;
}


/* ==========================================================================
 * QUEUE  -  a real ring buffer
 *
 * thermal_app.c:371 blocks on xQueueReceive and 'continue's if it fails, so
 * this one has to genuinely work or the control loop never executes.
 *
 * One queue is enough: the firmware creates exactly one.
 * ======================================================================== */

#define FAKE_Q_SLOTS 16
#define FAKE_Q_ITEM  32          /* bytes; sizeof(Event_t) must fit */

static struct {
    unsigned char buf[FAKE_Q_SLOTS][FAKE_Q_ITEM];
    UBaseType_t   item_size;
    int head, tail, used, created;
} s_q;


QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t item_size)
{
    (void)len;

    if (item_size > FAKE_Q_ITEM) return NULL;

    memset(&s_q, 0, sizeof(s_q));
    s_q.item_size = item_size;
    s_q.created   = 1;

    return (QueueHandle_t)&s_q;
}


BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait)
{
    (void)q; (void)wait;

    if (!s_q.created || s_q.used == FAKE_Q_SLOTS) return pdFALSE;

    memcpy(s_q.buf[s_q.head], item, s_q.item_size);
    s_q.head = (s_q.head + 1) % FAKE_Q_SLOTS;
    s_q.used++;

    return pdTRUE;
}


BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *woken)
{
    if (woken) *woken = pdFALSE;
    return xQueueSend(q, item, 0);
}


BaseType_t xQueueReceive(QueueHandle_t q, void *out, TickType_t wait)
{
    (void)q; (void)wait;

    if (!s_q.created || s_q.used == 0) return pdFALSE;

    memcpy(out, s_q.buf[s_q.tail], s_q.item_size);
    s_q.tail = (s_q.tail + 1) % FAKE_Q_SLOTS;
    s_q.used--;

    return pdTRUE;
}


void fake_queue_reset(void)
{
    memset(&s_q, 0, sizeof(s_q));
}
