#ifndef CYC_H
#define CYC_H

#include "stm32f4xx.h"

#define CPU_HZ        24000000u
#define CYC_TO_US(c)  ((c) / (CPU_HZ / 1000000u))   /* cycles -> microseconds */

static inline void cyc_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ── Profiling snapshot: add "g_prof" to Live Expressions, expand once ── */
typedef struct {
    /* tick -> Zone_Tick latency */
    uint32_t tick_stamp;        /* CYCCNT at TIM2 ISR (internal)        */
    uint32_t lat_last_us;
    uint32_t lat_max_us;
    uint32_t lat_min_us;        /* best case - shows the spread          */

    /* per-block costs, most recent */
    uint32_t adc_us;            /* one adc_average() burst               */
    uint32_t oled_us;           /* one oled_update()                     */
    uint32_t ctrl_us;           /* whole EVT_PID_TICK body (FSM+PID+log) */

    /* context */
    uint32_t loop_count;        /* free-spin passes since boot           */
    uint32_t tick_count;        /* 1 Hz ticks processed                  */
    uint32_t oled_max_us;       /* worst OLED ever seen                  */
    uint32_t node_us;
} Profiler;

extern volatile Profiler g_prof;

#endif
