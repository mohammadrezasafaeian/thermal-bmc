/* ==========================================================================
 * thermal_app.c  --  Thermal Logger with per-zone FSM + PID
 * Target : STM32F411CEU6 @ 24 MHz (HSI/PLL, verified via TIM2 cross-check)
 *
 * ARCHITECTURE
 * ============
 *   main.c  -> ThermalApp_Init()  once
 *           -> ThermalApp_Loop()  forever
 *
 *   TIM2 ISR @ 1 Hz stamps DWT->CYCCNT into g_prof.tick_stamp, then pushes
 *   EVT_PID_TICK into the ring buffer. ThermalApp_Loop drains the queue.
 *   On EVT_PID_TICK:
 *      1. ADC oversample -> V_node -> R_ntc -> raw_t_c
 *      2. pid_temp EMA   (control-path filter)
 *      3. Zone_Tick(...)  ONE call - the tick IS the loop, no inner spin
 *      4. log, OLED throttle
 *      5. pwm_update at the bottom of the loop
 *
 * PROFILING (Phase 0 baseline for FreeRTOS migration)
 * ============
 *   g_prof (cyc.h) - single Live Expressions entry:
 *      lat_*_us  : TIM2 ISR -> EVT_PID_TICK drain latency (the headline)
 *      adc_us    : one adc_average() oversample burst
 *      oled_us   : one oled_update() I2C redraw (+ worst case)
 *      ctrl_us   : whole control body (EMA+FSM+PID+log)
 *   All in microseconds via CYC_TO_US (CPU_HZ = 24 MHz in cyc.h).
 *
 * MENTAL MODEL
 * ============
 *   TICK = WHEN, STATE = WHAT.
 *   Two signals, two bandwidths:
 *      CONTROL path -> ema_temp  (slow, noise-reject)  -> PID
 *      SAFETY  path -> raw vnode (fast, exact)         -> fault detect
 *   Time-domain Schmitt for debounce: integer count on RAW (not EMA).
 *   Per-zone state = per-zone fault isolation.
 * ==========================================================================*/

#include "thermal_app.h"
#include "ssd1306.h"
#include "ring_buf.h"
#include "cyc.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#define THERMAL_LOG_MAGIC  0xC0FFEE42u

__attribute__((section(".noinit")))
uint32_t thermal_log_magic;

/* ── HAL handles (from CubeMX, in main.c) ───────────────────────────────── */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* ============================================================================
 * PUBLIC GLOBALS
 * ========================================================================== */
ZoneCtrl zone1 = {
    .name         = "Z1",
    .state        = ST_IDLE,
    .fault_reason = FR_NONE,
    .setpoint_c   = 30.0f,
    .duty_cmd     = 0.0f,
};

volatile float g_duty_cmd   = 0.0f;
volatile float g_setpoint_c = 30.0f;    /* legacy: kept for old Live Expr     */
volatile float g_fan_duty   = 0.0f;

PID_Handle pid;                          /* legacy alias - copy from zone1    */

__attribute__((aligned(4), section(".noinit")))
ThermalLogEntry thermal_log[THERM_LOG_LEN];

__attribute__((aligned(4), section(".noinit")))
ThermalEvent thermal_events[THERM_EVENT_LEN];

__attribute__((section(".noinit")))
volatile uint32_t thermal_log_idx;

__attribute__((section(".noinit")))
volatile uint32_t thermal_event_idx;

/* Debug read-back globals */
volatile uint32_t dbg_adc_avg = 0;
volatile float    dbg_vnode   = 0.0f;
volatile float    dbg_rntc    = 0.0f;
volatile float    dbg_temp_c  = 0.0f;

/* DWT profiling - ALL timing lives here. Add "g_prof" to Live Expressions. */
volatile Profiler g_prof = { .lat_min_us = 0xFFFFFFFFu };

/* ============================================================================
 * PRIVATE STATE
 * ========================================================================== */
static float   plot_buf[THERM_PLOT_LEN];
static uint8_t plot_head = 0;

static float   ema_temp_display = 0.0f;
static float   ema_mean         = 0.0f;
static float   ema_dev          = 1.0f;
static uint8_t ema_initialized  = 0;

static float   pid_temp         = 0.0f;  /* EMA-filtered temp for PID input  */

/* ============================================================================
 * SENSING HELPERS  (unchanged from pre-FSM version)
 * ========================================================================== */

static uint32_t adc_average(uint8_t n)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            sum += HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
    return sum / n;
}

static float adc_to_rntc(uint32_t adc_val)
{
    float v = THERM_ADC_VREF * ((float)adc_val / 4095.0f);
    if (v < 0.001f) v = 0.001f;
    float v_max = THERM_DIV_VSUP - 0.001f;
    if (v_max > (THERM_ADC_VREF - 0.001f)) v_max = THERM_ADC_VREF - 0.001f;
    if (v > v_max) v = v_max;
    float rntc = THERM_RTOP * (v / (THERM_DIV_VSUP - v));
    if (rntc <    0.1f) rntc =    0.1f;
    if (rntc > 5000.0f) rntc = 5000.0f;
    return rntc;
}

static float rntc_to_celsius(float rntc)
{
    float inv_T = (1.0f / THERM_T0_K)
                + (1.0f / THERM_BETA) * logf(rntc / THERM_R0);
    if (inv_T < 1e-6f) inv_T = 1e-6f;
    return (1.0f / inv_T) - 273.15f;
}

static inline float ema_step(float state, float value, float alpha)
{
    return state + alpha * (value - state);
}

/* ============================================================================
 * PWM OUTPUT
 *   Reads g_duty_cmd [-1..+1] and splits into heater (CH1) / fan (CH2).
 *   Single point of contact with the hardware - FSM only writes g_duty_cmd.
 * ========================================================================== */
static void pwm_update(void)
{
    float cmd = g_duty_cmd;
    if (cmd < -1.0f) cmd = -1.0f;
    if (cmd >  1.0f) cmd =  1.0f;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);

    float heater_duty, fan_duty;
    if (cmd >= 0.0f) { heater_duty = cmd;  fan_duty = 0.0f; }
    else             { heater_duty = 0.0f; fan_duty = -cmd; }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(heater_duty * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(fan_duty    * (float)arr));

    g_fan_duty = fan_duty;
}

/* ============================================================================
 * STREAM 1 - dense per-tick log
 * ========================================================================== */
static void log_sample(float temp_c, float temp_ema, float vnode, float duty)
{
    uint32_t idx = thermal_log_idx % THERM_LOG_LEN;
    thermal_log[idx].time_s   = HAL_GetTick() * 0.001f;
    thermal_log[idx].temp_c   = temp_c;
    thermal_log[idx].temp_ema = temp_ema;
    thermal_log[idx].vnode    = vnode;
    thermal_log[idx].duty_cmd = duty;
    thermal_log_idx++;
}

/* ============================================================================
 * STREAM 2 - sparse event log (one entry per CHANGE)
 * ========================================================================== */
static void event_log(EventKind kind, uint8_t u8, float f)
{
    uint32_t idx = thermal_event_idx % THERM_EVENT_LEN;
    thermal_events[idx].time_s     = HAL_GetTick() * 0.001f;
    thermal_events[idx].kind       = (uint8_t)kind;
    thermal_events[idx].u8_payload = u8;
    thermal_events[idx]._pad       = 0;
    thermal_events[idx].f_payload  = f;
    thermal_event_idx++;
}

/* ---- emit_change_events: edge detector ---------------------------------- */
static void emit_change_events(ZoneCtrl *z)
{
    static uint8_t last_state        = 0xFF;     /* impossible -> first tick logs */
    static uint8_t last_fault_reason = 0xFF;
    static float   last_setpoint     = -1000.0f; /* impossible setpoint           */

    if (z->state != last_state) {
        event_log(EV_STATE_CHANGE, z->state, 0.0f);
        last_state = z->state;
    }

    if (z->fault_reason != last_fault_reason) {
        if (z->fault_reason == FR_NONE)
            event_log(EV_FAULT_CLEARED, last_fault_reason, 0.0f);
        else
            event_log(EV_FAULT_RAISED,  z->fault_reason,   0.0f);
        last_fault_reason = z->fault_reason;
    }

    if (z->setpoint_c != last_setpoint) {
        event_log(EV_SETPOINT_CHG, 0, z->setpoint_c);
        last_setpoint = z->setpoint_c;
    }
}

/* ============================================================================
 * OLED  (unchanged - just renders whatever's in plot_buf + globals)
 * ========================================================================== */
static int format_fixed1(char *buf, int buf_len, float val)
{
    int sign = (val < 0.0f) ? 1 : 0;
    if (sign) val = -val;
    int32_t tenths = (int32_t)(val * 10.0f + 0.5f);
    int32_t whole  = tenths / 10;
    int32_t frac   = tenths % 10;
    if (sign) return snprintf(buf, buf_len, "-%ld.%ld", (long)whole, (long)frac);
    else      return snprintf(buf, buf_len, "%ld.%ld",  (long)whole, (long)frac);
}

static void oled_update(uint32_t adc_avg, float vnode, float rntc,
                        float temp_c, int duty_pct)
{
    char buf[22];
    ssd1306_clear();

    {
        char v_str[8];
        format_fixed1(v_str, sizeof(v_str), vnode);
        snprintf(buf, sizeof(buf), "A:%4lu V:%s", (unsigned long)adc_avg, v_str);
    }
    ssd1306_print(0, 0, buf);

    {
        char r_str[8], t_str[8];
        format_fixed1(r_str, sizeof(r_str), rntc);
        format_fixed1(t_str, sizeof(t_str), temp_c);
        snprintf(buf, sizeof(buf), "R:%s T:%sC", r_str, t_str);
    }
    ssd1306_print(0, 8, buf);

    const char *st_str = "?";
    switch (zone1.state) {
        case ST_IDLE:    st_str = "IDLE";  break;
        case ST_PID:     st_str = "PID ";  break;
        case ST_COOLING: st_str = "COOL";  break;
        case ST_FAULT:   st_str = "FALT";  break;
    }
    snprintf(buf, sizeof(buf), "%s %s D:%3d", zone1.name, st_str, duty_pct);
    ssd1306_print(0, 16, buf);

    ssd1306_draw_line(0, 20, 127, 20);

    const uint8_t PLOT_TOP    = 22;
    const uint8_t PLOT_BOTTOM = 63;
    const uint8_t PLOT_H      = PLOT_BOTTOM - PLOT_TOP;

    float half_span = THERM_PLOT_K * ema_dev;
    if (half_span < THERM_PLOT_MIN_SPAN / 2.0f) half_span = THERM_PLOT_MIN_SPAN / 2.0f;
    float plot_min = ema_mean - half_span;
    float span     = 2.0f * half_span;

    int32_t prev_y = -1;
    for (uint8_t x = 0; x < THERM_PLOT_LEN; x++) {
        uint8_t idx = (uint8_t)((plot_head + x) % THERM_PLOT_LEN);
        float val = plot_buf[idx];
        float norm = (val - plot_min) / span;
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        int32_t y = (int32_t)PLOT_BOTTOM - (int32_t)(norm * (float)PLOT_H);
        if (prev_y >= 0) ssd1306_draw_line(x - 1, (uint8_t)prev_y, x, (uint8_t)y);
        else             ssd1306_draw_pixel(x, (uint8_t)y, 1);
        prev_y = y;
    }
    ssd1306_update();
}

/* ============================================================================
 * FSM HELPERS - small, single-purpose, no hidden side effects
 * ========================================================================== */

/* detect_fault: pure function of RAW vnode. Caller debounces in time. */
static FaultReason detect_fault(float vnode)
{
    if (vnode > V_OPEN_THRESH)  return FR_NTC_OPEN;
    if (vnode < V_SHORT_THRESH) return FR_NTC_SHORT;
    return FR_NONE;
    /* FR_HEATER_OPEN / FR_FAN_OPEN: hooks for Topic 11 (current sense). */
}

/* zone_actuators_off: hardware-level cut. Used on every safety transition. */
static void zone_actuators_off(ZoneCtrl *z)
{
    z->duty_cmd = 0.0f;
    g_duty_cmd  = 0.0f;     /* mirror - pwm_update writes 0 at end of loop  */
}

/* zone_enter_pid: SINGLE entry path to PID state. Bumpless transfer. */
static void zone_enter_pid(ZoneCtrl *z, float seed_temp_c)
{
    pid_temp = seed_temp_c;              /* snap EMA to current truth */
    PID_Init(&z->pid, PID_KP, PID_KI, PID_KD, PID_TS, PID_TAU_F, seed_temp_c);
    z->fault_count   = 0;
    z->recover_count = 0;
    z->state         = ST_PID;
}

/* zone_enter_cooling: stop requested, graceful shutdown. */
static void zone_enter_cooling(ZoneCtrl *z)
{
    z->cool_ticks = 0;
    z->state      = ST_COOLING;
}

/* zone_enter_fault: cut on the transition (sub-tick latency). */
static void zone_enter_fault(ZoneCtrl *z, FaultReason r)
{
    zone_actuators_off(z);
    z->fault_reason  = r;
    z->recover_count = 0;
    z->state         = ST_FAULT;
}

/* ============================================================================
 * Zone_Tick - THE FSM  (unchanged)
 * ========================================================================== */
void Zone_Tick(ZoneCtrl *z, float vnode, float raw_t_c, float ema_t_c)
{
    FaultReason fault_candidate = detect_fault(vnode);

    switch (z->state) {

    case ST_IDLE:
        zone_actuators_off(z);

        if (z->start_req) {
            z->start_req = 0;
            if (fault_candidate == FR_NONE) {
                zone_enter_pid(z, raw_t_c);   /* RAW so snap is meaningful */
            } else {
                zone_enter_fault(z, fault_candidate);
            }
        }
        break;

    case ST_PID:
        /* Fault debounce on RAW signal (time-domain Schmitt) */
        if (fault_candidate != FR_NONE) {
            z->fault_count++;
            if (z->fault_count >= FAULT_TRIP_N) {
                zone_enter_fault(z, fault_candidate);
                break;
            }
        } else {
            z->fault_count = 0;
        }

        if (z->stop_req) {
            z->stop_req = 0;
            zone_enter_cooling(z);
            break;
        }

        z->duty_cmd = PID_Update(&z->pid, z->setpoint_c, ema_t_c);
        g_duty_cmd  = z->duty_cmd;
        break;

    case ST_COOLING:
        z->duty_cmd = -1.0f;            /* fan full on, heater off          */
        g_duty_cmd  = z->duty_cmd;
        z->cool_ticks++;

        {
            uint8_t cool_now  = (raw_t_c < COOL_THRESH_C);
            uint8_t sane      = (fault_candidate == FR_NONE);
            uint8_t timed_out = (z->cool_ticks >= COOLING_TIMEOUT_TICKS);

            if ((cool_now && sane) || timed_out) {
                zone_actuators_off(z);
                z->state = ST_IDLE;
            }
        }
        break;

    case ST_FAULT:
        zone_actuators_off(z);

        if (fault_candidate == FR_NONE) {
            z->recover_count++;
            if (z->recover_count >= FAULT_RECOVER_M) {
                z->fault_reason  = FR_NONE;
                z->recover_count = 0;
                z->state         = ST_IDLE;   /* never -> PID; operator must start */
            }
        } else {
            z->recover_count = 0;             /* glitch breaks the streak   */
        }
        break;

    default:
        zone_enter_fault(z, FR_NONE);   /* unknown state -> safe-park       */
        break;
    }
}

/* ============================================================================
 * PUBLIC API
 * ========================================================================== */
void ThermalApp_Init(void)
{
    /* 0. Start the cycle counter FIRST - every later block gets profiled. */
    cyc_init();

    /* 1. Drain event queue before any ISR can push to it */
    RingBuffer_Init();

    /* 2. Start PWM channels at 0 % (safe state) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

    /* plot_buf is not in .noinit - always clear (display state, not log). */
    memset(plot_buf, 0, sizeof(plot_buf));
    plot_head = 0;

    /* Black-box logs: only clear on COLD boot. Magic word distinguishes. */
    if (thermal_log_magic != THERMAL_LOG_MAGIC) {
        memset(thermal_log,    0, sizeof(thermal_log));
        memset(thermal_events, 0, sizeof(thermal_events));
        thermal_log_idx   = 0;
        thermal_event_idx = 0;
        thermal_log_magic = THERMAL_LOG_MAGIC;
    }
    /* On warm boot: do nothing - buffers retain previous run's data. */

    /* 4. Initial temperature reading (seeds PID derivative + EMA) */
    uint32_t adc_raw   = adc_average(THERM_ADC_OVERSAMPLE);
    float    init_rntc = adc_to_rntc(adc_raw);
    float    init_temp = rntc_to_celsius(init_rntc);
    pid_temp = init_temp;

    /* 5. Seed PID inside the zone. State remains ST_IDLE. */
    PID_Init(&zone1.pid, PID_KP, PID_KI, PID_KD, PID_TS, PID_TAU_F, init_temp);
    zone1.state         = ST_IDLE;
    zone1.fault_reason  = FR_NONE;
    zone1.fault_count   = 0;
    zone1.recover_count = 0;
    zone1.cool_ticks    = 0;

    /* 6. Splash screen */
    ssd1306_clear();
    ssd1306_print(0, 0,  "THERMAL FSM");
    ssd1306_print(0, 16, "V0.8 PROF");
    ssd1306_update();
    HAL_Delay(800);

    /* 7. LAST: start the 1 Hz heartbeat. After this line, ISR can fire. */
    HAL_TIM_Base_Start_IT(&htim2);
}

void ThermalApp_Loop(void)
{
    Event_t current_event;
    g_prof.loop_count++;

    /* --- STEP 1: SENSING (every loop iteration, regardless of events) --- */
    uint32_t t0 = DWT->CYCCNT;
    uint32_t adc_avg = adc_average(THERM_ADC_OVERSAMPLE);
    g_prof.adc_us = CYC_TO_US(DWT->CYCCNT - t0);

    float vnode  = THERM_ADC_VREF * ((float)adc_avg / 4095.0f);
    float rntc   = adc_to_rntc(adc_avg);
    float temp_c = rntc_to_celsius(rntc);

    dbg_adc_avg = adc_avg;
    dbg_vnode   = vnode;
    dbg_rntc    = rntc;
    dbg_temp_c  = temp_c;

    /* --- STEP 2: EVENT DRAIN --- */
    while (RingBuffer_Pop(&current_event))
    {
        switch (current_event)
        {
        case EVT_PID_TICK:
        {
            /* ── PROFILE: tick -> drain latency (the headline number) ──── */
            uint32_t lat = CYC_TO_US(DWT->CYCCNT - g_prof.tick_stamp);
            g_prof.lat_last_us = lat;
            if (lat > g_prof.lat_max_us) g_prof.lat_max_us = lat;
            if (lat < g_prof.lat_min_us) g_prof.lat_min_us = lat;
            g_prof.tick_count++;

            uint32_t tc0 = DWT->CYCCNT;          /* control body start     */

            /* Control-path EMA - only when controller is alive. */
            if (zone1.state == ST_PID) {
                pid_temp = pid_temp + 0.02f * (temp_c - pid_temp);
            }

        #if OPEN_LOOP_TEST_MODE
            /* Bypass FSM entirely for plant ID step test. */
            if (g_prof.tick_count <= OPEN_LOOP_BASELINE_SEC) {
                g_duty_cmd = 0.0f;
            } else if (g_prof.tick_count <= OPEN_LOOP_BASELINE_SEC + OPEN_LOOP_RUN_SEC) {
                g_duty_cmd = OPEN_LOOP_DUTY;
            } else {
                g_duty_cmd = 0.0f;
            }
        #else
            /* Normal path: FSM runs the show. */
            Zone_Tick(&zone1, vnode, temp_c, pid_temp);
        #endif

            /* Hard safety backstop - independent of FSM. */
            if (temp_c > MAX_SAFE_TEMP_C) {
                g_duty_cmd     = 0.0f;
                zone1.duty_cmd = 0.0f;
            }

            /* Display-side EMAs (cosmetic, for plot) */
            if (!ema_initialized) {
                ema_temp_display = temp_c;
                ema_mean         = temp_c;
                ema_dev          = 1.0f;
                ema_initialized  = 1;
            } else {
                ema_temp_display = ema_step(ema_temp_display, temp_c, THERM_EMA_DISPLAY);
                ema_mean         = ema_step(ema_mean, ema_temp_display, THERM_EMA_MEAN);
                float abs_dev = ema_temp_display - ema_mean;
                if (abs_dev < 0.0f) abs_dev = -abs_dev;
                ema_dev = ema_step(ema_dev, abs_dev, THERM_EMA_DEV);
                if (ema_dev < 0.05f) ema_dev = 0.05f;
            }

            plot_buf[plot_head] = ema_temp_display;
            plot_head = (uint8_t)((plot_head + 1) % THERM_PLOT_LEN);

            /* Stream 1: dense per-tick data (RAW temp - plant ID needs it) */
            log_sample(temp_c, pid_temp, vnode, g_duty_cmd);

            /* Stream 2: edge-detected sparse events */
            emit_change_events(&zone1);

            /* ── PROFILE: control body cost ─────────────────────────────── */
            g_prof.ctrl_us = CYC_TO_US(DWT->CYCCNT - tc0);
            break;
        }

        case EVT_START_CMD:
            zone1.start_req = 1;
            break;

        case EVT_STOP_CMD:
            zone1.stop_req = 1;
            break;

        default:
            break;
        }
    }

    /* --- STEP 3: ACTUATION --- */
    pwm_update();

    /* --- STEP 4: UI (throttled by THERM_OLED_DIVIDER) --- */
    {
        static uint8_t oled_div = 0;
        oled_div++;
        if (oled_div >= THERM_OLED_DIVIDER) {
            oled_div = 0;
            int duty_pct = (int)(g_duty_cmd * 100.0f + 0.5f);
            if (duty_pct < -100) duty_pct = -100;
            if (duty_pct >  100) duty_pct =  100;

            /* ── PROFILE: OLED redraw cost (the prime suspect) ─────────── */
            uint32_t t1 = DWT->CYCCNT;
            oled_update(adc_avg, vnode, rntc, ema_temp_display, duty_pct);
            g_prof.oled_us = CYC_TO_US(DWT->CYCCNT - t1);
            if (g_prof.oled_us > g_prof.oled_max_us)
                g_prof.oled_max_us = g_prof.oled_us;
        }
    }

    /* No HAL_Delay - the tick paces the control path; loop runs free. */
}
