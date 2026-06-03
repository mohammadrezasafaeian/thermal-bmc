/* ==========================================================================
 * thermal_app.c  --  Thermal Logger with per-zone FSM + PID
 * Target : STM32F411CEU6 @ 100 MHz  (STM32CubeIDE + HAL)
 *
 * ARCHITECTURE
 * ============
 *   main.c  -> ThermalApp_Init()  once
 *           -> ThermalApp_Loop()  forever
 *
 *   TIM2 ISR @ 1 Hz pushes EVT_PID_TICK into ring buffer.
 *   ThermalApp_Loop drains the queue. On EVT_PID_TICK:
 *      1. ADC oversample -> V_node -> R_ntc -> raw_t_c
 *      2. pid_temp EMA   (control-path filter)
 *      3. Zone_Tick(...)  ONE call - the tick IS the loop, no inner spin
 *      4. log, OLED throttle
 *      5. pwm_update at the bottom of the loop
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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

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

__attribute__((aligned(4)))
ThermalLogEntry thermal_log[THERM_LOG_LEN];
volatile uint32_t thermal_log_idx = 0;

__attribute__((aligned(4)))
ThermalEvent thermal_events[THERM_EVENT_LEN];
volatile uint32_t thermal_event_idx = 0;

/* Debug read-back globals */
volatile uint32_t dbg_adc_avg = 0;
volatile float    dbg_vnode   = 0.0f;
volatile float    dbg_rntc    = 0.0f;
volatile float    dbg_temp_c  = 0.0f;

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
 *   Pushed by emit_change_events() each tick; also pushed by ThermalApp_Init
 *   for the boot state.
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

/* ---- emit_change_events: edge detector ---------------------------------- *
 *   Sentinel-init "last seen" cache forces first-tick logging of boot state.
 *   Pattern is identical to a hardware rising-edge detector: store previous,
 *   compare to current, emit on disagreement, update the cache.            */
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

    /* Show FSM state + duty on row 16 - more useful than just duty */
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

/* zone_enter_pid: SINGLE entry path to PID state.
 *   Bumpless transfer twin to anti-windup: PID integrator was frozen in
 *   IDLE/FAULT - stale value would dump a step on re-entry. Reset.
 *   fault_count also reset: we're trusting the sensor again from this tick. */
static void zone_enter_pid(ZoneCtrl *z, float seed_temp_c)
{
    pid_temp = seed_temp_c;              // ← snap EMA to current truth
    PID_Init(&z->pid, PID_KP, PID_KI, PID_KD, PID_TS, PID_TAU_F, seed_temp_c);
    z->fault_count   = 0;
    z->recover_count = 0;
    z->state         = ST_PID;
}

/* zone_enter_cooling: stop requested, graceful shutdown.
 *   Fan command is asserted in the COOLING body each tick. */
static void zone_enter_cooling(ZoneCtrl *z)
{
    z->cool_ticks = 0;
    z->state      = ST_COOLING;
}

/* zone_enter_fault: cut on the transition.
 *   Cutting here (not next tick) prevents 1 full tick (1s @ 12W) of extra
 *   heating while ST_FAULT body waits to run.                              */
static void zone_enter_fault(ZoneCtrl *z, FaultReason r)
{
    zone_actuators_off(z);
    z->fault_reason  = r;
    z->recover_count = 0;
    z->state         = ST_FAULT;
}

/* ============================================================================
 * Zone_Tick - THE FSM
 *   Rules:
 *     - TICK = WHEN, STATE = WHAT
 *     - ONE evaluation per call, then return. The tick is the loop.
 *     - All loop memory in ZoneCtrl (no shared mutables)
 *     - Fault wins over stop (sensor lying -> can't trust any other logic)
 * ========================================================================== */
void Zone_Tick(ZoneCtrl *z, float vnode, float raw_t_c, float ema_t_c)
{
    FaultReason fault_candidate = detect_fault(vnode);

    switch (z->state) {

    /* ===================================================================== *
     * ST_IDLE: actuators off, sensors live, wait for start_req.
     *   Defensive: re-assert "off" every tick.
     *   start_req honored only if sensor sane (don't enter PID half-blind). */
    case ST_IDLE:
        zone_actuators_off(z);

        if (z->start_req) {
            z->start_req = 0;
            // In Zone_Tick, the ST_IDLE case:
            if (fault_candidate == FR_NONE) {
                zone_enter_pid(z, raw_t_c);     // was ema_t_c — pass RAW so snap is meaningful
            }else {
                zone_enter_fault(z, fault_candidate);
            }
        }
        break;

    /* ===================================================================== *
     * ST_PID: closed-loop control.
     *   Exit priority:
     *     1. FAULT  (debounced) - sensor lying, abort hard, cut on edge.
     *     2. COOLING (stop_req) - operator wants graceful stop.             */
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

        /* Normal control */
        z->duty_cmd = PID_Update(&z->pid, z->setpoint_c, ema_t_c);
        g_duty_cmd  = z->duty_cmd;
        break;

    /* ===================================================================== *
     * ST_COOLING: walk the plant down.
     *   Fan full on, heater off. Exit to IDLE on (cool AND sane) OR timeout.
     *   Timeout = false-safe backstop: physically cool after ~3*tau even if
     *   the sensor lies (open NTC reads cold -> would mimic "cooled").
     *   No COOLING->FAULT edge: FAULT stops the fan, defeating the cooldown.
     *   Better to keep blowing air through a bad reading than to park hot. */
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

    /* ===================================================================== *
     * ST_FAULT: EX-5 - forgiving auto-recovery to IDLE.
     *   Implemented in EX-5. For now: hold safe.                            */
    case ST_FAULT:
        zone_actuators_off(z);

        /* Recovery debounce: mirror of the trip-side debounce in ST_PID.
         *   sane tick   -> increment count; reset to 0 means CONSECUTIVE evidence.
         *   faulty tick -> slam to 0; a single glitch breaks the streak.
         * Option A (auto-recover, no ack) chosen for breadboard convenience.
         * Real product: AND with z->ack_req (operator intent + sensor evidence). */
        if (fault_candidate == FR_NONE) {
            z->recover_count++;
            if (z->recover_count >= FAULT_RECOVER_M) {
                z->fault_reason  = FR_NONE;   /* clear so OLED/UI shows clean   */
                z->recover_count = 0;
                z->state         = ST_IDLE;   /* never -> PID; operator must start */
            }
        } else {
            z->recover_count = 0;             /* glitch breaks the streak       */
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
    /* 1. Drain event queue before any ISR can push to it */
    RingBuffer_Init();

    /* 2. Start PWM channels at 0 % (safe state) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

    /* 3. Zero display + log buffers */
    memset(plot_buf,    0, sizeof(plot_buf));
    plot_head = 0;
    memset(thermal_log, 0, sizeof(thermal_log));
    thermal_log_idx = 0;
    memset(thermal_events, 0, sizeof(thermal_events));
    thermal_event_idx = 0;
    /* 4. Initial temperature reading (seeds PID derivative + EMA) */
    uint32_t adc_raw   = adc_average(THERM_ADC_OVERSAMPLE);
    float    init_rntc = adc_to_rntc(adc_raw);
    float    init_temp = rntc_to_celsius(init_rntc);
    pid_temp = init_temp;

    /* 5. Seed PID inside the zone. State remains ST_IDLE - operator must
     *    write zone1.start_req = 1 in Live Expressions to start control.   */
    PID_Init(&zone1.pid, PID_KP, PID_KI, PID_KD, PID_TS, PID_TAU_F, init_temp);
    zone1.state        = ST_IDLE;
    zone1.fault_reason = FR_NONE;
    zone1.fault_count  = 0;
    zone1.recover_count = 0;
    zone1.cool_ticks   = 0;

    /* 6. Splash screen */
    ssd1306_clear();
    ssd1306_print(0, 0,  "THERMAL FSM");
    ssd1306_print(0, 16, "V0.7 INIT OK");
    ssd1306_update();
    HAL_Delay(800);

    /* 7. LAST: start the 1 Hz heartbeat. After this line, ISR can fire. */
    HAL_TIM_Base_Start_IT(&htim2);
}

void ThermalApp_Loop(void)
{
    Event_t current_event;

    /* --- STEP 1: SENSING (every loop iteration, regardless of events) --- */
    uint32_t adc_avg = adc_average(THERM_ADC_OVERSAMPLE);
    float    vnode   = THERM_ADC_VREF * ((float)adc_avg / 4095.0f);
    float    rntc    = adc_to_rntc(adc_avg);
    float    temp_c  = rntc_to_celsius(rntc);

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
            static uint32_t tick_count = 0;
            tick_count++;

            /* Control-path EMA (slow, noise-reject). SAFETY uses raw vnode. */
            // In ThermalApp_Loop, EVT_PID_TICK case:
            // OLD (always runs):
            //   pid_temp = pid_temp + 0.02f * (temp_c - pid_temp);

            // NEW (only runs when controller is alive):
            if (zone1.state == ST_PID) {
                pid_temp = pid_temp + 0.02f * (temp_c - pid_temp);
            }
        #if OPEN_LOOP_TEST_MODE
            /* Bypass FSM entirely for plant ID step test. */
            if (tick_count <= OPEN_LOOP_BASELINE_SEC) {
                g_duty_cmd = 0.0f;
            } else if (tick_count <= OPEN_LOOP_BASELINE_SEC + OPEN_LOOP_RUN_SEC) {
                g_duty_cmd = OPEN_LOOP_DUTY;
            } else {
                g_duty_cmd = 0.0f;
            }
        #else
            /* Normal path: FSM runs the show. */
            Zone_Tick(&zone1, vnode, temp_c, pid_temp);
        #endif

            /* Hard safety backstop - belt + suspenders, independent of FSM. */
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

            /* Log RAW temperature (not EMA): plant ID needs the un-filtered
             * signal so the filter can be characterised separately.        */
            /* Stream 1: dense per-tick data */
            log_sample(temp_c, pid_temp, vnode, g_duty_cmd);

            /* Stream 2: edge-detected sparse events */
            emit_change_events(&zone1);            break;
        }

        case EVT_START_CMD:
            zone1.start_req = 1;        /* future: route into Zone_Tick    */
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
            oled_update(adc_avg, vnode, rntc, ema_temp_display, duty_pct);
        }
    }

    /* No HAL_Delay - the tick paces the control path; loop runs free. */
}
