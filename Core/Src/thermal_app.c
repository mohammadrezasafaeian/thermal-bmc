/* ==========================================================================
 * thermal_app.c  --  Chip-cooling thermal controller (FreeRTOS)
 * Target : STM32F411CEU6 @ 24 MHz
 *
 * CONTROL TIERS
 *   Tier 1 (ST_PID)      : fan PID cools toward setpoint, heater = user load.
 *   Tier 2 (ST_THROTTLE) : fan pegged at 1.0, heater PID finds sustainable
 *                           load. Entered when fan authority exhausted.
 *   Tier 3 (hard backstop): over-temp -> kill heater, fan full.
 *
 * FSM:  IDLE --> PID --> THROTTLE <--> PID
 *                 |         |
 *               fault     fault
 *                 v         v
 *               FAULT --> COOLING --> IDLE
 * ========================================================================== */

#include "thermal_app.h"
#include "ssd1306.h"
#include "cyc.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ============================================================================
 * RTOS OBJECTS + FORWARD DECLARATIONS
 * ========================================================================== */
static QueueHandle_t     xCtrlQueue;
static SemaphoreHandle_t xAdcMutex;

static void ControlTask(void const *argument);
static void UITask(void const *argument);
static void emit_change_events(ZoneCtrl *z);
static void zone_all_off(ZoneCtrl *z);
static void zone_enter_cooldown(ZoneCtrl *z);
static void zone_enter_throttle(ZoneCtrl *z);

#define THERMAL_LOG_MAGIC  0xC0FFEE42u
uint32_t thermal_log_magic;

/* ── HAL handles ────────────────────────────────────────────────────────── */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern I2C_HandleTypeDef hi2c1;

/* ============================================================================
 * PUBLIC GLOBALS
 * ========================================================================== */
ZoneCtrl zone1 = {
    .name         = "Z1",
    .state        = ST_IDLE,
    .fault_reason = FR_NONE,
    .setpoint_c   = 30.0f,
    .fan_cmd      = 0.0f,
};

volatile float g_heater_duty    = 0.0f;
volatile float g_heater_request = 0.2f;
volatile float g_fan_duty       = 0.0f;
volatile float g_setpoint_c     = 30.0f;

PID_Handle pid;

ThermalLogEntry thermal_log[THERM_LOG_LEN];
ThermalEvent    thermal_events[THERM_EVENT_LEN];
volatile uint32_t thermal_log_idx;
volatile uint32_t thermal_event_idx;

volatile uint32_t dbg_adc_avg = 0;
volatile float    dbg_vnode   = 0.0f;
volatile float    dbg_rntc    = 0.0f;
volatile float    dbg_temp_c  = 0.0f;

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

static float   pid_temp         = 0.0f;

/* ============================================================================
 * SENSING HELPERS
 * ========================================================================== */
static uint32_t adc_average(uint8_t n)
{
    BaseType_t locked = (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
    if (locked) xSemaphoreTake(xAdcMutex, portMAX_DELAY);

    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    uint32_t result = sum / n;
    if (locked) xSemaphoreGive(xAdcMutex);
    return result;
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
 * ========================================================================== */
static void pwm_update(void)
{
    float h = g_heater_duty;  if (h < 0.0f) h = 0.0f;  if (h > 1.0f) h = 1.0f;
    float f = g_fan_duty;     if (f < 0.0f) f = 0.0f;  if (f > 1.0f) f = 1.0f;

    uint32_t arr3 = __HAL_TIM_GET_AUTORELOAD(&htim3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(h * (float)arr3));

    uint32_t arr5 = __HAL_TIM_GET_AUTORELOAD(&htim5);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint32_t)(f * (float)arr5));
}

/* ============================================================================
 * LOGGING
 * ========================================================================== */
static void log_sample(float temp_c, float temp_ema, float vnode,
                       float fan, float heater, float setpoint)
{
    uint32_t idx = thermal_log_idx % THERM_LOG_LEN;
    thermal_log[idx].time_s      = HAL_GetTick() * 0.001f;
    thermal_log[idx].temp_c      = temp_c;
    thermal_log[idx].temp_ema    = temp_ema;
    thermal_log[idx].vnode       = vnode;
    thermal_log[idx].fan_duty    = fan;
    thermal_log[idx].heater_duty = heater;
    thermal_log[idx].setpoint    = setpoint;
    thermal_log_idx++;
}

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

static void emit_change_events(ZoneCtrl *z)
{
    static uint8_t last_state        = 0xFF;
    static uint8_t last_fault_reason = 0xFF;
    static float   last_setpoint     = -1000.0f;

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
 * OLED RENDER
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
                        float temp_c, int heat_pct, int fan_pct)
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
        case ST_IDLE:     st_str = "IDLE"; break;
        case ST_PID:      st_str = "RUN "; break;
        case ST_THROTTLE: st_str = "THRT"; break;
        case ST_COOLING:  st_str = "COOL"; break;
        case ST_FAULT:    st_str = "FALT"; break;
    }

    if (zone1.state == ST_FAULT) {
        const char *fr = "?";
        switch (zone1.fault_reason) {
            case FR_NTC_OPEN:  fr = "NTC OPEN";  break;
            case FR_NTC_SHORT: fr = "NTC SHORT"; break;
            default:           fr = "FAULT";     break;
        }
        snprintf(buf, sizeof(buf), "%s %s", zone1.name, fr);
    } else {
        char thr = (zone1.state == ST_THROTTLE) ? '!' : ' ';
        snprintf(buf, sizeof(buf), "%s %s H%2d F%2d%c",
                 zone1.name, st_str, heat_pct, fan_pct, thr);
    }
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
 * FSM HELPERS
 * ========================================================================== */
static FaultReason detect_fault(float vnode)
{
    if (vnode > V_OPEN_THRESH)  return FR_NTC_OPEN;
    if (vnode < V_SHORT_THRESH) return FR_NTC_SHORT;
    return FR_NONE;
}

static void zone_all_off(ZoneCtrl *z)
{
    z->fan_cmd    = 0.0f;
    g_fan_duty    = 0.0f;
    g_heater_duty = 0.0f;
}

static void zone_enter_cooldown(ZoneCtrl *z)
{
    z->cool_ticks = 0;
    g_heater_duty = 0.0f;
    z->fan_cmd    = 1.0f;
    g_fan_duty    = 1.0f;
    z->state      = ST_COOLING;
}

static void zone_enter_pid(ZoneCtrl *z, float seed_temp_c)
{
    pid_temp = seed_temp_c;
    PID_Init(&z->pid, PID_KP, PID_KI, PID_KD, PID_TS, PID_TAU_F, seed_temp_c);
    z->pid.out_min     = 0.0f;
    z->pid.out_max     = 1.0f;
    z->fault_count     = 0;
    z->recover_count   = 0;
    z->throttle_count  = 0;
    g_heater_duty      = g_heater_request;
    z->state           = ST_PID;
}


static void zone_enter_throttle(ZoneCtrl *z)
{
    /* snapshot user's request and setpoint so exit logic can detect changes */
    z->mode_c_snap_request  = g_heater_request;
    z->mode_c_snap_setpoint = z->setpoint_c;

    /* peg the fan to maximum — bookkeeping AND hardware, this tick */
    z->fan_cmd = 1.0f;
    g_fan_duty = 1.0f;

    /* throttle PID: forward-acting (positive gains), seeded for bumpless
     * transfer — first output lands on the duty already in the heater */
                PID_Init(&z->pid_throttle, THROTTLE_PID_KP, THROTTLE_PID_KI, THROTTLE_PID_KD,
 PID_TS, PID_TAU_F, pid_temp);
    z->pid_throttle.out_min  = 0.0f;
    z->pid_throttle.out_max  = HEATER_MAX;
    z->pid_throttle.integral = g_heater_duty
                             - THROTTLE_PID_KP * (z->setpoint_c - pid_temp);
    z->state = ST_THROTTLE;
}
static void zone_enter_fault(ZoneCtrl *z, FaultReason r)
{
    g_heater_duty    = 0.0f;
    z->fan_cmd       = 1.0f;
    g_fan_duty       = 1.0f;
    z->fault_reason  = r;
    z->recover_count = 0;
    z->state         = ST_FAULT;
}

/* ============================================================================
 * Zone_Tick
 * ========================================================================== */
void Zone_Tick(ZoneCtrl *z, float vnode, float raw_t_c, float ema_t_c)
{
    FaultReason fault_candidate = detect_fault(vnode);

    switch (z->state) {

    case ST_IDLE:
        zone_all_off(z);
        if (z->start_req) {
            z->start_req = 0;
            if (fault_candidate == FR_NONE) zone_enter_pid(z, raw_t_c);
            else                            zone_enter_fault(z, fault_candidate);
        }
        break;

    case ST_PID:
        /* ── fault debounce ─────────────────────────────────────────── */
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
            zone_enter_cooldown(z);
            break;
        }

        /* ── Tier 1: fan PID ────────────────────────────────────────── */
        z->fan_cmd = PID_Update(&z->pid, z->setpoint_c, ema_t_c);
        if (z->fan_cmd < 0.0f) z->fan_cmd = 0.0f;
        if (z->fan_cmd > 1.0f) z->fan_cmd = 1.0f;
        g_fan_duty    = z->fan_cmd;
        g_heater_duty = g_heater_request;

        /* ── Tier 2: detect Mode C entry ────────────────────────────── */
        {
            uint8_t fan_saturated = (z->fan_cmd > THROTTLE_FAN_SAT);
            uint8_t still_hot     = (ema_t_c > z->setpoint_c + THROTTLE_MARGIN_C);

            if (fan_saturated && still_hot) {
                z->throttle_count++;
                if (z->throttle_count >= THROTTLE_ENGAGE_N) {
                    z->throttle_count = 0;
                    zone_enter_throttle(z);
                    break;              /* leave switch; we're ST_THROTTLE now */
                }
            } else {
                z->throttle_count = 0;  /* any good tick resets the debounce */
            }
        }
        break;

    case ST_THROTTLE:
        /* ── fault debounce (same as ST_PID) ────────────────────────── */
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
            zone_enter_cooldown(z);
            break;
        }

        g_fan_duty = 1.0f;

        /* control law: throttle PID finds the largest sustainable heater duty */
        g_heater_duty = PID_Update(&z->pid_throttle, z->setpoint_c, ema_t_c);

        /* exit checks */
        {
            uint8_t request_changed =
                (z->setpoint_c      != z->mode_c_snap_setpoint) ||
                (g_heater_request   != z->mode_c_snap_request);

            /* Sustainable = throttle output has climbed back up to the user's
             * request: the plant can hold setpoint at full requested load.
             * Margin guards the case where request ≈ HEATER_MAX and the
             * clamped output can only asymptote toward it. */
            uint8_t sustainable =
                (g_heater_duty >= z->mode_c_snap_request - THROTTLE_EXIT_MARGIN);

            if (request_changed || sustainable) {
                zone_enter_pid(z, ema_t_c);          /* clean re-entry        */
                z->pid.integral = g_fan_duty;        /* your bumpless seed:   */
                break;                               /* fan resumes from 1.0, */
            }                                        /* not from a cliff      */
        }
        break;

    case ST_COOLING:
        g_heater_duty = 0.0f;
        z->fan_cmd    = 1.0f;
        g_fan_duty    = 1.0f;
        z->cool_ticks++;
        {
            uint8_t cool_now  = (raw_t_c < COOL_THRESH_C);
            uint8_t sane      = (fault_candidate == FR_NONE);
            uint8_t timed_out = (z->cool_ticks >= COOLING_TIMEOUT_TICKS);
            if ((cool_now && sane) || timed_out) {
                zone_all_off(z);
                z->state = ST_IDLE;
            }
        }
        break;

    case ST_FAULT:
        g_heater_duty = 0.0f;
        z->fan_cmd    = 1.0f;
        g_fan_duty    = 1.0f;
        if (fault_candidate == FR_NONE) {
            z->recover_count++;
            if (z->recover_count >= FAULT_RECOVER_M) {
                z->fault_reason = FR_NONE;
                zone_enter_cooldown(z);
            }
        } else {
            z->recover_count = 0;
        }
        break;

    default:
        zone_enter_fault(z, FR_NONE);
        break;
    }
}

/* ============================================================================
 * CONTROL TASK
 * ========================================================================== */
static void ControlTask(void const *argument)
{
    (void)argument;
    Event_t evt;

    for (;;)
    {
        if (xQueueReceive(xCtrlQueue, &evt, portMAX_DELAY) != pdPASS) continue;

        switch (evt)
        {
        case EVT_PID_TICK:
        {
            uint32_t lat = CYC_TO_US(DWT->CYCCNT - g_prof.tick_stamp);
            g_prof.lat_last_us = lat;
            if (lat > g_prof.lat_max_us) g_prof.lat_max_us = lat;
            if (lat < g_prof.lat_min_us) g_prof.lat_min_us = lat;
            g_prof.tick_count++;

            uint32_t tc0 = DWT->CYCCNT;

            uint32_t adc_avg = adc_average(THERM_ADC_OVERSAMPLE);
            float vnode  = THERM_ADC_VREF * ((float)adc_avg / 4095.0f);
            float rntc   = adc_to_rntc(adc_avg);
            float temp_c = rntc_to_celsius(rntc);

            if (zone1.state == ST_PID || zone1.state == ST_THROTTLE)
                pid_temp = pid_temp + 0.02f * (temp_c - pid_temp);

            Zone_Tick(&zone1, vnode, temp_c, pid_temp);

            /* Tier 3 hard safety backstop */
            if (temp_c > MAX_SAFE_TEMP_C) {
                g_heater_duty = 0.0f;
                g_fan_duty    = 1.0f;
            }

            log_sample(temp_c, pid_temp, vnode, g_fan_duty, g_heater_duty,
                       zone1.setpoint_c);
            emit_change_events(&zone1);

            pwm_update();

            g_prof.ctrl_us = CYC_TO_US(DWT->CYCCNT - tc0);
            break;
        }

        case EVT_START_CMD: zone1.start_req = 1; break;
        case EVT_STOP_CMD:  zone1.stop_req  = 1; break;
        default: break;
        }
    }
}

/* ============================================================================
 * UI TASK
 * ========================================================================== */
static void UITask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t adc_avg = adc_average(THERM_ADC_OVERSAMPLE);
        float vnode  = THERM_ADC_VREF * ((float)adc_avg / 4095.0f);
        float rntc   = adc_to_rntc(adc_avg);
        float temp_c = rntc_to_celsius(rntc);

        dbg_adc_avg = adc_avg; dbg_vnode = vnode;
        dbg_rntc = rntc;       dbg_temp_c = temp_c;

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

        int heat_pct = (int)(g_heater_duty * 100.0f + 0.5f);
        int fan_pct  = (int)(g_fan_duty    * 100.0f + 0.5f);

        oled_update(adc_avg, vnode, rntc, ema_temp_display, heat_pct, fan_pct);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================================
 * ISR
 * ========================================================================== */
void ThermalApp_TickISR(void)
{
    g_prof.tick_stamp = DWT->CYCCNT;
    BaseType_t woken = pdFALSE;
    Event_t evt = EVT_PID_TICK;
    xQueueSendFromISR(xCtrlQueue, &evt, &woken);
    portYIELD_FROM_ISR(woken);
}

/* ============================================================================
 * INIT
 * ========================================================================== */
void ThermalApp_Init(void)
{
    cyc_init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

    memset(plot_buf, 0, sizeof(plot_buf));
    plot_head = 0;

    if (thermal_log_magic != THERMAL_LOG_MAGIC) {
        memset(thermal_log,    0, sizeof(thermal_log));
        memset(thermal_events, 0, sizeof(thermal_events));
        thermal_log_idx   = 0;
        thermal_event_idx = 0;
        thermal_log_magic = THERMAL_LOG_MAGIC;
    }

    uint32_t adc_raw   = adc_average(THERM_ADC_OVERSAMPLE);
    float    init_temp  = rntc_to_celsius(adc_to_rntc(adc_raw));
    pid_temp = init_temp;

    PID_Init(&zone1.pid, PID_KP, PID_KI, PID_KD, PID_TS, PID_TAU_F, init_temp);
    zone1.state          = ST_IDLE;
    zone1.fault_reason   = FR_NONE;
    zone1.fault_count    = 0;
    zone1.recover_count  = 0;
    zone1.cool_ticks     = 0;
    zone1.throttle_count = 0;

    ssd1306_clear();
    ssd1306_print(0, 0,  "CHIP COOLER");
    ssd1306_print(0, 16, "V4.0 MODE-C");
    ssd1306_update();
    HAL_Delay(800);
}

void ThermalApp_StartTasks(void)
{
    xCtrlQueue = xQueueCreate(8, sizeof(Event_t));
    xAdcMutex  = xSemaphoreCreateMutex();
    xTaskCreate(ControlTask, "Ctrl", 512, NULL, 3, NULL);
    xTaskCreate(UITask,      "UI",   512, NULL, 1, NULL);
    HAL_TIM_Base_Start_IT(&htim2);
}
