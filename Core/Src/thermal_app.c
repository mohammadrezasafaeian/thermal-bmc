/* ==========================================================================
 * thermal_app.c  --  Chip-cooling thermal controller (FreeRTOS)
 * Target : STM32F411CEU6 @ 24 MHz
 *
 * PROJECT 2: DISTRIBUTED I2C BMC ARCHITECTURE
 *   - Local STM32: Brains (RTOS, PID, UI, I2C Master)
 *   - Remote ATmegas: Muscle (Sensors, PWM Actuators)
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
SemaphoreHandle_t        xNodeMutex;

static void ControlTask(void *argument);
static void UITask(void *argument);
static void BusTask(void *argument);

static void emit_change_events(ZoneCtrl *z);
static void zone_all_off(ZoneCtrl *z);
static void zone_enter_cooldown(ZoneCtrl *z);
static void zone_enter_throttle(ZoneCtrl *z);
static void zone_enter_fault(ZoneCtrl *z, FaultReason r);
static void zone_enter_pid(ZoneCtrl *z, float seed_temp_c);

#define THERMAL_LOG_MAGIC  0xC0FFEE42u
uint32_t thermal_log_magic;

/* ── HAL handles ────────────────────────────────────────────────────────── */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern I2C_HandleTypeDef hi2c1;

/* ============================================================================
 * PUBLIC GLOBALS (DISTRIBUTED)
 * ========================================================================== */
RemoteNode g_nodes[NUM_REMOTE_NODES];
ZoneCtrl   zones[NUM_REMOTE_NODES];

ThermalLogEntry thermal_log[THERM_LOG_LEN];
ThermalEvent    thermal_events[THERM_EVENT_LEN];
volatile uint32_t thermal_log_idx;
volatile uint32_t thermal_event_idx;

volatile Profiler g_prof = { .lat_min_us = 0xFFFFFFFFu };

/* ============================================================================
 * PRIVATE STATE (UI)
 * ========================================================================== */
static float   plot_buf[THERM_PLOT_LEN];
static uint8_t plot_head = 0;

static float   ema_temp_display = 0.0f;
static float   ema_mean         = 0.0f;
static float   ema_dev          = 1.0f;
static uint8_t ema_initialized  = 0;

/* ============================================================================
 * SENSING HELPERS (Adapted for 5V ATmega ADC over I2C)
 * ========================================================================== */
static float adc_to_rntc_5v(float vnode)
{
    if (vnode < 0.001f) vnode = 0.001f;
    float v_max = THERM_DIV_VSUP - 0.001f;
    if (vnode > v_max) vnode = v_max;

    float rntc = THERM_RTOP * (vnode / (THERM_DIV_VSUP - vnode));
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
 * LOGGING (Monitoring Node 0)
 * ========================================================================== */
static void log_sample(float temp_c, float temp_ema, float vnode,
                       float fan, float heater, float heater_req, float setpoint, uint32_t fan_rpm)
{
    uint32_t idx = thermal_log_idx % THERM_LOG_LEN;
    thermal_log[idx].time_s      = HAL_GetTick() * 0.001f;
    thermal_log[idx].temp_c      = temp_c;
    thermal_log[idx].temp_ema    = temp_ema;
    thermal_log[idx].vnode       = vnode;
    thermal_log[idx].fan_duty    = fan;
    thermal_log[idx].heater_duty = heater;
    thermal_log[idx].heater_req  = heater_req;
    thermal_log[idx].setpoint    = setpoint;
    thermal_log[idx].fan_rpm     = fan_rpm;
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
    /* To track multiple zones properly, these statics would need to be per-zone,
     * but for the single-zone portoflio logger, we track Zone 0. */
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
 * FSM HELPERS (Fully Encapsulated)
 * ========================================================================== */
static FaultReason detect_fault(float vnode, uint32_t fan_rpm){
    if (vnode > V_OPEN_THRESH)  return FR_NTC_OPEN;
    if (vnode < V_SHORT_THRESH) return FR_NTC_SHORT;
    if (fan_rpm < FAN_STALL_RPM) return FR_FAN_OPEN;
    return FR_NONE;
}

static void zone_all_off(ZoneCtrl *z)
{
    z->fan_cmd     = 0.0f;
    z->fan_duty    = 0.0f;
    z->heater_duty = 0.0f;
}

static void zone_enter_cooldown(ZoneCtrl *z)
{
    z->cool_ticks  = 0;
    z->heater_duty = 0.0f;
    z->fan_cmd     = 1.0f;
    z->fan_duty    = 1.0f;
    z->state       = ST_COOLING;
}

static void zone_enter_pid(ZoneCtrl *z, float seed_temp_c)
{
    z->pid_temp = seed_temp_c;
    PID_Init(&z->pid, PID_KP, PID_KI, PID_KD, PID_TS, PID_TAU_F, seed_temp_c);
    z->pid.out_min     = 0.0f;
    z->pid.out_max     = 1.0f;
    z->fault_count     = 0;
    z->recover_count   = 0;
    z->throttle_count  = 0;
    z->heater_duty     = z->requested_heater_duty;
    z->state           = ST_PID;
}

static void zone_enter_throttle(ZoneCtrl *z)
{
    /* Snapshot FSM Memory */
    z->mode_c_snap_request  = z->requested_heater_duty;
    z->mode_c_snap_setpoint = z->setpoint_c;

    z->fan_cmd  = 1.0f;
    z->fan_duty = 1.0f;

    PID_Init(&z->pid_throttle, THROTTLE_PID_KP, THROTTLE_PID_KI, THROTTLE_PID_KD, PID_TS, PID_TAU_F, z->pid_temp);
    z->pid_throttle.out_min  = 0.0f;
    z->pid_throttle.out_max  = HEATER_MAX;

    /* TRAP 3 FIXED: Seeded bumplessly from the plant's actual physical output at t-1 */
    z->pid_throttle.integral = z->heater_duty - THROTTLE_PID_KP * (z->setpoint_c - z->pid_temp);
    z->state = ST_THROTTLE;
}

static void zone_enter_fault(ZoneCtrl *z, FaultReason r)
{
    z->heater_duty   = 0.0f;
    z->fan_cmd       = 1.0f;
    z->fan_duty      = 1.0f;
    z->fault_reason  = r;
    z->recover_count = 0;
    z->state         = ST_FAULT;
}

/* ============================================================================
 * Zone_Tick (The Universal Engine)
 * ========================================================================== */
void Zone_Tick(ZoneCtrl *z, float vnode, float raw_t_c, float ema_t_c, uint32_t fan_rpm){
    FaultReason fault_candidate = detect_fault(vnode, fan_rpm);

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

        /* Tier 1: fan PID */
        z->fan_cmd = PID_Update(&z->pid, z->setpoint_c, ema_t_c);
        if (z->fan_cmd < 0.0f) z->fan_cmd = 0.0f;
        if (z->fan_cmd > 1.0f) z->fan_cmd = 1.0f;
        z->fan_duty    = z->fan_cmd;
        z->heater_duty = z->requested_heater_duty;

        /* Tier 2: detect Mode C entry */
        {
            uint8_t fan_saturated = (z->fan_cmd > THROTTLE_FAN_SAT);
            uint8_t still_hot     = (ema_t_c > z->setpoint_c + THROTTLE_MARGIN_C);

            if (fan_saturated && still_hot) {
                z->throttle_count++;
                if (z->throttle_count >= THROTTLE_ENGAGE_N) {
                    z->throttle_count = 0;
                    zone_enter_throttle(z);
                    break;
                }
            } else {
                z->throttle_count = 0;
            }
        }
        break;

    case ST_THROTTLE:
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

        z->fan_duty = 1.0f;
        z->heater_duty = PID_Update(&z->pid_throttle, z->setpoint_c, ema_t_c);

        /* exit checks */
        {
            uint8_t request_changed =
                (z->setpoint_c             != z->mode_c_snap_setpoint) ||
                (z->requested_heater_duty  != z->mode_c_snap_request);

            uint8_t recovered = (ema_t_c <= z->setpoint_c);
            uint8_t sustainable = (z->heater_duty >= z->mode_c_snap_request - THROTTLE_EXIT_MARGIN) && recovered;

            if (request_changed || sustainable) {
                zone_enter_pid(z, ema_t_c);
                z->pid.integral = z->fan_duty; /* bumpless reverse */
                break;
            }
        }
        break;

    case ST_COOLING:
        z->heater_duty = 0.0f;
        z->fan_cmd     = 1.0f;
        z->fan_duty    = 1.0f;
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
        z->heater_duty = 0.0f;
        z->fan_cmd     = 1.0f;
        z->fan_duty    = 1.0f;
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
 * CONTROL TASK (The Multi-Zone Brain)
 * ========================================================================== */
static void ControlTask(void *argument)
{
    (void)argument;
    Event_t evt;

    for (;;) {
        if (xQueueReceive(xCtrlQueue, &evt, portMAX_DELAY) != pdPASS) continue;

        if (evt == EVT_PID_TICK) {
            uint32_t tc0 = DWT->CYCCNT;

            for (int i = 0; i < NUM_REMOTE_NODES; i++) {

                I2C_Telemetry tel;
                uint8_t online;

                /* 1: Safely lock, read shared memory, unlock */
                xSemaphoreTake(xNodeMutex, portMAX_DELAY);
                tel = g_nodes[i].tel;
                online = g_nodes[i].is_online;
                xSemaphoreGive(xNodeMutex);

                float vnode = 0.0f;
                float temp_c = 0.0f;
                uint32_t fan_rpm = 0;

                if (online) {
                    /* Convert 10-bit ATmega ADC to Voltage and Temp */
                    vnode = 5.0f * ((float)tel.adc_raw / 1023.0f);
                    temp_c = rntc_to_celsius(adc_to_rntc_5v(vnode));
                    fan_rpm = tel.tach_pulses * 30;
                }

                /* Independent EMA calculation per zone */
                if (zones[i].state == ST_PID || zones[i].state == ST_THROTTLE) {
                    zones[i].pid_temp = zones[i].pid_temp + 0.02f * (temp_c - zones[i].pid_temp);
                }

                /* 2: FSM Execution */
                if (!online) {
                    zone_enter_fault(&zones[i], FR_NODE_OFFLINE);
                } else {
                    Zone_Tick(&zones[i], vnode, temp_c, zones[i].pid_temp, fan_rpm);
                }

                /* Tier 3 hard safety backstop (Per Zone) */
                if (temp_c > MAX_SAFE_TEMP_C) {
                    zones[i].heater_duty = 0.0f;
                    zones[i].fan_duty    = 1.0f;
                }

                /* 3: Safely write commands back to the I2C buffer */
                xSemaphoreTake(xNodeMutex, portMAX_DELAY);
                g_nodes[i].cmd.heater_pwm = (uint8_t)(zones[i].heater_duty * 255.0f);
                g_nodes[i].cmd.fan_pwm    = (uint8_t)(zones[i].fan_duty * 255.0f);
                xSemaphoreGive(xNodeMutex);

                /* For portfolio logging, we capture Zone 0's data */
                if (i == 0) {
                    log_sample(temp_c, zones[i].pid_temp, vnode, zones[i].fan_duty,
                               zones[i].heater_duty, zones[i].requested_heater_duty,
                               zones[i].setpoint_c, fan_rpm);
                    emit_change_events(&zones[i]);
                }
            }

            g_prof.ctrl_us = CYC_TO_US(DWT->CYCCNT - tc0);
        }
        else if (evt == EVT_START_CMD) { zones[0].start_req = 1; }
        else if (evt == EVT_STOP_CMD)  { zones[0].stop_req  = 1; }
    }
}

/* ============================================================================
 * BUS TASK (I2C Master)
 * ========================================================================== */
static void BusTask(void *argument)
{
    (void)argument;
    const uint16_t node_addrs[NUM_REMOTE_NODES] = {0x20, 0x22, 0x24};

    for(;;) {
        for(int i = 0; i < NUM_REMOTE_NODES; i++) {
            I2C_Telemetry temp_tel;
            I2C_Command   temp_cmd;

            xSemaphoreTake(xNodeMutex, portMAX_DELAY);
            temp_cmd = g_nodes[i].cmd;
            xSemaphoreGive(xNodeMutex);

            HAL_StatusTypeDef tx_stat = HAL_I2C_Master_Transmit(&hi2c1, node_addrs[i], (uint8_t*)&temp_cmd, sizeof(I2C_Command), 10);
            HAL_StatusTypeDef rx_stat = HAL_I2C_Master_Receive(&hi2c1, node_addrs[i], (uint8_t*)&temp_tel, sizeof(I2C_Telemetry), 10);

            if (tx_stat == HAL_OK && rx_stat == HAL_OK) {
                xSemaphoreTake(xNodeMutex, portMAX_DELAY);
                g_nodes[i].tel = temp_tel;
                g_nodes[i].is_online = 1;
                xSemaphoreGive(xNodeMutex);
            } else {
                xSemaphoreTake(xNodeMutex, portMAX_DELAY);
                g_nodes[i].is_online = 0;
                xSemaphoreGive(xNodeMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================================
 * UI TASK (Now driven by Shared Memory)
 * ========================================================================== */
static int format_fixed1(char *buf, int buf_len, float val) {
    int sign = (val < 0.0f) ? 1 : 0;
    if (sign) val = -val;
    int32_t tenths = (int32_t)(val * 10.0f + 0.5f);
    int32_t whole  = tenths / 10;
    int32_t frac   = tenths % 10;
    if (sign) return snprintf(buf, buf_len, "-%ld.%ld", (long)whole, (long)frac);
    else      return snprintf(buf, buf_len, "%ld.%ld",  (long)whole, (long)frac);
}

static void oled_update(uint32_t adc_avg, float vnode, float rntc, float temp_c, int heat_pct, int fan_pct, ZoneCtrl *z) {
    char buf[22];
    ssd1306_clear();

    char v_str[8]; format_fixed1(v_str, sizeof(v_str), vnode);
    snprintf(buf, sizeof(buf), "A:%4lu V:%s", (unsigned long)adc_avg, v_str);
    ssd1306_print(0, 0, buf);

    char r_str[8], t_str[8];
    format_fixed1(r_str, sizeof(r_str), rntc);
    format_fixed1(t_str, sizeof(t_str), temp_c);
    snprintf(buf, sizeof(buf), "R:%s T:%sC", r_str, t_str);
    ssd1306_print(0, 8, buf);

    const char *st_str = "?";
    switch (z->state) {
        case ST_IDLE:     st_str = "IDLE"; break;
        case ST_PID:      st_str = "RUN "; break;
        case ST_THROTTLE: st_str = "THRT"; break;
        case ST_COOLING:  st_str = "COOL"; break;
        case ST_FAULT:    st_str = "FALT"; break;
    }

    if (z->state == ST_FAULT) {
        const char *fr = "?";
        switch (z->fault_reason) {
            case FR_NTC_OPEN:     fr = "NTC OPEN";  break;
            case FR_NTC_SHORT:    fr = "NTC SHORT"; break;
            case FR_FAN_OPEN:     fr = "FAN OPEN";  break;
            case FR_NODE_OFFLINE: fr = "OFFLINE";   break;
            default:              fr = "FAULT";     break;
        }
        snprintf(buf, sizeof(buf), "%s %s", z->name, fr);
    } else {
        char thr = (z->state == ST_THROTTLE) ? '!' : ' ';
        snprintf(buf, sizeof(buf), "%s %s H%2d F%2d%c", z->name, st_str, heat_pct, fan_pct, thr);
    }
    ssd1306_print(0, 16, buf);
    ssd1306_draw_line(0, 20, 127, 20);

    const uint8_t PLOT_TOP = 22, PLOT_BOTTOM = 63, PLOT_H = PLOT_BOTTOM - PLOT_TOP;
    float half_span = THERM_PLOT_K * ema_dev;
    if (half_span < THERM_PLOT_MIN_SPAN / 2.0f) half_span = THERM_PLOT_MIN_SPAN / 2.0f;
    float plot_min = ema_mean - half_span;
    float span = 2.0f * half_span;

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

static void UITask(void *argument)
{
    (void)argument;
    for (;;) {
        I2C_Telemetry tel;
        uint8_t online;

        xSemaphoreTake(xNodeMutex, portMAX_DELAY);
        tel = g_nodes[0].tel;
        online = g_nodes[0].is_online;
        xSemaphoreGive(xNodeMutex);

        float temp_c = 0.0f, vnode = 0.0f, rntc = 0.0f;
        if (online) {
            vnode = 5.0f * ((float)tel.adc_raw / 1023.0f);
            rntc = adc_to_rntc_5v(vnode);
            temp_c = rntc_to_celsius(rntc);
        }

        if (!ema_initialized) {
            ema_temp_display = temp_c;
            ema_mean = temp_c;
            ema_dev = 1.0f;
            ema_initialized = 1;
        } else {
            ema_temp_display = ema_step(ema_temp_display, temp_c, THERM_EMA_DISPLAY);
            ema_mean = ema_step(ema_mean, ema_temp_display, THERM_EMA_MEAN);
            float abs_dev = ema_temp_display - ema_mean;
            if (abs_dev < 0.0f) abs_dev = -abs_dev;
            ema_dev = ema_step(ema_dev, abs_dev, THERM_EMA_DEV);
            if (ema_dev < 0.05f) ema_dev = 0.05f;
        }

        plot_buf[plot_head] = ema_temp_display;
        plot_head = (uint8_t)((plot_head + 1) % THERM_PLOT_LEN);

        int heat_pct = (int)(zones[0].heater_duty * 100.0f + 0.5f);
        int fan_pct  = (int)(zones[0].fan_duty    * 100.0f + 0.5f);

        oled_update(tel.adc_raw, vnode, rntc, ema_temp_display, heat_pct, fan_pct, &zones[0]);
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

    memset(plot_buf, 0, sizeof(plot_buf));
    plot_head = 0;

    if (thermal_log_magic != THERMAL_LOG_MAGIC) {
        memset(thermal_log,    0, sizeof(thermal_log));
        memset(thermal_events, 0, sizeof(thermal_events));
        thermal_log_idx   = 0;
        thermal_event_idx = 0;
        thermal_log_magic = THERMAL_LOG_MAGIC;
    }

    for (int i = 0; i < NUM_REMOTE_NODES; i++) {
        char name_buf[4];
        snprintf(name_buf, sizeof(name_buf), "Z%d", i);
        zones[i].name = strdup(name_buf); /* Fine for embedded if called once */
        zones[i].state = ST_IDLE;
        zones[i].fault_reason = FR_NONE;
        zones[i].setpoint_c = 30.0f;
        zones[i].requested_heater_duty = 0.2f;
        zones[i].pid_temp = 25.0f;
        PID_Init(&zones[i].pid, PID_KP, PID_KI, PID_KD, PID_TS, PID_TAU_F, 25.0f);
    }

    ssd1306_clear();
    ssd1306_print(0, 0,  "BMC MASTER");
    ssd1306_print(0, 16, "V5.0 DISTRIBUTED");
    ssd1306_update();
    HAL_Delay(800);
}

void ThermalApp_StartTasks(void)
{
    xCtrlQueue = xQueueCreate(8, sizeof(Event_t));
    xAdcMutex  = xSemaphoreCreateMutex();
    xNodeMutex = xSemaphoreCreateMutex();

    xTaskCreate(BusTask, "Bus", 512, NULL, 2, NULL);
    xTaskCreate(ControlTask, "Ctrl", 512, NULL, 3, NULL);
    xTaskCreate(UITask,      "UI",   512, NULL, 1, NULL);

    HAL_TIM_Base_Start_IT(&htim2);
}
