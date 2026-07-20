/* ==========================================================================
 * thermal_app.h  --  Chip-cooling thermal controller (FreeRTOS)
 * Target : STM32F411CEU6 @ 24 MHz
 * PROJECT 2 : DISTRIBUTED I2C BMC ARCHITECTURE
 * ========================================================================== */
#ifndef INC_THERMAL_APP_H_
#define INC_THERMAL_APP_H_

#include "stm32f4xx_hal.h"
#include "pid.h"
#include <stdint.h>

#define NUM_REMOTE_NODES 3

/* ============================================================================
 * HARDWARE / DIVIDER CONSTANTS
 * ========================================================================== */
#define THERM_RTOP            330.0f
#define THERM_DIV_VSUP        5.0f
#define THERM_ADC_VREF        3.3f
#define THERM_BETA            3500.0f
#define THERM_R0              10.0f
#define THERM_T0_K            298.15f

/* ============================================================================
 * SAMPLING / DISPLAY CONSTANTS
 * ========================================================================== */
#define THERM_ADC_OVERSAMPLE  128
#define THERM_LOG_LEN         2048
#define THERM_PLOT_LEN        128

#define THERM_PLOT_K          3.0f
#define THERM_PLOT_MIN_SPAN   2.0f

#define THERM_EMA_DISPLAY     0.20f
#define THERM_EMA_MEAN        0.02f
#define THERM_EMA_DEV         0.02f

/* ============================================================================
 * FAN PID GAINS  (reverse-acting, SIMC: K~8.2, tau~111, theta~10, lambda=10)
 * ========================================================================== */
#define PID_KP               -0.68f
#define PID_KI               -0.0061f
#define PID_KD                0.0f
#define PID_TS                1.0f
#define PID_TAU_F             10.0f

/* ============================================================================
 * THROTTLE (MODE C) HEATER PID GAINS  (forward-acting, positive)
 * ========================================================================== */
#define THROTTLE_PID_KP       0.10f
#define THROTTLE_PID_KI       0.0008f
#define THROTTLE_PID_KD       0.0f

/* ============================================================================
 * SAFETY
 * ========================================================================== */
#define MAX_SAFE_TEMP_C       80.0f

/* ============================================================================
 * STATE MACHINE & FAULTS
 * ========================================================================== */
typedef enum {
    ST_IDLE     = 0,
    ST_PID      = 1,
    ST_THROTTLE = 2,
    ST_COOLING  = 3,
    ST_FAULT    = 4
} ThermalState;

typedef enum {
    FR_NONE             = 0,
    FR_NTC_OPEN         = 1,
    FR_NTC_SHORT        = 2,
    FR_HEATER_OPEN      = 3,
    FR_FAN_OPEN         = 4,
    FR_ALL_DISCONNECTED = 5,
    FR_NODE_OFFLINE     = 6
} FaultReason;

/* NTC fault thresholds, count domain (rail-immune, signed counts).
 * Anchors: short=-151, 80C=-126 (gap 25) ; open=+511 SAT, 0C=+267.
 * PLACEMENT + JUSTIFICATION: ___[#12 - two numbers, one sentence
 * each: where in the gap and why, vs noise + FAULT_TRIP_N]___    */
#define CNT_NTC_SHORT_THRESH  (-140)   /* TUTOR PLACEHOLDER - ratify or move */
#define CNT_NTC_OPEN_THRESH   (+400)   /* TUTOR PLACEHOLDER - ratify or move */
#define FAN_STALL_RPM         300

#define FAULT_TRIP_N          3
#define FAULT_RECOVER_M       10

#define COOL_THRESH_C            35.0f
#define COOLING_TIMEOUT_TICKS    400

#define THROTTLE_FAN_SAT       0.98f
#define THROTTLE_MARGIN_C      0.3f
#define THROTTLE_ENGAGE_N      5
#define HEATER_MAX             1.0f
#define THROTTLE_EXIT_MARGIN   0.02f

/* ============================================================================
 * DISTRIBUTED DATA CONTRACTS (I2C)
 * ========================================================================== */
/* Sent from ATmega -> STM32 (Master Read, 4 bytes) */
typedef struct {
    uint16_t adc_raw;
    uint16_t tach_pulses;
} I2C_Telemetry;

/* Sent from STM32 -> ATmega (Master Write, 2 bytes) */
typedef struct {
    uint8_t heater_pwm;
    uint8_t fan_pwm;
} I2C_Command;

/* The Shared Memory Object */
typedef struct {
    I2C_Telemetry tel;
    I2C_Command   cmd;
    uint8_t       is_online;  /* 1 = healthy, 0 = unplugged/failed */
} RemoteNode;

/* ============================================================================
 * ZoneCtrl - the unit of replication
 * ========================================================================== */
typedef struct {
    const char     *name;
    ThermalState    state;
    FaultReason     fault_reason;
    uint8_t         fault_count;
    uint8_t         recover_count;
    uint16_t        cool_ticks;
    uint16_t        throttle_count;

    volatile uint8_t start_req;
    volatile uint8_t stop_req;
    volatile uint8_t ack_req;

    /* Per-Zone State & Data */
    float           pid_temp;              /* Local EMA temperature */
    float           setpoint_c;
    float           requested_heater_duty; /* User's simulated load */

    /* Actuator Output State */
    float           fan_cmd;
    float           fan_duty;              /* Actual physical fan output */
    float           heater_duty;           /* Actual physical heater output */

    /* Fan control (ST_PID) */
    PID_Handle      pid;

    /* Heater throttle control (ST_THROTTLE) */
    PID_Handle      pid_throttle;
    float           mode_c_snap_request;
    float           mode_c_snap_setpoint;

} ZoneCtrl;

/* ============================================================================
 * PUBLIC GLOBALS
 * ========================================================================== */
extern ZoneCtrl   zones[NUM_REMOTE_NODES];
extern RemoteNode g_nodes[NUM_REMOTE_NODES];

/* ============================================================================
 * DUAL LOG STREAMS (Tracking Zone 0 for Demo)
 * ========================================================================== */
typedef struct {
    float time_s;
    float temp_c;
    float temp_ema;
    float vnode;
    float fan_duty;
    float heater_duty;
    float heater_req;
    float setpoint;
    uint32_t fan_rpm;
} ThermalLogEntry;

typedef enum {
    EV_NONE          = 0,
    EV_STATE_CHANGE  = 1,
    EV_FAULT_RAISED  = 2,
    EV_FAULT_CLEARED = 3,
    EV_SETPOINT_CHG  = 4,
    EV_START_REQ     = 5,
    EV_STOP_REQ      = 6,
} EventKind;

typedef struct {
    float    time_s;
    uint8_t  kind;
    uint8_t  u8_payload;
    uint16_t _pad;
    float    f_payload;
} ThermalEvent;

#define THERM_EVENT_LEN     64

extern ThermalLogEntry thermal_log[THERM_LOG_LEN];
extern volatile uint32_t thermal_log_idx;

extern ThermalEvent thermal_events[THERM_EVENT_LEN];
extern volatile uint32_t thermal_event_idx;


/* ============================================================================
 * RTOS EVENTS
 * ========================================================================== */
typedef enum {
    EVT_PID_TICK  = 0,
    EVT_START_CMD = 1,
    EVT_STOP_CMD  = 2,
} Event_t;

/* ============================================================================
 * PUBLIC API
 * ========================================================================== */
void ThermalApp_Init(void);
void ThermalApp_StartTasks(void);
void ThermalApp_TickISR(void);
void Zone_Tick(ZoneCtrl *z, int counts, float raw_t_c, float ema_t_c, uint32_t fan_rpm);

#endif /* INC_THERMAL_APP_H_ */
