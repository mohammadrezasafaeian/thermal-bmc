/* ==========================================================================
 * thermal_app.h  --  Chip-cooling thermal controller (FreeRTOS)
 * Target : STM32F411CEU6 @ 24 MHz  (HSI/PLL, verified via TIM2 cross-check)
 * ========================================================================== */
#ifndef INC_THERMAL_APP_H_
#define INC_THERMAL_APP_H_

#include "stm32f4xx_hal.h"
#include "pid.h"
#include <stdint.h>

/* ============================================================================
 * HARDWARE / DIVIDER CONSTANTS
 *   Divider: 5V -- R_top(330R) -- NODE -- NTC(~10R cold) -- GND
 *   ADC measures V_node with VREF = 3.3 V.
 *   Temperature rises -> R_ntc falls -> V_node falls (closer to 0 V).
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
#define THERM_OLED_DIVIDER    1

/* Plot auto-scaling */
#define THERM_PLOT_K          3.0f
#define THERM_PLOT_MIN_SPAN   2.0f

/* Display-side EMAs */
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
 *   Extracted from earlier heater open-loop step test.
 *   Tuned with fan NOT pegged — retune on hardware with fan at 100%.
 * ========================================================================== */
#define THROTTLE_PID_KP       0.10f
#define THROTTLE_PID_KI       0.0008f
#define THROTTLE_PID_KD       0.0f

/* ============================================================================
 * OPEN-LOOP STEP TEST  (set MODE=0 for normal closed-loop)
 * ========================================================================== */
#define OPEN_LOOP_TEST_MODE       0
#define OPEN_LOOP_DUTY            0.5f
#define OPEN_LOOP_BASELINE_SEC    100
#define OPEN_LOOP_RUN_SEC         1300

/* ============================================================================
 * SAFETY
 * ========================================================================== */
#define MAX_SAFE_TEMP_C       80.0f

/* ============================================================================
 * STATE MACHINE
 * ========================================================================== */
typedef enum {
    ST_IDLE     = 0,
    ST_PID      = 1,   /* fan PID active, heater = user request             */
    ST_THROTTLE = 2,   /* Mode C: fan pegged, heater PID finds sustainable  */
    ST_COOLING  = 3,
    ST_FAULT    = 4
} ThermalState;

typedef enum {
    FR_NONE             = 0,
    FR_NTC_OPEN         = 1,
    FR_NTC_SHORT        = 2,
    FR_HEATER_OPEN      = 3,
    FR_FAN_OPEN         = 4,
    FR_ALL_DISCONNECTED = 5
} FaultReason;

/* ============================================================================
 * VOLTAGE-BASED FAULT THRESHOLDS
 * ========================================================================== */
#define V_OPEN_THRESH         2.5f
#define V_SHORT_THRESH        0.05f

/* ============================================================================
 * FAULT DEBOUNCE
 * ========================================================================== */
#define FAULT_TRIP_N          3
#define FAULT_RECOVER_M       10

/* ============================================================================
 * COOLING / THROTTLE PARAMETERS
 * ========================================================================== */
#define COOL_THRESH_C            35.0f
#define COOLING_TIMEOUT_TICKS    400

#define THROTTLE_FAN_SAT       0.98f   /* fan "saturated" above this         */
#define THROTTLE_MARGIN_C      0.3f    /* temp must exceed SP by this        */
#define THROTTLE_ENGAGE_N      5       /* ticks before Mode C engages        */
#define HEATER_MAX             1.0f
#define THROTTLE_EXIT_MARGIN   0.02f   /* duty headroom for Mode C exit      */
#define FAN_STALL_RPM 300  /* Fault threshold: min speed is ~1200 RPM */
#define NUM_REMOTE_NODES 3

/* ============================================================================
 * ZoneCtrl - the unit of replication
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
typedef struct {
    const char     *name;
    ThermalState    state;
    FaultReason     fault_reason;
    uint8_t         fault_count;
    uint8_t         recover_count;
    uint16_t        cool_ticks;
    uint16_t        throttle_count;    /* engage delay counter (in ST_PID)   */

    volatile uint8_t start_req;
    volatile uint8_t stop_req;
    volatile uint8_t ack_req;

    /* Fan control (ST_PID) */
    PID_Handle      pid;
    float           setpoint_c;
    float           fan_cmd;           /* fan PID output [0,1]               */

    /* Heater throttle control (ST_THROTTLE) */
    PID_Handle      pid_throttle;
    float           mode_c_snap_request;
    float           mode_c_snap_setpoint;
} ZoneCtrl;

/* ============================================================================
 * PUBLIC GLOBALS
 * ========================================================================== */
extern ZoneCtrl zone1;

extern volatile float g_heater_duty;
extern volatile float g_heater_request;
extern volatile float g_fan_duty;
extern volatile float g_setpoint_c;
extern PID_Handle pid;

/* ============================================================================
 * DUAL LOG STREAMS
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

extern volatile uint32_t dbg_adc_avg;
extern volatile float    dbg_vnode;
extern volatile float    dbg_rntc;
extern volatile float    dbg_temp_c;



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
void Zone_Tick(ZoneCtrl *z, float vnode, float raw_t_c, float ema_t_c, uint32_t fan_rpm);
#endif /* INC_THERMAL_APP_H_ */
