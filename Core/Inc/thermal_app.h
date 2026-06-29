/* ==========================================================================
 * thermal_app.h  --  Dual-zone thermal controller (FreeRTOS)
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
#define THERM_RTOP            330.0f    /* top resistor (ohms)                */
#define THERM_DIV_VSUP        5.0f      /* divider supply (V)                 */
#define THERM_ADC_VREF        3.3f      /* ADC reference (V)                  */
#define THERM_BETA            3500.0f   /* NTC Beta (K), approx for 10D-9     */
#define THERM_R0              10.0f     /* NTC R at T0 (ohms)                 */
#define THERM_T0_K            298.15f   /* T0 in Kelvin (25 C)                */

/* ============================================================================
 * SAMPLING / DISPLAY CONSTANTS
 * ========================================================================== */
#define THERM_ADC_OVERSAMPLE  128
#define THERM_LOG_LEN         2048
#define THERM_PLOT_LEN        128
#define THERM_OLED_DIVIDER    1         /* legacy; UITask now paces via vTaskDelay */

/* Plot auto-scaling */
#define THERM_PLOT_K          3.0f      /* half-span = K * ema_dev            */
#define THERM_PLOT_MIN_SPAN   2.0f      /* never zoom in tighter than this    */

/* Display-side EMAs (slow, for human-readable plot) */
#define THERM_EMA_DISPLAY     0.20f
#define THERM_EMA_MEAN        0.02f
#define THERM_EMA_DEV         0.02f

/* ============================================================================
 * PID GAINS (SIMC from measured plant: K~17, tau~111, theta~10)
 * ========================================================================== */
#define PID_KP                	−0.68
#define PID_KI                −0.0061
#define PID_KD                0.0f
#define PID_TS                1.0f      /* tick period (s)                    */
#define PID_TAU_F             10.0f     /* derivative filter time const       */

/* ============================================================================
 * OPEN-LOOP STEP TEST  (set MODE=0 for normal closed-loop)
 * ========================================================================== */
#define OPEN_LOOP_TEST_MODE       0
#define OPEN_LOOP_DUTY            0.5f
#define OPEN_LOOP_BASELINE_SEC    100
#define OPEN_LOOP_RUN_SEC         1300

/* ============================================================================
 * SAFETY
 *   MAX_SAFE_TEMP_C  : real overtemperature (sensor TRUSTED, plant too hot)
 *   FR_NTC_SHORT     : sensor LYING about being hot (different root cause)
 *   These are distinct and demand different responses - do not merge them.
 * ========================================================================== */
#define MAX_SAFE_TEMP_C       80.0f

/* ============================================================================
 * STATE MACHINE - TYPES
 * ========================================================================== */
typedef enum {
    ST_IDLE    = 0,   /* actuators off, sensors live, waiting for start_req   */
    ST_PID     = 1,   /* normal closed-loop control                          */
    ST_COOLING = 2,   /* graceful shutdown: fan forced on until cool & sane   */
    ST_FAULT   = 3    /* sensor lying or hardware fault; safe-park            */
} ThermalState;

typedef enum {
    FR_NONE             = 0,
    FR_NTC_OPEN         = 1,   /* V_node hugs top rail  -> NTC disconnected   */
    FR_NTC_SHORT        = 2,   /* V_node ~ 0 V          -> NTC shorted        */
    FR_HEATER_OPEN      = 3,   /* Topic 11 hook - needs current shunt         */
    FR_FAN_OPEN         = 4,   /* Topic 11 hook                               */
    FR_ALL_DISCONNECTED = 5
} FaultReason;

/* ============================================================================
 * VOLTAGE-BASED FAULT THRESHOLDS
 *   Normal V_node = 75..147 mV (hugs GND, far from 3.3 V rail).
 *   OPEN  : ~3.3 V (clamp). Detection margin huge.
 *   SHORT : ~0 V. Detection margin TIGHT.
 * ========================================================================== */
#define V_OPEN_THRESH         2.5f      /* > this  -> NTC open                */
#define V_SHORT_THRESH        0.05f     /* < this  -> NTC short               */

/* ============================================================================
 * FAULT DEBOUNCE  (time-domain Schmitt; integer count on RAW signal, not EMA)
 *   M >> N by design: hysteresis-in-time defeats limit-cycle chatter.
 * ========================================================================== */
#define FAULT_TRIP_N          3         /* faulty ticks to trip -> FAULT      */
#define FAULT_RECOVER_M       10        /* sane ticks to recover -> IDLE      */

/* ============================================================================
 * COOLING parameters
 * ========================================================================== */
#define COOL_THRESH_C            35.0f
#define COOLING_TIMEOUT_TICKS    400    /* ~6.5 min at 1 Hz (3 * tau_eff)     */

/* ============================================================================
 * ZoneCtrl - the unit of replication
 *   One instance per physical zone. All per-zone state lives here so:
 *     - Zone 1 FAULT cannot halt Zone 2 (fault isolation)
 *     - PID + debounce counters reset cleanly on entry (bumpless)
 *     - trivially becomes a remote node over a bus later
 * ========================================================================== */
typedef struct {
    const char     *name;            /* "Z1", "Z2" - for OLED/logs            */
    ThermalState    state;
    FaultReason     fault_reason;    /* committed fault (latched until recover)*/
    uint8_t         fault_count;     /* debounce: ticks signal looked faulty  */
    uint8_t         recover_count;   /* debounce: ticks signal looked sane    */
    uint16_t        cool_ticks;      /* COOLING timeout counter               */

    /* Command flags - written by UI/comms, consumed by FSM.
     * volatile: writes can come from any context (ISR, debugger, task).      */
    volatile uint8_t start_req;
    volatile uint8_t stop_req;
    volatile uint8_t ack_req;        /* future: clear a sticky fault          */

    /* Control state */
    PID_Handle      pid;
    float           setpoint_c;
    float           duty_cmd;        /* PID output [-1..+1], mirrored global  */
    float           fan_cmd;        /* PID output [-1..+1], mirrored global  */

} ZoneCtrl;

/* ============================================================================
 * PUBLIC GLOBALS  (defined in thermal_app.c)
 * ========================================================================== */
extern ZoneCtrl zone1;

extern volatile float g_duty_cmd;
extern volatile float g_setpoint_c;     /* kept for legacy Live Expressions   */
extern volatile float g_fan_duty;
extern PID_Handle pid;                  /* legacy alias                       */

/* ============================================================================
 * DUAL LOG STREAMS
 *   Stream 1 (DENSE)  - one entry per tick (continuous physics).
 *   Stream 2 (SPARSE) - one entry per CHANGE (edge-detected events).
 * ========================================================================== */

/* ---- Stream 1: per-tick time series --------------------------------------*/
typedef struct {
    float time_s;
    float temp_c;       /* raw temperature                    */
    float temp_ema;     /* filtered temp (PID input)          */
    float vnode;        /* sensor voltage                     */
    float fan_duty;     /* CONTROLLED cooling [0,1]           */
    float heater_duty;  /* open-loop chip load [0,1]          */
    float setpoint;     /* live target                        */
} ThermalLogEntry;      /* now 28 bytes (was 20)              */

/* ---- Stream 2: event kinds -----------------------------------------------*/
typedef enum {
    EV_NONE          = 0,
    EV_STATE_CHANGE  = 1,   /* u8_payload = new ThermalState                 */
    EV_FAULT_RAISED  = 2,   /* u8_payload = FaultReason                      */
    EV_FAULT_CLEARED = 3,   /* u8_payload = previous FaultReason             */
    EV_SETPOINT_CHG  = 4,   /* f_payload  = new setpoint (C)                 */
    EV_START_REQ     = 5,
    EV_STOP_REQ      = 6,
} EventKind;

typedef struct {
    float    time_s;
    uint8_t  kind;          /* EventKind                                     */
    uint8_t  u8_payload;
    uint16_t _pad;          /* keep struct 12-byte, naturally aligned        */
    float    f_payload;
} ThermalEvent;

#define THERM_EVENT_LEN     64      /* 64 x 12 = 768 B                       */

/* ---- Globals (defined in thermal_app.c) ----------------------------------*/
extern ThermalLogEntry thermal_log[THERM_LOG_LEN];
extern volatile uint32_t thermal_log_idx;

extern ThermalEvent thermal_events[THERM_EVENT_LEN];
extern volatile uint32_t thermal_event_idx;

/* Debug read-back (Live Expressions) */
extern volatile uint32_t dbg_adc_avg;
extern volatile float    dbg_vnode;
extern volatile float    dbg_rntc;
extern volatile float    dbg_temp_c;
extern volatile float g_heater_duty;
/* ============================================================================
 * PUBLIC API  (FreeRTOS)
 *   ThermalApp_Init        : seed PID/EMA, PWM safe state, splash. Call ONCE
 *                            before the scheduler starts.
 *   ThermalApp_StartTasks  : create queue + ControlTask + UITask, then start
 *                            the TIM2 heartbeat. Call ONCE before osKernelStart.
 *   ThermalApp_TickISR     : called from the TIM2 period-elapsed callback;
 *                            posts EVT_PID_TICK to the control queue.
 *   Zone_Tick              : the FSM for one zone (the unit of replication).
 * ========================================================================== */
void ThermalApp_Init(void);
void ThermalApp_StartTasks(void);
void ThermalApp_TickISR(void);

/* Tick the FSM. Called from ControlTask on EVT_PID_TICK with all signals:
 *   vnode    : RAW node voltage  (safety path - no EMA lag)
 *   raw_t_c  : RAW temperature   (log, threshold checks)
 *   ema_t_c  : EMA-filtered temp (control path - feeds PID)                  */
void Zone_Tick(ZoneCtrl *z, float vnode, float raw_t_c, float ema_t_c);

#endif /* INC_THERMAL_APP_H_ */
