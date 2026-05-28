/* ==========================================================================
 * thermal_app.h  —  Thermal Logger with PID Controller
 * Target : STM32F411CEU6 "Black Pill" @ 100 MHz  (STM32CubeIDE + HAL)
 * ==========================================================================
 *
 *  SENSOR WIRING  (flipped divider for low‑Ohm NTC)
 *  ─────────────────────────────────────────────────
 *
 *   5.0 V (USB) ──┬── R_top = 330 Ω ──┬── NTC 10D‑9 (~10 Ω) ── GND
 *                  │                    │
 *                  │                   PA0 (ADC1_IN0)
 *                  │                   (NODE voltage)
 *
 *   As temperature ↑  →  R_ntc ↓  →  V_node ↑  (approaches 3.3 V)
 *   The ADC reference is 3.3 V; VSUP is 5.0 V.
 *
 *  HEATER WIRING
 *  ─────────────
 *   PA6 (TIM3_CH1 PWM) ──► IRLZ44N gate ──► 10 Ω / 5 W ──► USB 5 V
 *
 *  REQUIRED CubeMX SETTINGS  (configure BEFORE using this module)
 *  ──────────────────────────────────────────────────────────────
 *   ADC1  : Channel IN0 (PA0), 12‑bit, right‑align, software trigger,
 *            single‑conversion mode, NO DMA, scan disabled.
 *
 *   TIM3  : Prescaler = 99, Counter Period = 999, no reload preload,
 *            Channel 1 = PWM Generation CH1, PA6 assigned to TIM3_CH1.
 *            → PWM frequency = 100 MHz / 100 / 1000 = 1 kHz
 *
 *   I2C1  : Already configured for the SSD1306 (keep as‑is).
 *
 * ========================================================================== */
/* ==========================================================================
 * thermal_app.h  —  Thermal Logger configuration & public API
 * ========================================================================== */

#ifndef THERMAL_APP_H
#define THERMAL_APP_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "pid.h"                // PID controller (your tuned gains)

/* ── Divider constants ──────────────────────────────────────────────────────
 *
 *  Wiring: VSUP (5.0 V) → R_top (330 Ω) → NODE (PA0) → NTC (10D‑9) → GND
 *
 *  THERM_ADC_VREF  – voltage the ADC uses to map counts → volts (3.3 V)
 *  THERM_DIV_VSUP  – actual supply rail of the divider (5.0 V from USB)
 *  THERM_RTOP      – top resistor value (Ω)
 *
 *  To power the divider from 3.3 V instead (recommended for MCU safety):
 *    #define THERM_DIV_VSUP   3.3f
 *  No other code changes are needed.                                        */
#define THERM_ADC_VREF      3.3f
#define THERM_DIV_VSUP      5.0f
#define THERM_RTOP          330.0f

/* ── NTC Beta model ─────────────────────────────────────────────────────── */
#define THERM_R0            10.0f       /* Nominal NTC resistance @ T0 (Ω)  */
#define THERM_T0_K          298.15f     /* Reference temperature (25 °C)     */
#define THERM_BETA          3500.0f     /* Beta coefficient (K) — recalibrate*/

/* ── Sampling ───────────────────────────────────────────────────────────── */
#define THERM_ADC_OVERSAMPLE  128        /* Conversions averaged per iteration*/
#define THERM_SAMPLE_MS       1000       /* Target loop period (ms) → 1 Hz   */

/* ── EMA filter coefficients (0 < alpha ≤ 1; smaller = smoother) ────────── */
#define THERM_EMA_DISPLAY   0.05f        /* Display/log smoothing             */
#define THERM_EMA_MEAN      0.01f        /* Slow mean tracker (plot centre)   */
#define THERM_EMA_DEV       0.05f        /* Mean‑abs‑deviation (plot scale)   */

/* ── OLED plot ──────────────────────────────────────────────────────────── */
#define THERM_PLOT_LEN      128          /* Ring‑buffer length = OLED width   */
#define THERM_PLOT_K        3.0f         /* Half‑span multiplier (× ema_dev)  */
#define THERM_PLOT_MIN_SPAN 2.0f         /* Minimum full‑scale span (°C)      */
#define THERM_OLED_DIVIDER  1            /* Refresh OLED every N samples      */

/* ── RAM log ────────────────────────────────────────────────────────────── */
#define THERM_LOG_LEN       2048         /* Entries; 2048 × 12 B = 24 576 B   */

/* ── Log entry ──────────────────────────────────────────────────────────── */
typedef struct {
    float time_s;
    float temp_c;
    float setpoint_c;    /* ← add this */
    float heater_duty;
    float fan_duty;
} ThermalLogEntry;

/* ── Public globals (accessible from debugger / main.c) ─────────────────── */
extern volatile float         g_duty_cmd;       /* PID output / manual duty  */
extern volatile float         g_setpoint_c;     /* Desired temperature (°C)  */
extern          ThermalLogEntry thermal_log[THERM_LOG_LEN];
extern volatile uint32_t      thermal_log_idx;
extern PID_Handle             pid;              /* PID state – watch integral,deriv */
extern volatile float g_fan_duty;   /* Manual fan control (0.0–1.0)  */
/* ── Public API ─────────────────────────────────────────────────────────── */
void ThermalApp_Init(void);
void ThermalApp_Loop(void);

#endif /* THERMAL_APP_H */
