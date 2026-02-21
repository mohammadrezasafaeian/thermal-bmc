#ifndef OSC_CONFIG_H
#define OSC_CONFIG_H

#include <stdint.h>

/* ==================== BUFFER SIZES ==================== */
#define ADC_BUFFER_SIZE     8192    // Power of 2 for FFT efficiency
#define DISPLAY_SAMPLES     256     // Samples sent to ESP32
#define OLED_SAMPLES        128     // OLED width in pixels
#define CMD_BUFFER_SIZE     32
#define FFT_SIZE            4096

/* ==================== CLOCK CONFIG ==================== */
#define SYSTEM_CLOCK_HZ     100000000UL  // 100 MHz
#define TIM3_CLOCK_HZ       100000000UL  // PWM generator clock
#define SR_TIME_MODE_MAX    1000000UL    // 1 MSPS max
#define SR_FFT_MODE         500000UL     // 500 kSPS for FFT

/* ==================== MEASUREMENT THRESHOLDS ==================== */
#define MIN_SIGNAL_AMPLITUDE    40      // Min ADC counts for valid signal
#define ZC_HYSTERESIS_PERCENT   10      // Zero-crossing hysteresis
#define EMA_ALPHA               0.15f   // EMA filter coefficient

/* ==================== ENUMERATIONS ==================== */
typedef enum {
    MODE_NORMAL = 0,        // Direct sampling
    MODE_AVERAGE,           // Noise reduction
    MODE_PEAK_DETECT        // Transient capture
} ScopeMode;

typedef enum {
    DISPLAY_TIME = 0,       // Time domain
    DISPLAY_FREQ            // FFT spectrum
} DisplayMode;

/* ==================== DATA STRUCTURES ==================== */
typedef struct {
    uint16_t time_div_us;           // Timebase µs/div
    uint16_t volt_div_mv;           // Voltage scale (web UI)
    uint16_t generator_freq_hz;     // PWM output frequency
    uint8_t  duty_cycle_percent;    // PWM duty cycle
    uint32_t sample_rate_hz;        // Current ADC rate
    ScopeMode mode;                 // Acquisition mode
    DisplayMode display_mode;       // Time/freq domain
    uint8_t  average_count;         // FFT averaging frames
} OscSettings;

typedef struct {
    uint16_t amplitude_mv;          // Peak-to-peak
    uint32_t frequency_hz;          // Measured freq
    uint16_t period_us;             // Signal period
    uint16_t vmax_mv, vmin_mv;      // Voltage extremes
    uint16_t vrms_mv;               // RMS voltage
    uint16_t duty_percent;          // Duty cycle
    uint32_t peak_freqs[5];         // Top 5 FFT peaks (Hz)
    uint16_t peak_mags[5];          // Peak magnitudes
    uint8_t  num_peaks;             // Valid peak count
    uint8_t  valid;                 // Measurement validity
} Measurements;

/* ==================== DEFAULT SETTINGS ==================== */
#define DEFAULT_SETTINGS { \
    .time_div_us = 100,             \
    .volt_div_mv = 500,             \
    .generator_freq_hz = 1000,      \
    .duty_cycle_percent = 50,       \
    .sample_rate_hz = SR_TIME_MODE_MAX, \
    .mode = MODE_NORMAL,            \
    .display_mode = DISPLAY_TIME,   \
    .average_count = 20             \
}

#endif /* OSC_CONFIG_H */
