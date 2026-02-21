#include "osc_signal.h"
#include <string.h>
#include <math.h>

/* ==================== BUFFERS ==================== */
uint16_t adc_buffer[ADC_BUFFER_SIZE] __attribute__((aligned(4)));
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t fft_accumulator[FFT_SIZE/2];
arm_rfft_fast_instance_f32 fft_instance;
uint8_t fft_frame_count = 0;

/* ==================== EMA FILTER STATE ==================== */
static struct {
    float32_t freq, amp, rms, duty, period;
    uint8_t valid_count;
} meas_filter = {0};

/* ==================== HELPER: EMA UPDATE ==================== */
static inline void ema_update(float32_t *state, float32_t value, uint8_t init) {
    *state = init ? value : (*state + EMA_ALPHA * (value - *state));
}

/* ==================== INITIALIZATION ==================== */
void init_fft(void) {
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
}

void reset_measurement_filter(void) {
    memset(&meas_filter, 0, sizeof(meas_filter));
}

/* ==================== DECIMATION ==================== */
void decimate_samples(uint16_t *src, uint16_t src_size,
                      uint16_t *dst, uint16_t dst_size, ScopeMode mode) {
    if(!src_size || !dst_size) return;

    // Source smaller than dest: pad with last sample
    if(src_size <= dst_size) {
        for(uint16_t i = 0; i < dst_size; i++)
            dst[i] = src[(i < src_size) ? i : src_size - 1];
        return;
    }

    for(uint16_t i = 0; i < dst_size; i++) {
        // Map destination indices to source range
        uint32_t start = ((uint32_t)i * (src_size - 1)) / (dst_size - 1);
        uint32_t end = (i == dst_size - 1) ? src_size - 1 :
                       ((uint32_t)(i + 1) * (src_size - 1)) / (dst_size - 1);

        // Clamp bounds
        if(start >= src_size) start = src_size - 1;
        if(end >= src_size) end = src_size - 1;

        uint16_t block_size = end - start + 1;
        uint16_t *block = &src[start];

        switch(mode) {
            case MODE_AVERAGE: {
                uint32_t sum = 0;
                for(uint16_t j = 0; j < block_size; j++) sum += block[j];
                dst[i] = sum / block_size;
                break;
            }
            case MODE_PEAK_DETECT: {
                uint16_t vmin = 4095, vmax = 0;
                for(uint16_t j = 0; j < block_size; j++) {
                    if(block[j] < vmin) vmin = block[j];
                    if(block[j] > vmax) vmax = block[j];
                }
                dst[i] = (i & 1) ? vmax : vmin;  // Alternate min/max
                break;
            }
            default:  // MODE_NORMAL
                dst[i] = block[block_size / 2];
        }
    }
}

/* ==================== TIME DOMAIN MEASUREMENTS ==================== */
void measure_time_domain(uint16_t *buffer, uint32_t size,
                         uint32_t sample_rate, Measurements *m) {
    if(size < 64) { m->valid = 0; return; }

    // Single-pass statistics
    uint32_t sum = 0;
    uint64_t sum_sq = 0;
    uint16_t vmin = 4095, vmax = 0;
    uint32_t high_count = 0;

    for(uint32_t i = 0; i < size; i++) {
        uint16_t val = buffer[i];
        sum += val;
        sum_sq += (uint64_t)val * val;
        if(val < vmin) vmin = val;
        if(val > vmax) vmax = val;
    }

    uint16_t dc = sum / size;
    uint16_t amplitude = vmax - vmin;

    // Convert to millivolts (3.3V reference, 12-bit ADC)
    float32_t raw_amp = (amplitude * 3300.0f) / 4095.0f;
    m->vmax_mv = (vmax * 3300UL) / 4095;
    m->vmin_mv = (vmin * 3300UL) / 4095;

    // RMS from variance
    float32_t variance = (float32_t)(sum_sq / size) - (float32_t)dc * dc;
    float32_t rms_adc;
    arm_sqrt_f32((variance > 0) ? variance : 0, &rms_adc);
    float32_t raw_rms = (rms_adc * 3300.0f) / 4095.0f;

    // Signal too weak check
    if(amplitude < MIN_SIGNAL_AMPLITUDE) {
        m->valid = 0;
        m->frequency_hz = 0;
        m->period_us = 0;
        m->amplitude_mv = (uint16_t)raw_amp;
        m->vrms_mv = (uint16_t)raw_rms;
        m->duty_percent = 50;
        meas_filter.valid_count = 0;
        return;
    }

    // Zero-crossing frequency detection with hysteresis
    uint16_t hyst = (amplitude * ZC_HYSTERESIS_PERCENT) / 100;
    if(hyst < 20) hyst = 20;
    uint16_t thresh_high = dc + hyst, thresh_low = dc - hyst;

    uint32_t rising_edges = 0, first_edge = 0, last_edge = 0;
    uint8_t state = (buffer[0] > dc);

    for(uint32_t i = 1; i < size; i++) {
        if(buffer[i] > dc) high_count++;

        // Schmitt trigger edge detection
        if(!state && buffer[i] > thresh_high) {
            state = 1;
            if(!rising_edges) first_edge = i;
            last_edge = i;
            rising_edges++;
        } else if(state && buffer[i] < thresh_low) {
            state = 0;
        }
    }

    // Calculate frequency from edge timing
    float32_t raw_freq = 0, raw_period = 0;
    if(rising_edges >= 2) {
        uint32_t total_samples = last_edge - first_edge;
        uint32_t periods = rising_edges - 1;
        if(total_samples > 0 && periods > 0) {
            float32_t avg_period = (float32_t)total_samples / periods;
            if(avg_period >= 2.0f) {
                raw_freq = (float32_t)sample_rate / avg_period;
                raw_period = 1000000.0f / raw_freq;
                // Nyquist limit check
                if(raw_freq > sample_rate / 2.0f) raw_freq = raw_period = 0;
            }
        }
    }

    float32_t raw_duty = (float32_t)(high_count * 100) / (size - 1);

    // Invalid frequency check
    if(raw_freq < 0.5f) {
        m->valid = 0;
        m->frequency_hz = 0;
        m->period_us = 0;
        m->amplitude_mv = (uint16_t)raw_amp;
        m->vrms_mv = (uint16_t)raw_rms;
        m->duty_percent = (uint8_t)raw_duty;
        meas_filter.valid_count = 0;
        return;
    }

    // EMA filtering for stable readings
    uint8_t init = (meas_filter.valid_count < 3);
    if(!init && meas_filter.freq > 10.0f) {
        float32_t ratio = raw_freq / meas_filter.freq;
        if(ratio > 1.5f || ratio < 0.67f) init = 1;  // Reset on large jump
    }

    ema_update(&meas_filter.freq, raw_freq, init);
    ema_update(&meas_filter.amp, raw_amp, init);
    ema_update(&meas_filter.rms, raw_rms, init);
    ema_update(&meas_filter.duty, raw_duty, init);
    ema_update(&meas_filter.period, raw_period, init);
    if(meas_filter.valid_count < 255) meas_filter.valid_count++;

    // Output filtered values
    m->frequency_hz = (uint32_t)(meas_filter.freq + 0.5f);
    m->period_us = (uint16_t)(meas_filter.period + 0.5f);
    m->amplitude_mv = (uint16_t)(meas_filter.amp + 0.5f);
    m->vrms_mv = (uint16_t)(meas_filter.rms + 0.5f);
    m->duty_percent = (uint8_t)(meas_filter.duty + 0.5f);
    if(m->duty_percent < 1) m->duty_percent = 1;
    if(m->duty_percent > 99) m->duty_percent = 99;
    m->valid = 1;
}

/* ==================== FREQUENCY DOMAIN (FFT) ==================== */
void measure_freq_domain(uint16_t *src, uint32_t sample_rate,
                         uint16_t *dst, uint16_t dst_size, Measurements *m) {
    // Reset accumulator on first frame
    if(fft_frame_count == 0)
        memset(fft_accumulator, 0, sizeof(fft_accumulator));

    // Remove DC and apply Hanning window
    float32_t dc = 0;
    for(uint16_t i = 0; i < FFT_SIZE; i++) dc += src[i];
    dc /= FFT_SIZE;

    for(uint16_t i = 0; i < FFT_SIZE; i++) {
        float32_t window = 0.5f - 0.5f * arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1));
        fft_input[i] = ((float32_t)src[i] - dc) * window;
    }

    // Execute FFT
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

    // Magnitude with exponential averaging
    float32_t hz_per_bin = (float32_t)sample_rate / FFT_SIZE;
    float32_t max_mag = 0;
    uint16_t max_idx = 0;
    float32_t alpha = 0.3f;

    for(uint16_t i = 0; i < FFT_SIZE/2; i++) {
        float32_t real = fft_output[i*2], imag = fft_output[i*2+1];
        float32_t mag;
        arm_sqrt_f32(real*real + imag*imag, &mag);

        fft_accumulator[i] = (fft_frame_count == 0) ? mag :
                             fft_accumulator[i] * (1.0f - alpha) + mag * alpha;

        if(i > 2 && fft_accumulator[i] > max_mag) {
            max_mag = fft_accumulator[i];
            max_idx = i;
        }
    }
    fft_frame_count++;

    // Peak detection with quadratic interpolation
    typedef struct { float32_t idx, mag; } Peak;
    Peak peaks[20];
    uint8_t peak_count = 0;
    float32_t threshold = max_mag * 0.10f;

    for(uint16_t i = 3; i < (FFT_SIZE/2) - 3 && peak_count < 20; i++) {
        if(fft_accumulator[i] > threshold &&
           fft_accumulator[i] > fft_accumulator[i-1] &&
           fft_accumulator[i] > fft_accumulator[i+1] &&
           fft_accumulator[i] > fft_accumulator[i-2] &&
           fft_accumulator[i] > fft_accumulator[i+2]) {
            // Quadratic interpolation for sub-bin accuracy
            float32_t y0 = fft_accumulator[i-1];
            float32_t y1 = fft_accumulator[i];
            float32_t y2 = fft_accumulator[i+1];
            float32_t denom = y0 - 2*y1 + y2;
            float32_t delta = (denom != 0) ? (0.5f * (y0 - y2) / denom) : 0;

            peaks[peak_count].idx = (float32_t)i + delta;
            peaks[peak_count].mag = y1;
            peak_count++;
            i += 3;  // Skip nearby bins
        }
    }

    // Sort peaks by magnitude (descending)
    for(uint8_t i = 0; i < peak_count; i++) {
        for(uint8_t j = 0; j < peak_count - i - 1; j++) {
            if(peaks[j].mag < peaks[j+1].mag) {
                Peak tmp = peaks[j];
                peaks[j] = peaks[j+1];
                peaks[j+1] = tmp;
            }
        }
    }

    // Scale factor for display
    float32_t scale = (max_mag > 1.0f) ? (3800.0f / max_mag) : 1.0f;

    // Store top 5 peaks
    m->num_peaks = (peak_count > 5) ? 5 : peak_count;
    for(uint8_t i = 0; i < m->num_peaks; i++) {
        m->peak_freqs[i] = (uint32_t)(peaks[i].idx * hz_per_bin);
        m->peak_mags[i] = (uint16_t)(peaks[i].mag * scale);
    }

    // Basic measurements
    m->frequency_hz = (uint32_t)(max_idx * hz_per_bin);
    m->period_us = m->frequency_hz ? (1000000UL / m->frequency_hz) : 0;
    m->amplitude_mv = (uint16_t)(max_mag * scale * 0.87f);

    // RMS via Parseval's theorem
    float32_t total_power = 0;
    for(uint16_t i = 1; i < FFT_SIZE/2; i++)
        total_power += fft_accumulator[i] * fft_accumulator[i];
    float32_t rms;
    arm_sqrt_f32(total_power * 2.0f / FFT_SIZE, &rms);
    m->vrms_mv = (uint16_t)(rms * 3300.0f / 4095.0f / sqrtf(2.0f));
    m->valid = (max_mag > 100.0f && m->num_peaks > 0);

    // Downsample spectrum for display
    if(dst) {
        uint16_t bin_step = (FFT_SIZE / 2) / dst_size;
        for(uint16_t i = 0; i < dst_size; i++) {
            float32_t local_max = 0;
            for(uint16_t j = 0; j < bin_step; j++) {
                uint16_t idx = i * bin_step + j;
                if(idx < FFT_SIZE/2 && fft_accumulator[idx] > local_max)
                    local_max = fft_accumulator[idx];
            }
            dst[i] = (uint16_t)(local_max * scale);
        }
    }
}
