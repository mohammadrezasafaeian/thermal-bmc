#include "osc_display.h"
#include "ssd1306.h"

#define WAVE_TOP    16      // Top of waveform area
#define WAVE_BOTTOM 63      // Bottom of display
#define WAVE_CENTER 40      // Vertical center

/* ==================== TIME DOMAIN GRID ==================== */
void draw_grid(void) {
    // Horizontal lines (4 divisions)
    for(uint8_t y = WAVE_TOP; y < WAVE_BOTTOM; y += 12)
        for(uint8_t x = 0; x < 128; x += 4)
            ssd1306_draw_pixel(x, y, 1);

    // Vertical lines (10 divisions to match timebase)
    for(uint8_t i = 0; i <= 10; i++) {
        uint8_t x = (i * 127) / 10;
        for(uint8_t y = WAVE_TOP; y < WAVE_BOTTOM; y += 4)
            ssd1306_draw_pixel(x, y, 1);
    }
}

/* ==================== FREQUENCY DOMAIN GRID ==================== */
void draw_freq_grid(void) {
    // Horizontal lines
    for(uint8_t y = WAVE_TOP; y < WAVE_BOTTOM; y += 12)
        for(uint8_t x = 0; x < 128; x += 4)
            ssd1306_draw_pixel(x, y, 1);

    // Vertical lines (8 frequency divisions)
    for(uint8_t i = 0; i <= 8; i++)
        for(uint8_t y = WAVE_TOP; y < WAVE_BOTTOM; y += 4)
            ssd1306_draw_pixel(i * 16, y, 1);
}

/* ==================== WAVEFORM DRAWING ==================== */
void draw_waveform(uint16_t *buffer, uint16_t size) {
    if(size > 128) size = 128;
    const int32_t scale = 4;

    // Clamp and scale first point
    int32_t prev_y = WAVE_CENTER - ((buffer[0] - 2048) * scale / 85);
    if(prev_y < WAVE_TOP) prev_y = WAVE_TOP;
    if(prev_y > WAVE_BOTTOM) prev_y = WAVE_BOTTOM;

    // Draw connected line segments
    for(uint16_t x = 1; x < size; x++) {
        int32_t y = WAVE_CENTER - ((buffer[x] - 2048) * scale / 85);
        if(y < WAVE_TOP) y = WAVE_TOP;
        if(y > WAVE_BOTTOM) y = WAVE_BOTTOM;

        ssd1306_draw_line(x - 1, prev_y, x, y);
        prev_y = y;
    }
}

/* ==================== SPECTRUM DRAWING ==================== */
void draw_spectrum(uint16_t *buffer, uint16_t size) {
    if(size > 128) size = 128;
    const uint8_t height = WAVE_BOTTOM - WAVE_TOP;

    // Find max for auto-scaling
    uint16_t max_val = 1;
    for(uint16_t i = 0; i < size; i++)
        if(buffer[i] > max_val) max_val = buffer[i];

    // Draw vertical bars
    for(uint16_t x = 0; x < size; x++) {
        uint32_t bar = ((uint32_t)buffer[x] * height) / max_val;
        if(bar > height) bar = height;
        if(bar < 1 && buffer[x] > 0) bar = 1;

        if(bar > 0)
            ssd1306_draw_line(x, WAVE_BOTTOM, x, WAVE_BOTTOM - bar);
    }
}
