#ifndef OSC_DISPLAY_H
#define OSC_DISPLAY_H

#include <stdint.h>

// Draw dotted grid for time domain (10 divisions)
void draw_grid(void);

// Draw dotted grid for frequency domain (8 divisions)
void draw_freq_grid(void);

// Draw time-domain waveform from buffer
void draw_waveform(uint16_t *buffer, uint16_t size);

// Draw frequency spectrum as bar graph
void draw_spectrum(uint16_t *buffer, uint16_t size);

#endif /* OSC_DISPLAY_H */
