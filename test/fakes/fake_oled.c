/* The OLED writes nowhere on the host. Kept as real functions rather than
   empty macros so anything the firmware passes is still type-checked. */

#include "ssd1306.h"

void ssd1306_init(void)   { }
void ssd1306_clear(void)  { }
void ssd1306_update(void) { }

void ssd1306_print(uint8_t x, uint8_t y, const char *str)
{
    (void)x; (void)y; (void)str;
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
    (void)x; (void)y; (void)color;
}

void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    (void)x0; (void)y0; (void)x1; (void)y1;
}

void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    (void)x; (void)y; (void)w; (void)h;
}
