/*
 * ssd1306.h
 *
 *  Created on: Oct 15, 2025
 *      Author: Tick
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"

// Public functions
void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_update(void);
void ssd1306_print(uint8_t x, uint8_t y, const char *str);
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color);
void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

// Test functions (optional, for testing only)
void test_screen_sensor_data(void);
void test_screen_menu(uint8_t selected);
void test_screen_node_detail(void);
void test_screen_graph(void);
void test_screen_alert(void);
void run_oled_test(void);

#endif

#endif /* INC_SSD1306_H_ */
