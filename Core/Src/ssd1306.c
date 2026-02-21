
#include "ssd1306.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ==================== Private Defines ====================
#define SSD1306_I2C_ADDR    0x78    // 0x3C << 1 (Change to 0x7A if 0x78 doesn't work)
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64

// IMPORTANT: Change this to match your I2C peripheral
extern I2C_HandleTypeDef hi2c1;  // Change to hi2c2 if you enabled I2C2 in CubeMX

// ==================== Private Variables ====================
static uint8_t ssd1306_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// ==================== Font Data ====================
// Simple 5x7 font (only printable ASCII 32-90)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x56, 0x20, 0x50}, // &
    {0x00, 0x08, 0x07, 0x03, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x2A, 0x1C, 0x7F, 0x1C, 0x2A}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x80, 0x70, 0x30, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x00, 0x60, 0x60, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x72, 0x49, 0x49, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x49, 0x4D, 0x33}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x31}, // 6
    {0x41, 0x21, 0x11, 0x09, 0x07}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x46, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x00, 0x14, 0x00, 0x00}, // :
    {0x00, 0x40, 0x34, 0x00, 0x00}, // ;
    {0x00, 0x08, 0x14, 0x22, 0x41}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x59, 0x09, 0x06}, // ?
    {0x3E, 0x41, 0x5D, 0x59, 0x4E}, // @
    {0x7C, 0x12, 0x11, 0x12, 0x7C}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x41, 0x51, 0x73}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x1C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x26, 0x49, 0x49, 0x49, 0x32}, // S
    {0x03, 0x01, 0x7F, 0x01, 0x03}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x59, 0x49, 0x4D, 0x43}, // Z
};

void ssd1306_command(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data, 2, 100);
}

void ssd1306_init(void) {
    HAL_Delay(100);
    ssd1306_command(0xAE); // Display off
    ssd1306_command(0x20); // Set memory addressing mode
    ssd1306_command(0x00); // Horizontal addressing mode
    ssd1306_command(0xB0); // Set page start address
    ssd1306_command(0xC8); // Set COM scan direction
    ssd1306_command(0x00); // Set low column address
    ssd1306_command(0x10); // Set high column address
    ssd1306_command(0x40); // Set start line address
    ssd1306_command(0x81); // Set contrast
    ssd1306_command(0xFF); // Max contrast
    ssd1306_command(0xA1); // Set segment re-map
    ssd1306_command(0xA6); // Normal display
    ssd1306_command(0xA8); // Set multiplex ratio
    ssd1306_command(0x3F); // 1/64 duty
    ssd1306_command(0xA4); // Display all on resume
    ssd1306_command(0xD3); // Set display offset
    ssd1306_command(0x00); // No offset
    ssd1306_command(0xD5); // Set display clock
    ssd1306_command(0xF0); // Fastest
    ssd1306_command(0xD9); // Set pre-charge period
    ssd1306_command(0x22);
    ssd1306_command(0xDA); // Set COM pins
    ssd1306_command(0x12);
    ssd1306_command(0xDB); // Set VCOMH
    ssd1306_command(0x20);
    ssd1306_command(0x8D); // Enable charge pump
    ssd1306_command(0x14);
    ssd1306_command(0xAF); // Display on
}

void ssd1306_clear(void) {
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
}

void ssd1306_update(void) {
    for(uint8_t page = 0; page < 8; page++) {
        ssd1306_command(0xB0 + page);
        ssd1306_command(0x00);
        ssd1306_command(0x10);

        uint8_t data[SSD1306_WIDTH + 1];
        data[0] = 0x40; // Data mode
        memcpy(&data[1], &ssd1306_buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data, SSD1306_WIDTH + 1, 100);
    }
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    if(color)
        ssd1306_buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else
        ssd1306_buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

void ssd1306_draw_char(uint8_t x, uint8_t y, char c) {
    if(c < 32 || c > 90) c = 32; // Only support space to Z

    const uint8_t *glyph = font5x7[c - 32];
    for(uint8_t i = 0; i < 5; i++) {
        for(uint8_t j = 0; j < 8; j++) {
            if(glyph[i] & (1 << j)) {
                ssd1306_draw_pixel(x + i, y + j, 1);
            }
        }
    }
}

void ssd1306_print(uint8_t x, uint8_t y, const char *str) {
    while(*str) {
        ssd1306_draw_char(x, y, *str++);
        x += 6;
        if(x > SSD1306_WIDTH - 6) break;
    }
}

void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while(1) {
        ssd1306_draw_pixel(x0, y0, 1);
        if(x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if(e2 > -dy) { err -= dy; x0 += sx; }
        if(e2 < dx) { err += dx; y0 += sy; }
    }
}

void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    for(uint8_t i = 0; i < w; i++) {
        for(uint8_t j = 0; j < h; j++) {
            ssd1306_draw_pixel(x + i, y + j, 1);
        }
    }
}

// ==================== Test Screens for Your Project ====================

void test_screen_sensor_data(void) {
    // Simulates displaying 3 nodes with sensor readings
    ssd1306_clear();

    // Title
    ssd1306_print(25, 0, "GREENHOUSE");
    ssd1306_draw_line(0, 9, 127, 9);

    // Node 1
    ssd1306_print(0, 12, "N1 M:75 T:22 L:85");

    // Node 2
    ssd1306_print(0, 22, "N2 M:45 T:24 L:90");

    // Node 3
    ssd1306_print(0, 32, "N3 M:82 T:21 L:78");

    // Status bar
    ssd1306_draw_line(0, 54, 127, 54);
    ssd1306_print(0, 56, "AUTO MODE  12:34");

    ssd1306_update();
}

void test_screen_menu(uint8_t selected) {
    // Simulates menu navigation
    ssd1306_clear();

    ssd1306_print(35, 0, "MAIN MENU");
    ssd1306_draw_line(0, 9, 127, 9);

    const char *menu_items[] = {
        "1.VIEW NODES",
        "2.PROFILES",
        "3.MANUAL CTRL",
        "4.SETTINGS"
    };

    for(uint8_t i = 0; i < 4; i++) {
        if(i == selected) {
            ssd1306_fill_rect(0, 12 + i*12, 128, 10);
            // Invert text (simplified - would need inverse print)
        }
        ssd1306_print(5, 13 + i*12, menu_items[i]);
    }

    ssd1306_update();
}

void test_screen_node_detail(void) {
    // Detailed view of one node
    ssd1306_clear();

    ssd1306_print(30, 0, "NODE 1");
    ssd1306_draw_line(0, 9, 127, 9);

    ssd1306_print(0, 12, "Moisture: 75%");
    ssd1306_print(0, 22, "Temp:     22C");
    ssd1306_print(0, 32, "Light:    850");

    ssd1306_draw_line(0, 44, 127, 44);

    ssd1306_print(0, 48, "Pump:  ON");
    ssd1306_print(0, 56, "Profile: TOMATO");

    ssd1306_update();
}

void test_screen_graph(void) {
    // Simple bar graph of moisture levels
    ssd1306_clear();

    ssd1306_print(15, 0, "MOISTURE LEVELS");
    ssd1306_draw_line(0, 9, 127, 9);

    uint8_t values[] = {75, 45, 82, 60, 55, 90};

    for(uint8_t i = 0; i < 6; i++) {
        uint8_t bar_height = values[i] * 40 / 100;
        uint8_t x = 10 + i * 20;

        // Draw bar
        ssd1306_fill_rect(x, 50 - bar_height, 15, bar_height);

        // Draw value
        char buf[4];
        sprintf(buf, "%d", values[i]);
        ssd1306_print(x + 2, 53, buf);
    }

    // X-axis
    ssd1306_draw_line(5, 50, 125, 50);

    ssd1306_update();
}

void test_screen_alert(void) {
    // Alert screen
    ssd1306_clear();

    // Border
    ssd1306_draw_line(0, 0, 127, 0);
    ssd1306_draw_line(0, 63, 127, 63);
    ssd1306_draw_line(0, 0, 0, 63);
    ssd1306_draw_line(127, 0, 127, 63);

    ssd1306_print(40, 10, "ALERT");

    ssd1306_print(10, 25, "NODE 2");
    ssd1306_print(10, 35, "LOW MOISTURE");
    ssd1306_print(10, 45, "CHECK SENSOR");

    ssd1306_update();
}

// ==================== Main Test Function ====================

void run_oled_test(void) {
    ssd1306_init();
    HAL_Delay(500);

    uint32_t screen = 0;
    uint8_t menu_pos = 0;

    while(1) {
        switch(screen % 5) {
            case 0:
                test_screen_sensor_data();
                break;
            case 1:
                test_screen_menu(menu_pos);
                menu_pos = (menu_pos + 1) % 4;
                break;
            case 2:
                test_screen_node_detail();
                break;
            case 3:
                test_screen_graph();
                break;
            case 4:
                test_screen_alert();
                break;
        }

        screen++;
        HAL_Delay(2000); // 2 seconds per screen
    }
}

