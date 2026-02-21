#include "keypad.h"

#define SCL_PIN GPIO_PIN_8
#define SDO_PIN GPIO_PIN_9
#define KEYPAD_PORT GPIOB

void keypad_init(void) {
    HAL_GPIO_WritePin(KEYPAD_PORT, SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(2);  // Give keypad time to stabilize
}

uint8_t keypad_read(void) {
    uint8_t key_state = 0;

    for (uint8_t count = 1; count <= 16; count++) {
        HAL_GPIO_WritePin(KEYPAD_PORT, SCL_PIN, GPIO_PIN_RESET);
        // Use shorter delay - microseconds would be better but HAL_Delay(1) = 1ms
        for(volatile uint16_t i = 0; i < 100; i++);  // ~microsecond delay

        if (HAL_GPIO_ReadPin(KEYPAD_PORT, SDO_PIN) == GPIO_PIN_RESET) {
            key_state = count;
        }

        HAL_GPIO_WritePin(KEYPAD_PORT, SCL_PIN, GPIO_PIN_SET);
        for(volatile uint16_t i = 0; i < 100; i++);  // ~microsecond delay
    }

    return key_state;
}
