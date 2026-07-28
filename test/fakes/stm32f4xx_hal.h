/* Host stand-in for ST's HAL.
 *
 * Only what the firmware actually touches. Peripheral handles are opaque:
 * thermal_app.c never reads their fields, it only passes addresses around,
 * so an empty struct is a faithful stand-in.
 */
#ifndef FAKE_STM32F4XX_HAL_H
#define FAKE_STM32F4XX_HAL_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

typedef struct { int _unused; } TIM_HandleTypeDef;
typedef struct { int _unused; } ADC_HandleTypeDef;
typedef struct { int _unused; } I2C_HandleTypeDef;
typedef struct { int _unused; } UART_HandleTypeDef;
typedef struct { int _unused; } GPIO_TypeDef;

#define GPIO_PIN_SET    1
#define GPIO_PIN_RESET  0
typedef int GPIO_PinState;

uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h);

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr,
                                        uint32_t trials, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t addr,
                                          uint8_t *data, uint16_t len,
                                          uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t addr,
                                         uint8_t *data, uint16_t len,
                                         uint32_t timeout);

/* ---- test control ------------------------------------------------------ */
/* The harness owns the clock: time only moves when a test says so. */
void fake_tick_advance(uint32_t ms);

/* Script what the I2C nodes reply with, and inspect what was sent. */
void fake_i2c_set_present(int node, int present);
void fake_i2c_set_telemetry(int node, uint16_t adc_raw, uint16_t tach);
void fake_i2c_last_command(int node, uint8_t *heater_pwm, uint8_t *fan_pwm);
void fake_i2c_reset(void);

#endif
