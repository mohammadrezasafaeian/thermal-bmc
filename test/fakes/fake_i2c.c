/* Host stand-in for the I2C bus and the three ATmega nodes on it.
 *
 * The test scripts the bus: it decides which nodes answer and what they
 * report. Bus timing, arbitration and clock stretching are not modelled -
 * what the firmware branches on is only HAL_OK vs not-HAL_OK.
 *
 * A node that is not present NAKs, which the real HAL reports as HAL_ERROR,
 * and BusTask turns that into is_online = 0.
 */

#include "stm32f4xx_hal.h"
#include "thermal_app.h"
#include <string.h>

I2C_HandleTypeDef hi2c1;

#define NODE_BASE_ADDR7  0x20        /* nodes live at 0x20, 0x22, 0x24 */
#define NODE_ADDR_STEP   2

static struct {
    int      present;
    uint16_t adc_raw;
    uint16_t tach;
    uint8_t  last_heater_pwm;
    uint8_t  last_fan_pwm;
} s_node[NUM_REMOTE_NODES];

/* HAL takes 8-bit addresses (7-bit shifted left). Returns -1 if the address
   does not belong to one of our nodes. */
static int node_of(uint16_t hal_addr)
{
    int a7 = hal_addr >> 1;
    int i  = (a7 - NODE_BASE_ADDR7) / NODE_ADDR_STEP;

    if (a7 < NODE_BASE_ADDR7 || i >= NUM_REMOTE_NODES) return -1;
    if (a7 != NODE_BASE_ADDR7 + i * NODE_ADDR_STEP)    return -1;

    return i;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr,
                                        uint32_t trials, uint32_t timeout)
{
    (void)h; (void)trials; (void)timeout;

    int i = node_of(addr);
    return (i >= 0 && s_node[i].present) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t addr,
                                          uint8_t *data, uint16_t len,
                                          uint32_t timeout)
{
    (void)h; (void)timeout;

    int i = node_of(addr);
    if (i < 0 || !s_node[i].present) return HAL_ERROR;
    if (len != sizeof(I2C_Command))  return HAL_ERROR;

    I2C_Command cmd;
    memcpy(&cmd, data, sizeof(cmd));
    s_node[i].last_heater_pwm = cmd.heater_pwm;
    s_node[i].last_fan_pwm    = cmd.fan_pwm;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t addr,
                                         uint8_t *data, uint16_t len,
                                         uint32_t timeout)
{
    (void)h; (void)timeout;

    int i = node_of(addr);
    if (i < 0 || !s_node[i].present)  return HAL_ERROR;
    if (len != sizeof(I2C_Telemetry)) return HAL_ERROR;

    I2C_Telemetry tel;
    tel.adc_raw     = s_node[i].adc_raw;
    tel.tach_pulses = s_node[i].tach;
    memcpy(data, &tel, sizeof(tel));

    return HAL_OK;
}

/* ---- test control ------------------------------------------------------ */

void fake_i2c_set_present(int node, int present)
{
    if (node >= 0 && node < NUM_REMOTE_NODES) s_node[node].present = present;
}

void fake_i2c_set_telemetry(int node, uint16_t adc_raw, uint16_t tach)
{
    if (node < 0 || node >= NUM_REMOTE_NODES) return;
    s_node[node].adc_raw = adc_raw;
    s_node[node].tach    = tach;
}

void fake_i2c_last_command(int node, uint8_t *heater_pwm, uint8_t *fan_pwm)
{
    if (node < 0 || node >= NUM_REMOTE_NODES) return;
    if (heater_pwm) *heater_pwm = s_node[node].last_heater_pwm;
    if (fan_pwm)    *fan_pwm    = s_node[node].last_fan_pwm;
}

void fake_i2c_reset(void)
{
    memset(s_node, 0, sizeof(s_node));
}
