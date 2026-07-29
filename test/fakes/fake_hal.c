/* Host implementations of the HAL calls the firmware makes.
 *
 * The clock is owned by the test: it only moves when fake_tick_advance() is
 * called, which keeps runs deterministic and lets a 30-minute scenario finish
 * in milliseconds. HAL_Delay advances it rather than blocking, so timestamps
 * in the log stay consistent with what the target would have produced.
 */

#include "stm32f4xx_hal.h"

static uint32_t s_now_ms = 0;

uint32_t HAL_GetTick(void)
{
    return s_now_ms;
}

void fake_tick_advance(uint32_t ms)
{
    s_now_ms += ms;
}

void HAL_Delay(uint32_t ms)
{
    fake_tick_advance(ms);
}

/* Peripheral handles the firmware declares extern. Their fields are never
   read, so empty objects are enough to satisfy the linker. */
ADC_HandleTypeDef  hadc1;
TIM_HandleTypeDef  htim2, htim3, htim5;

/* Starting the tick timer means nothing here: the test calls the ISR
   directly whenever it wants a control cycle to happen. */
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;
}
