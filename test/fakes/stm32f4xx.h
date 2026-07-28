/* Host stand-in for the CMSIS device header.
 *
 * cyc.h pokes the Cortex-M debug unit (DWT cycle counter) directly. That is
 * genuinely ARM-only, and it is pure profiling - no control decision depends
 * on it - so here the registers are plain memory that nothing reads.
 */
#ifndef FAKE_STM32F4XX_H
#define FAKE_STM32F4XX_H

#include <stdint.h>

typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
typedef struct { volatile uint32_t CTRL; volatile uint32_t CYCCNT; } DWT_Type;

extern CoreDebug_Type *CoreDebug;
extern DWT_Type       *DWT;

#define CoreDebug_DEMCR_TRCENA_Msk  (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk      (1UL << 0)

#endif
