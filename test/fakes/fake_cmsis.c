/* cyc.h pokes the Cortex-M debug unit directly for cycle profiling. Point
   those registers at plain memory: nothing on the host reads them, and no
   control decision depends on them. */

#include "stm32f4xx.h"

static CoreDebug_Type s_coredebug;
static DWT_Type       s_dwt;

CoreDebug_Type *CoreDebug = &s_coredebug;
DWT_Type       *DWT       = &s_dwt;
