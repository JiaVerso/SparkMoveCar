#include "stm32f4xx_hal.h"

#include <time.h>

int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
    (void)clock_id;

    if (tp == NULL) {
        return -1;
    }

    uint32_t tick_ms = HAL_GetTick();
    tp->tv_sec = (time_t)(tick_ms / 1000U);
    tp->tv_nsec = (long)((tick_ms % 1000U) * 1000000UL);

    return 0;
}
