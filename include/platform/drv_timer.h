/** @file drv_timer.h
 * @brief Free-running RP2040 microsecond clock.
 */
#ifndef PLATFORM_DRV_TIMER_H
#define PLATFORM_DRV_TIMER_H

#include <stdint.h>
#include <tk/tkernel.h>

#define DRV_TIMER_WORD_BITS (32U)

/* Low word for elapsed-time measurements that tolerate unsigned wrap. */
static inline uint32_t now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/* Raw reads avoid the shared high-word latch. Retry across low-word wrap. */
static inline uint64_t drv_timer_now_us64 (void)
{
    uint32_t high = 0U;
    uint32_t low  = 0U;

    do
    {
        high = in_w(TIMER_TIMERAWH);
        low = now_us();
    } while (high != in_w(TIMER_TIMERAWH));
    return ((uint64_t)high << DRV_TIMER_WORD_BITS) | low;
}

#endif /* PLATFORM_DRV_TIMER_H */

/*** end of file ***/
