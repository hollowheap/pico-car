/*
 *	watchdog.c
 *	RP2040 hardware watchdog and the reset reason. See watchdog.h.
 */

#include <stdint.h>

#include <tk/smp.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "platform/watchdog.h"

#define WDT_BASE   (0x40058000U)
#define WDT_CTRL   (WDT_BASE + 0x00U)
#define WDT_LOAD   (WDT_BASE + 0x04U)
#define WDT_REASON (WDT_BASE + 0x08U)
#define WDT_TICK   (WDT_BASE + 0x2CU)

#define WDT_CTRL_ENABLE     (1UL << 30U)
#define WDT_CTRL_PAUSE_JTAG (1UL << 24U)
#define WDT_CTRL_PAUSE_DBG0 (1UL << 25U)
#define WDT_CTRL_PAUSE_DBG1 (1UL << 26U)

#define WDT_REASON_TIMER (1UL << 0U)

#define WDT_TICK_ENABLE  (1UL << 9U)
#define WDT_TICK_RUNNING (1UL << 10U)
/* The tick generator divides the 12 MHz reference clock to 1 MHz. */
#define WDT_TICK_CYCLES (12UL)

/* The counter drops by 2 per tick (RP2040 erratum E1), so the load is twice
   the time in microseconds. The load register holds 24 bits. */
#define WDT_LOAD_PER_MS (2000UL)
#define WDT_LOAD_MAX    (0x00FFFFFFUL)

static uint8_t g_reason = PLATFORM_RESET_POWER;
static bool    g_reason_read = false;
static uint32_t g_load = 0U;
static bool    g_running = false;

uint8_t platform_reset_reason (void)
{
    if (!g_reason_read)
    {
        g_reason      = (0U != (in_w (WDT_REASON) & WDT_REASON_TIMER)) ? PLATFORM_RESET_WATCHDOG
                                                                       : PLATFORM_RESET_POWER;
        g_reason_read = true;
    }

    return g_reason;
}

void platform_wdt_start (uint32_t timeout_ms)
{
    uint32_t load = timeout_ms * WDT_LOAD_PER_MS;

    (void) platform_reset_reason ();
    if (load > WDT_LOAD_MAX)
    {
        load = WDT_LOAD_MAX;
    }
    if (0U == (in_w (WDT_TICK) & WDT_TICK_RUNNING))
    {
        out_w (WDT_TICK, WDT_TICK_ENABLE | WDT_TICK_CYCLES);
    }
    g_load = load;
    out_w (WDT_LOAD, load);
    out_w (WDT_CTRL, WDT_CTRL_ENABLE | WDT_CTRL_PAUSE_JTAG | WDT_CTRL_PAUSE_DBG0 | WDT_CTRL_PAUSE_DBG1);
    g_running = true;
}

void platform_wdt_feed (void)
{
    if (g_running)
    {
        out_w (WDT_LOAD, g_load);
    }
}

bool platform_wdt_running (void)
{
    return g_running;
}
