/*
 *	watchdog.c
 *	RP2040 hardware watchdog and the reset reason. See watchdog.h.
 */

#include <stdint.h>

#include <tk/smp.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "platform/drv_watchdog.h"

#define WDT_BASE     (0x40058000U)
#define WDT_CTRL     (WDT_BASE + 0x00U)
#define WDT_LOAD     (WDT_BASE + 0x04U)
#define WDT_REASON   (WDT_BASE + 0x08U)
#define WDT_SCRATCH0 (WDT_BASE + 0x0CU)
#define WDT_SCRATCH1 (WDT_BASE + 0x10U)
#define WDT_SCRATCH2 (WDT_BASE + 0x14U)
#define WDT_TICK     (WDT_BASE + 0x2CU)

#define RUN_TOKEN_MAGIC         (0x52554E31UL)
#define RUN_TOKEN_RESUMES_SHIFT (8U)
#define RUN_TOKEN_BYTE_MASK     (0xFFU)

#define WDT_CTRL_ENABLE     (1UL << 30U)
#define WDT_CTRL_PAUSE_JTAG (1UL << 24U)
#define WDT_CTRL_PAUSE_DBG0 (1UL << 25U)
#define WDT_CTRL_PAUSE_DBG1 (1UL << 26U)

#define WDT_REASON_TIMER (1UL << 0U)
#define WDT_REASON_FORCE (1UL << 1U)

#define CHIP_RESET             (0x40064008U)
#define CHIP_RESET_HAD_POR     (1UL << 8U)
#define CHIP_RESET_HAD_RUN     (1UL << 16U)
#define CHIP_RESET_HAD_RESTART (1UL << 20U)
#define CHIP_RESET_CAUSES      (CHIP_RESET_HAD_POR | CHIP_RESET_HAD_RUN | CHIP_RESET_HAD_RESTART)

#define WDT_TICK_ENABLE  (1UL << 9U)
#define WDT_TICK_RUNNING (1UL << 10U)
/* The tick generator divides the 12 MHz reference clock to 1 MHz. */
#define WDT_TICK_CYCLES (12UL)

/* The counter drops by 2 per tick (RP2040 erratum E1), so the load is twice
   the time in microseconds. The load register holds 24 bits. */
#define WDT_LOAD_PER_MS (2000UL)
#define WDT_LOAD_MAX    (0x00FFFFFFUL)

/* Power-on state machine: the blocks that a watchdog timeout resets. With
   WDSEL at its reset value 0 a timeout resets nothing. Every block except the
   two oscillators (bits 0 and 1) is selected, as the Pico SDK does, so IO_BANK0
   and PADS_BANK0 reset and the motor pins become inputs with pull-downs. */
#define PSM_BASE              (0x40010000U)
#define PSM_WDSEL             (PSM_BASE + 0x08U)
#define PSM_WDSEL_ALL_BUT_OSC (0x0001FFFCUL)

static uint8_t g_reason = PLATFORM_RESET_POWER;
static bool    g_reason_read = false;
static bool    g_b_power_up;
static uint32_t g_load = 0U;
static bool    g_running = false;

uint8_t platform_reset_reason (void)
{
    if (!g_reason_read)
    {
        bool const b_watchdog = (0U != (in_w (WDT_REASON) & (WDT_REASON_TIMER | WDT_REASON_FORCE)));

        g_reason = b_watchdog ? PLATFORM_RESET_WATCHDOG : PLATFORM_RESET_POWER;
        /* HAD_POR represents both power-on and brown-out on the RP2040. */
        g_b_power_up = !b_watchdog && (CHIP_RESET_HAD_POR == (in_w (CHIP_RESET) & CHIP_RESET_CAUSES));
        g_reason_read = true;
    }

    return g_reason;
}

bool platform_reset_was_power_up (void)
{
    (void) platform_reset_reason ();
    return g_b_power_up;
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
    out_w (PSM_WDSEL, PSM_WDSEL_ALL_BUT_OSC);
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

void platform_run_token_write (uint8_t flags, uint8_t resumes)
{
    uint32_t const word = (uint32_t) flags | ((uint32_t) resumes << RUN_TOKEN_RESUMES_SHIFT);

    out_w (WDT_SCRATCH0, RUN_TOKEN_MAGIC);
    out_w (WDT_SCRATCH1, word);
    out_w (WDT_SCRATCH2, ~word);
}

bool platform_run_token_read (uint8_t * p_flags, uint8_t * p_resumes)
{
    uint32_t const word     = in_w (WDT_SCRATCH1);
    bool const     b_intact = (RUN_TOKEN_MAGIC == in_w (WDT_SCRATCH0)) &&
                              (~word == in_w (WDT_SCRATCH2));

    if (b_intact)
    {
        *p_flags   = (uint8_t) (word & RUN_TOKEN_BYTE_MASK);
        *p_resumes = (uint8_t) ((word >> RUN_TOKEN_RESUMES_SHIFT) & RUN_TOKEN_BYTE_MASK);
    }

    return b_intact;
}
