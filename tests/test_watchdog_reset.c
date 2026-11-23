/* Exercise the real reset classifier with the RP2040 reset registers faked. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform/watchdog.h"

#define WATCHDOG_REASON_ADDR (0x40058008U)
#define CHIP_RESET_ADDR      (0x40064008U)
#define CHIP_RESET_POR       (0x00000100U)
#define CHIP_RESET_RUN       (0x00010000U)
#define CHIP_RESET_DEBUG     (0x00100000U)

static uint32_t g_watchdog_reason;
static uint32_t g_chip_reset;

uint32_t shim_reg_read (uint32_t addr)
{
    assert ((WATCHDOG_REASON_ADDR == addr) || (CHIP_RESET_ADDR == addr));
    return (WATCHDOG_REASON_ADDR == addr) ? g_watchdog_reason : g_chip_reset;
}

void shim_reg_write (uint32_t addr, uint32_t value)
{
    (void)addr;
    (void)value;
    abort ();
}

int main (int argc, char ** argv)
{
    struct reset_case
    {
        const char * p_name;
        uint32_t     watchdog_reason;
        uint32_t     chip_reset;
        uint8_t      wire_reason;
        bool         b_power_up;
    } const cases[] = {
        {"timer", 1U, 0U, PLATFORM_RESET_WATCHDOG, false},
        {"forced", 2U, CHIP_RESET_POR, PLATFORM_RESET_WATCHDOG, false},
        {"both_watchdog", 3U, 0U, PLATFORM_RESET_WATCHDOG, false},
        {"power", 0U, CHIP_RESET_POR, PLATFORM_RESET_POWER, true},
        {"run", 0U, CHIP_RESET_RUN, PLATFORM_RESET_POWER, false},
        {"debug", 0U, CHIP_RESET_DEBUG, PLATFORM_RESET_POWER, false},
        {"unknown", 0U, 0U, PLATFORM_RESET_POWER, false},
        {"mixed", 0U, CHIP_RESET_POR | CHIP_RESET_RUN, PLATFORM_RESET_POWER, false},
    };

    if (2 != argc)
    {
        return EXIT_FAILURE;
    }

    for (size_t idx = 0U; idx < sizeof(cases) / sizeof(cases[0]); idx++)
    {
        if (0 == strcmp (argv[1], cases[idx].p_name))
        {
            g_watchdog_reason = cases[idx].watchdog_reason;
            g_chip_reset = cases[idx].chip_reset;
            bool const b_power_up = platform_reset_was_power_up ();
            uint8_t const reason = platform_reset_reason ();

            if ((b_power_up != cases[idx].b_power_up) || (reason != cases[idx].wire_reason))
            {
                (void)fprintf (stderr, "%s reset classified as wire=%u, power-up=%u\n",
                               argv[1], (unsigned)reason, (unsigned)b_power_up);
                return EXIT_FAILURE;
            }
            /* Preserve the boot facts if the registers change after the first read. */
            g_watchdog_reason ^= 1U;
            g_chip_reset ^= CHIP_RESET_POR;
            assert (b_power_up == platform_reset_was_power_up ());
            assert (reason == platform_reset_reason ());
            return EXIT_SUCCESS;
        }
    }
    return EXIT_FAILURE;
}
