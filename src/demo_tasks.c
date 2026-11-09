/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	demo_tasks.c
 *	The console-independent liveness task.  See demo_tasks.h.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "usb_console_compat.h"
#include "demo_tasks.h"

/* Blink timing in milliseconds and the long/short count encoding. */
#define BLINK_LONG_UNITS      (5)
#define BLINK_LONG_ON_MS      (600)
#define BLINK_LONG_OFF_MS     (300)
#define BLINK_SHORT_ON_MS     (150)
#define BLINK_SHORT_OFF_MS    (250)
#define BLINK_CODE_GAP_MS     (2000)
#define BLINK_LIVE_HALF_MS    (250)

/* tm_usb_diag_level() value for a fully enumerated USB device. */
#define BLINK_USB_READY_LEVEL (8)

/* Diagnostic checkpoint: the blink task is being scheduled. */
#define BLINK_DIAG_CHECKPOINT (8)

/*
 * Blink task: liveness independent of the console, and -- since USB does not
 * enumerate and therefore cannot report on itself -- a readout of how far USB
 * bring-up gets, using the long/short count encoding from Phase 0.
 *
 *	0  (dark)  tusb_init() failed
 *	1          tk_def_int() rejected the USB interrupt
 *	2          NVIC never unmasked USBCTRL_IRQ
 *	3          USB controller not enabled
 *	4          D+ pull-up not asserted, so no host can see the device
 *	5          pull-up on, but not one USB interrupt has ever fired
 *	6          interrupts firing, but no bus reset or connect from the host
 *	7          host talking, enumeration not completing
 *	8          fully enumerated
 *
 * If any console call blocks, this task stops and the LED freezes entirely.
 */
LOCAL void blink_code (INT n)
{
    INT i = 0;

    for (i = 0; i < n / BLINK_LONG_UNITS; i++)
    { /* long flash = 5 */
        gpio_set_val(BOARD_LED_PIN, 1);
        tk_dly_tsk(BLINK_LONG_ON_MS);
        gpio_set_val(BOARD_LED_PIN, 0);
        tk_dly_tsk(BLINK_LONG_OFF_MS);
    }
    for (i = 0; i < n % BLINK_LONG_UNITS; i++)
    { /* short flash = 1 */
        gpio_set_val(BOARD_LED_PIN, 1);
        tk_dly_tsk(BLINK_SHORT_ON_MS);
        gpio_set_val(BOARD_LED_PIN, 0);
        tk_dly_tsk(BLINK_SHORT_OFF_MS);
    }
    gpio_set_val(BOARD_LED_PIN, 0);
    tk_dly_tsk(BLINK_CODE_GAP_MS);
}

EXPORT void blink_task (INT stacd, void * exinf)
{
    INT level = 0;

    (void)stacd;
    (void)exinf;

#if BOARD_DIAG_GPIO_SCAN
    /* CP8: the blink task is being scheduled */
    knl_diag_pulse(BLINK_DIAG_CHECKPOINT);
#endif

    for (;;)
    {
        level = tm_usb_diag_level();
        if (level >= BLINK_USB_READY_LEVEL)
        {
            /* Fully up: plain 2 Hz liveness, so a console stall or a
               blocked kernel is visible at a glance. */
            gpio_set_val(BOARD_LED_PIN, 1);
            tk_dly_tsk(BLINK_LIVE_HALF_MS);
            gpio_set_val(BOARD_LED_PIN, 0);
            tk_dly_tsk(BLINK_LIVE_HALF_MS);
        }
        else
        {
            blink_code(level);
        }
    }
}
