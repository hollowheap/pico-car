/** @file wdt_check.c
 *
 * @brief Watchdog test W1 (Q61): do the H-bridge inputs stay low across an
 *        RP2040 watchdog reset? Standalone application with its own usermain.
 *
 * Wheels off the floor. Press GP21: both wheels run at full duty, then the
 * bench arms the 1 s watchdog and hangs on purpose. The board resets. Watch
 * the wheels during the 1 s before the reset and for 10 s after it. After the
 * reset the bench holds the motor pins low and says so on the console.
 *
 * Pass: the wheels stop at the reset and stay stopped.
 * Fail: the wheels keep turning, or twitch, after the reset. Then the
 * watchdog must stay off (PICO_CAR_WATCHDOG not set).
 *
 * The bench also prints the level of the four motor pins when it starts. After
 * a power-up the kernel's I2C init can leave GP8 and GP9 high; that shows
 * whether the motors twitch at every boot.
 *
 * NOTE: usermain() is the name the kernel calls, so it has no module prefix.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/smp.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "platform/isr.h"
#include "platform/log.h"
#include "platform/watchdog.h"
#include "../usb_console_compat.h"

#define BTN_RUN_PIN   (21U)
#define PAD_PULL_UP   (1UL << 3U)
#define PAD_PULL_DOWN (1UL << 2U)

#define MOTOR_PIN_COUNT (4U)
#define MOTOR_PIN_A     (8U)
#define MOTOR_PIN_B     (9U)
#define MOTOR_PIN_C     (10U)
#define MOTOR_PIN_D     (11U)

#define USB_WAIT_LOOPS (200U)
#define USB_WAIT_MS    (100U)
#define USB_UP         (2U)
#define SPIN_MS        (2000U)
#define WATCHDOG_MS    (1000U)
#define REPORT_MS      (5000U)
#define POLL_MS        (20U)

static UINT const MOTOR_PINS[MOTOR_PIN_COUNT] = {MOTOR_PIN_A, MOTOR_PIN_B, MOTOR_PIN_C, MOTOR_PIN_D};

static void pins_out (UINT const * p_levels)
{
    unsigned int idx = 0U;

    for (idx = 0U; idx < MOTOR_PIN_COUNT; idx++)
    {
        (void) gpio_set_val (MOTOR_PINS[idx], p_levels[idx]);
        (void) gpio_set_pin (MOTOR_PINS[idx], GPIO_MODE_OUT);
    }
}

EXPORT INT usermain (void)
{
    UINT const low[MOTOR_PIN_COUNT]  = {0U, 0U, 0U, 0U};
    UINT const spin[MOTOR_PIN_COUNT] = {1U, 0U, 1U, 0U};
    UINT       early[MOTOR_PIN_COUNT] = {0U, 0U, 0U, 0U};
    uint8_t const reason = platform_reset_reason ();
    unsigned int  idx    = 0U;
    uint32_t      loops  = 0U;
    bool          b_down = false;

    for (idx = 0U; idx < MOTOR_PIN_COUNT; idx++)
    {
        early[idx] = gpio_get_val (MOTOR_PINS[idx]);
    }
    log_init ();
    (void) gpio_set_pin (BTN_RUN_PIN, GPIO_MODE_IN);
    clr_w (GPIO (BTN_RUN_PIN), PAD_PULL_DOWN);
    set_w (GPIO (BTN_RUN_PIN), PAD_PULL_UP);
    for (loops = 0U; (loops < USB_WAIT_LOOPS) && (tm_usb_state () < USB_UP); loops++)
    {
        (void) tk_dly_tsk (USB_WAIT_MS);
    }

    LOG_NOW ("DATA", "bench:wdt", "W1_PINS,GP8=%u,GP9=%u,GP10=%u,GP11=%u,reset_reason=%u\n",
             (unsigned) early[0], (unsigned) early[1], (unsigned) early[2], (unsigned) early[3],
             (unsigned) reason);
    pins_out (low);

    if (PLATFORM_RESET_WATCHDOG == reason)
    {
        for (;;)
        {
            LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE,
                      "W1: this boot follows the watchdog reset. Motor pins are held low. Did the wheels stop?");
            log_flush ();
            (void) tk_dly_tsk (REPORT_MS);
        }
    }

    LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE,
              "W1: wheels off the floor. GP21 runs both wheels, then hangs on purpose.");
    log_flush ();
    for (;;)
    {
        bool const b_now = (0U == gpio_get_val (BTN_RUN_PIN));

        if (b_now && !b_down)
        {
            LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE, "W1: wheels ON for 2 s");
            log_flush ();
            pins_out (spin);
            (void) tk_dly_tsk (SPIN_MS);
            LOG_INFO (LOG_TASK_BOOT, LOG_MOD_NONE,
                      "W1: watchdog armed, hanging now. Watch the wheels for 10 s.");
            log_flush ();
            platform_wdt_start (WATCHDOG_MS);
            for (;;)
            {
                /* No feed: the watchdog resets the board. */
            }
        }
        b_down = b_now;
        (void) tk_dly_tsk (POLL_MS);
    }
}

/*** end of file ***/
