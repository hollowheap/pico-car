/*
 *	watchdog.h
 *	RP2040 hardware watchdog and the reset reason.
 *
 *	The registers are read and written directly, like the other peripherals in
 *	this firmware. Only the supervisor starts and feeds the watchdog.
 */

#ifndef PLATFORM_WATCHDOG_H
#define PLATFORM_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/* Reset reason codes. They match the reset_reason enum of the wire schema. */
#define PLATFORM_RESET_POWER    (0U)
#define PLATFORM_RESET_WATCHDOG (1U)

/* The wire report uses WATCHDOG for either watchdog cause and POWER for other
   resets. Use platform_reset_was_power_up for offline start admission. */
uint8_t platform_reset_reason (void);

/* True with affirmative power-on or brown-out evidence from the RP2040. */
bool platform_reset_was_power_up (void);

/* Start the watchdog with this timeout. The board resets when the time runs
   out before the next feed. The timer stops while a debugger halts the core. */
void platform_wdt_start (uint32_t timeout_ms);

/* Reload the timer. Does nothing before platform_wdt_start. */
void platform_wdt_feed (void);

/* True once the watchdog runs. */
bool platform_wdt_running (void);

/* Run token flags. The token lives in watchdog SCRATCH0-3, which a watchdog
   reset keeps and a power-up clears; SCRATCH4-7 belong to the boot ROM. */
#define PLATFORM_RUN_ARMED      (1U)
#define PLATFORM_RUN_CONTINUOUS (2U)

/* Store the run flags and the watchdog resumes used. */
void platform_run_token_write (uint8_t flags, uint8_t resumes);

/* False when the scratch words hold no intact token. */
bool platform_run_token_read (uint8_t * p_flags, uint8_t * p_resumes);

#endif /* PLATFORM_WATCHDOG_H */
