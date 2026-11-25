/*
 *	watchdog.h
 *	RP2040 hardware watchdog and the reset reason (D160).
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

/* The cause of the last reset, read once at start-up. The RP2040 cannot tell
   a brown-out from a power-on, so a brown-out reads as POWER. */
uint8_t platform_reset_reason (void);

/* Start the watchdog with this timeout. The board resets when the time runs
   out before the next feed. The timer stops while a debugger halts the core. */
void platform_wdt_start (uint32_t timeout_ms);

/* Reload the timer. Does nothing before platform_wdt_start. */
void platform_wdt_feed (void);

/* True once the watchdog runs. */
bool platform_wdt_running (void);

#endif /* PLATFORM_WATCHDOG_H */
