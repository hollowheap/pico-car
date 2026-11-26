/** @file navigation.h
 *
 * @brief Buddy 5 Phase 2 glue: connects the obstacle subsystem to the line
 *        task in main.c and to the motion service.
 *
 * Use from the line task (processor 1), after gpio_irq_init(),
 * motion_service_start() and ir_capture_init():
 *   1. extras_nav_init() once.
 *   2. Pass the supervisor's run state and START generation on every
 *      extras_nav_step() call. Drive permission also requires fresh IR,
 *      idle turn/probe execution and ready motion. When the step returns
 *      true, the line follower yields its speed command for this pass.
 */

#ifndef EXTRAS_NAV_H
#define EXTRAS_NAV_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

ER   extras_nav_init (void);
bool extras_nav_step (bool b_line_left, bool b_line_right, bool b_armed,
                      uint8_t start_count, bool b_drive_allowed);

#endif /* EXTRAS_NAV_H */

/*** end of file ***/
