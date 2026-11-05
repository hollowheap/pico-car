/** @file obstacle_nav.h
 *
 * @brief Buddy 5 Phase 2 glue: connects the obstacle subsystem to the line
 *        task in main.c and to the motion service.
 *
 * Use from the line task (processor 1), after gpio_irq_init(),
 * motion_service_init() and ir_capture_init():
 *   1. obstacle_nav_init() once.
 *   2. Every loop pass, obstacle_nav_step(left, right). When it returns true,
 *      Buddy 5 is driving round an obstacle or holding the car, so the
 *      line follower must not send its own speed this pass.
 */

#ifndef OBSTACLE_NAV_H
#define OBSTACLE_NAV_H

#include <stdbool.h>
#include <tk/tkernel.h>

ER   obstacle_nav_init (void);
bool obstacle_nav_step (bool b_line_left, bool b_line_right);

#endif /* OBSTACLE_NAV_H */

/*** end of file ***/

