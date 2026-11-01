/** @file scan_task.h
 *
 * @brief Buddy 5 task body. obstacle_init() creates the task; nothing else
 *        calls it.
 */

#ifndef SCAN_TASK_H
#define SCAN_TASK_H

#include <tk/tkernel.h>

void scan_task_obstacle_task (INT stacd, void * exinf);

#endif /* SCAN_TASK_H */

/*** end of file ***/

