/** @file scan_task.h
 *
 * @brief Buddy 5 task body. obstacle_extras_init() creates the task; nothing else
 *        calls it.
 */

#ifndef EXTRAS_SCAN_TASK_H
#define EXTRAS_SCAN_TASK_H

#include <tk/tkernel.h>

void extras_scan_task_run (INT stacd, void * exinf);

#endif /* EXTRAS_SCAN_TASK_H */

/*** end of file ***/
