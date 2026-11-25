/** @file platform.h
 *
 * @brief Platform subsystem: the startup task and the startup barrier.
 *
 * One startup task owns the shared hardware init. Every other task waits on
 * the barrier with platform_wait_startup() before it runs its service.
 * Subsystems create their tasks only in their own start function.
 */

#ifndef PLATFORM_PLATFORM_H
#define PLATFORM_PLATFORM_H

#include <stdbool.h>
#include <tk/tkernel.h>

/* Initialise the log and create the startup barrier. */
void platform_init (void);

/* Create and start the startup task. Call it after every other subsystem has
   started, so that the task IDs it reads exist. Returns false when the task
   could not be created or started. */
bool platform_start (void);

/* Wait until the startup task has finished. Returns false on a startup
   failure; the caller must then park. */
bool platform_wait_startup (void);

/* Park a task whose service could not start. Never returns. */
void platform_park (void);

/* Tell every waiting task that the system will not start. */
void platform_startup_failed (void);

/* Check the result of tk_cre_tsk(). On a failure, raise the stalled-task
   fault and log it. Returns true when the task was created. */
bool platform_task_made (const char * p_what, ID id);

/* Start a task. On a failure, raise the stalled-task fault and log it.
   Returns true when the task started. */
bool platform_task_started (const char * p_what, ID tid);

#endif /* PLATFORM_PLATFORM_H */

/*** end of file ***/
