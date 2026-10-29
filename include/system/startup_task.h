#ifndef SYSTEM_STARTUP_TASK_H
#define SYSTEM_STARTUP_TASK_H

#include <stdbool.h>

bool startup_ready (void);
bool startup_failed (void);
void startup_run (void);

#endif /* SYSTEM_STARTUP_TASK_H */
