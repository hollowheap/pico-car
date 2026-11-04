/** @file track.h
 * @brief Track subsystem facade: line sensor capture and line task.
 */
#ifndef TRACK_TRACK_H
#define TRACK_TRACK_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef struct
{
    bool barcode : 1;
    bool left : 1;
    bool right : 1;
} ir_sensors_state_t;

/** Initialise the IR sensor interrupt and register with the shared dispatcher. */
ER ir_capture_init (void);
/** Get the current state of the IR sensors. */
ir_sensors_state_t ir_get_state (void);

/** Composition-root line task entry. */
void line_task (INT stacd, void * exinf);

#endif /* TRACK_TRACK_H */
