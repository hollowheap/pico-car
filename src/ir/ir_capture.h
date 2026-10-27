#ifndef IR_CAPTURE_H
#define IR_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#define IR_TYPE_BARCODE 0U
#define IR_TYPE_LINE    1U

typedef struct
{
    /*
     * The barcode sensor.
     */
    bool barcode : 1;
    /*
     * The left line sensor.
     */
    bool left : 1;
    /*
     * The right line sensor.
     */
    bool right : 1;
} ir_sensors_state_t;

/** Initialise the IR sensor interrupt. */
ER ir_capture_init (void);
/** Get the current state of the IR sensors. */
ir_sensors_state_t ir_get_state (void);
/** Wait for a change in the IR sensors of the given type. */
ER ir_wait_change (uint8_t type, TMO tmout);

#endif /* IR_CAPTURE_H */

/*** end of file ***/
