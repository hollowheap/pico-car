/** @file attitude.h
 *
 * @brief Lift and overturn detection from the gravity direction (D140).
 *
 * Pure logic with no hardware access. It uses the angle to the rest gravity
 * vector g0, which the hump detector stores when it calibrates, so it does
 * not depend on which board axis points forward.
 *
 *   tilt  angle between the current acceleration and g0 (0 on flat ground)
 *   roll  sideways angle, left-down positive
 *   amag  acceleration magnitude in g
 *
 * States: OK, CANDIDATE, LIFTED, OVERTURNED. Before the rest calibration the
 * module reports OK and calibrated = false. All limits are placeholders until
 * tuned on logs from the car (artifacts/notes/imu-lift-overturn.md).
 */

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

/* Board axes of the 3-vectors. */
#define ATTITUDE_AXIS_X (0U)
#define ATTITUDE_AXIS_Y (1U)
#define ATTITUDE_AXIS_Z (2U)
#define ATTITUDE_AXES   (3U)

/* Values match the attitude_state enum of the wire schema. */
typedef enum
{
    ATTITUDE_OK = 0,
    ATTITUDE_CANDIDATE,
    ATTITUDE_LIFTED,
    ATTITUDE_OVERTURNED
} attitude_state_t;

typedef struct
{
    float    tilt_envelope_deg;   /* A tilt above this is a breach.             */
    float    lift_dev_g;          /* | |a| - 1 g | above this is unsettled.     */
    uint16_t lift_dev_samples;    /* Unsettled samples in a row that count.     */
    uint16_t lift_hold_samples;   /* The unsettled cue stays this long.         */
    uint16_t lift_samples;        /* Candidate cue held this long: lifted.      */
    uint16_t candidate_clear;     /* No cue this long: back to OK.              */
    float    overturn_tilt_deg;   /* Tilt above this: overturned.               */
    float    overturn_roll_deg;   /* | roll | above this: overturned.           */
    uint16_t overturn_samples;    /* Held this long: overturned.                */
    float    clear_tilt_deg;      /* Level: tilt under this.                    */
    float    clear_dev_g;         /* Still: | |a| - 1 g | under this.           */
    uint16_t clear_samples;       /* Level and still this long: lifted clears.  */
} attitude_config_t;

typedef struct
{
    float ax_g;                   /* Board-axis acceleration, g.                */
    float ay_g;
    float az_g;
    bool  calibrated;             /* g0 is valid.                               */
    float g0[ATTITUDE_AXES];      /* Rest gravity vector, board axes, g.        */
    float fwd[ATTITUDE_AXES];     /* Unit vector of travel, board axes.         */
    bool  b_wheels_free;          /* The wheels turn on target with low duty.   */
} attitude_input_t;

typedef struct
{
    attitude_state_t state;
    bool  calibrated;
    float tilt_deg;
    float roll_deg;
    float amag_g;
    float az_g;                   /* Board z reading, g.                        */
    bool  changed;                /* state differs from the previous call.      */
} attitude_output_t;

typedef struct
{
    attitude_config_t cfg;
    attitude_state_t  state;
    uint16_t dev_run;
    uint16_t dev_hold;
    uint16_t cue_count;
    uint16_t no_cue_count;
    uint16_t overturn_count;
    uint16_t clear_count;
    bool     was_calibrated;
} attitude_ctx_t;

/* Fill cfg with the placeholder defaults. */
void attitude_default_config (attitude_config_t * p_cfg);

/* Check cfg and reset the context. Returns false for a null argument or a
 * setting out of range. */
bool attitude_init (attitude_ctx_t * p_ctx, const attitude_config_t * p_cfg);

/* Process one valid accelerometer sample. Returns false for a null argument or
 * a non-finite value. Do not call it for a failed read: the state holds. */
bool attitude_update (attitude_ctx_t * p_ctx, const attitude_input_t * p_in,
                      attitude_output_t * p_out);

/* Forget the state. The next calibrated sample starts at OK. */
void attitude_reset (attitude_ctx_t * p_ctx);

#endif /* ATTITUDE_H */

/*** end of file ***/
