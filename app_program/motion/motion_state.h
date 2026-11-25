/** @file motion_state.h
 *
 * @brief Motion public facade: runtime commands, status and feedback.
 *
 * The controller, PID, odometry and PWM headers stay private in
 * app_program/motion/. Configuration and task start-up remain in the private
 * motion_service.h and are called only by the motion subsystem and the startup task.
 */

#ifndef MOTION_MOTION_H
#define MOTION_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

#define MOTION_WHEELS              2U
#define MOTION_FEEDBACK_MAX_AGE_US 25000U

/* State and fault values are the controller's; the dashboard owns the
 * decoded names. */
#define MOTION_STATE_IDLE      (0U)
#define MOTION_STATE_RUNNING   (1U)
#define MOTION_STATE_REVERSING (2U)
#define MOTION_STATE_STOPPED   (3U)
#define MOTION_STATE_FAULT     (4U)

/* Hold reasons. Each owner sets/clears one bit. */
#define MOTION_HOLD_OBSTACLE  UINT32_C(1)
/* Bit value 2 is retired: losing the line never holds the car. Keep
 * the gap so the telemetry hold_mask bit positions stay fixed. */
#define MOTION_HOLD_OPERATOR  UINT32_C(4)
#define MOTION_HOLD_BARCODE   UINT32_C(8)  /* a barcode stop; START releases it */
#define MOTION_HOLD_LIFT      UINT32_C(16) /* lifted or overturned; START releases it */
#define MOTION_HOLD_SUPERVISOR UINT32_C(32) /* the run permit lapsed */
#define MOTION_HOLD_CALIBRATE UINT32_C(64) /* the IMU calibrates at rest */
#define MOTION_HOLD_ALL       UINT32_C(125)

/* One coherent feedback snapshot. Speeds and displacements are signed and
 * positive car-forward. Encoder recovery retains accumulated displacement.
 * fresh means the sample is within the age limit. */
typedef struct
{
    int32_t  left_speed_um_s;
    int32_t  right_speed_um_s;
    int64_t  left_displacement_um;
    int64_t  right_displacement_um;
    uint32_t timestamp_us;
    uint32_t age_us;
    bool     speed_valid[MOTION_WHEELS];
    bool     odometry_valid[MOTION_WHEELS];
    bool     fresh;
    uint8_t  invalid_samples[MOTION_WHEELS]; /* Wraps; see motion_wheel_sample_t. */
} motion_feedback_t;

STATIC_ASSERT (sizeof(motion_feedback_t) == 40U, "motion_feedback_t size");

typedef struct
{
    uint8_t  state;          /* MOTION_STATE_* */
    uint8_t  fault;          /* controller fault code */
    bool     ready;
    bool     stop_requested; /* Direct emergency latch. */
    uint32_t hold_mask;      /* MOTION_HOLD_* */
    int      init_error; /* an ER code; this header stays kernel-free */
    uint32_t stop_age_ms; /* Age of the first emergency request, when latched. */
    /* Advances on each valid moving interval entirely after the last hold
       release. Remains readable after an emergency stop. */
    uint32_t movement_sequence;
} motion_status_t;

STATIC_ASSERT (sizeof(motion_status_t) == 20U, "motion_status_t size");

/* What the controller is doing, for telemetry. Targets are what the last
 * speed command asked for; duty is what the controller applied, signed
 * permille. overruns counts cyclic wake-ups the motion task missed. */
typedef struct
{
    int32_t  left_target_um_s;
    int32_t  right_target_um_s;
    uint32_t overruns;
    int16_t  left_duty_permille;
    int16_t  right_duty_permille;
} motion_diag_t;

STATIC_ASSERT (sizeof(motion_diag_t) == 16U, "motion_diag_t size");

int motion_feedback_snapshot (motion_feedback_t * p_feedback);
int motion_status (motion_status_t * p_status);
int motion_diag_snapshot (motion_diag_t * p_diag);
/* Status and feedback from one controller pass. Diagnostics are optional. */
int motion_snapshot (motion_status_t * p_status, motion_feedback_t * p_feedback,
                     motion_diag_t * p_diag);

int motion_set_speed (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us);
/* A scan aiming pivot uses the normal speed-loop duty ceiling. */
int motion_set_scan_pivot (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us);
int motion_set_hold (uint32_t reason, bool held);
int motion_request_stop (void);

/* The supervisor renews the run permit for lease_ms. When the permit lapses,
 * the motion task sets MOTION_HOLD_SUPERVISOR by itself, and the car brakes
 * even if the supervisor hangs. The permit is not enforced until the first
 * renewal. */
int motion_renew_permit (uint32_t lease_ms);
/* Whole ms since the last renewal, rounded up, on the motion clock that holds
 * the wheels. UINT32_MAX before the first renewal. */
uint32_t motion_permit_age_ms (void);

/* The supervisor tells the motion task that the encoders answer crosswise.
 * The motion task adds the encoder_dead fault bit while it is true. */
void motion_note_encoder_crossed (bool b_crossed);

#endif /* MOTION_MOTION_H */

/*** end of file ***/
