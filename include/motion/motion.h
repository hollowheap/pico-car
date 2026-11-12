/** @file motion.h
 *
 * @brief Motion public facade: runtime commands, status and feedback.
 *
 * The controller, PID, odometry and PWM headers stay private in
 * src/motion/. Configuration and task start-up remain in the private
 * motion_service.h and are called only by the composition root.
 */

#ifndef MOTION_MOTION_H
#define MOTION_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

#define MOTION_WHEELS              2U
#define MOTION_FEEDBACK_MAX_AGE_US 600000U

/* State and fault values are the controller's; the dashboard owns the
 * decoded names. */
#define MOTION_STATE_IDLE      (0U)
#define MOTION_STATE_RUNNING   (1U)
#define MOTION_STATE_REVERSING (2U)
#define MOTION_STATE_STOPPED   (3U)
#define MOTION_STATE_FAULT     (4U)

/* Hold reasons. Each owner sets/clears one bit. */
#define MOTION_HOLD_OBSTACLE  UINT32_C(1)
/* Bit value 2 is retired: losing the line never holds the car (D180). Keep
 * the gap so the telemetry hold_mask bit positions stay fixed. */
#define MOTION_HOLD_OPERATOR  UINT32_C(4)
#define MOTION_HOLD_BARCODE   UINT32_C(8)  /* a barcode stop; START releases it */
#define MOTION_HOLD_LIFT      UINT32_C(16) /* lifted or overturned; START releases it */
#define MOTION_HOLD_SUPERVISOR UINT32_C(32) /* the run permit lapsed (D120) */
#define MOTION_HOLD_ALL       UINT32_C(61)

/* One coherent feedback snapshot. Speeds and displacements are signed and
 * positive car-forward. fresh means the sample is within the age limit. */
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
} motion_feedback_t;

STATIC_ASSERT (sizeof(motion_feedback_t) == 40U, "motion_feedback_t size");

typedef struct
{
    uint8_t  state;          /* MOTION_STATE_* */
    uint8_t  fault;          /* controller fault code */
    bool     ready;
    bool     stop_requested; /* Direct emergency latch. */
    uint32_t hold_mask;      /* MOTION_HOLD_* */
    int      init_error;
} motion_status_t;

STATIC_ASSERT (sizeof(motion_status_t) == 12U, "motion_status_t size");

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

int motion_set_speed (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us);
int motion_set_hold (uint32_t reason, bool held);
int motion_request_stop (void);

/* The supervisor renews the run permit for lease_ms. When the permit lapses,
 * the motion task sets MOTION_HOLD_SUPERVISOR by itself, and the car brakes
 * even if the supervisor hangs. The permit is not enforced until the first
 * renewal. */
int motion_renew_permit (uint32_t lease_ms);

/* The supervisor tells the motion task that the encoders answer crosswise
 * (D161). The motion task adds the encoder_dead fault bit while it is true. */
void motion_note_encoder_crossed (bool b_crossed);

#endif /* MOTION_MOTION_H */

/*** end of file ***/
