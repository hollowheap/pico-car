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
#define MOTION_HOLD_LINE_LOST UINT32_C(2)
#define MOTION_HOLD_OPERATOR  UINT32_C(4)
#define MOTION_HOLD_ALL       UINT32_C(7)

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

typedef struct
{
    uint8_t  state;          /* MOTION_STATE_* */
    uint8_t  fault;          /* controller fault code */
    bool     ready;
    bool     stop_requested; /* Direct emergency latch. */
    uint32_t hold_mask;      /* MOTION_HOLD_* */
    int      init_error;
} motion_status_t;

int motion_feedback_snapshot (motion_feedback_t * p_feedback);
int motion_status (motion_status_t * p_status);

int motion_set_speed (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us);
int motion_set_hold (uint32_t reason, bool held);
int motion_request_stop (void);

#endif /* MOTION_MOTION_H */

/*** end of file ***/
