/** @file motion.h
 * @brief Optional micro T-Kernel motor owner and cross-core command interface.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <tk/tkernel.h>
#include "motion/feedback.h"
#include "motion/types.h"

typedef struct
{
    bool           ready;
    bool           stop_requested;
    motion_state_t state;
    motion_fault_t fault;
    uint32_t       hold_mask;
} motion_status_t;

/* Lifecycle: call ONCE from core-0 startup task before launching producers
 * or status readers. Reserves encoder bank IRQ and motor PWM slices4/5.
 * E_OK means task creation succeeded, not that hardware init/tuning passed:
 * read status. No default gains, boot command, automatic arming or diagnostic
 * button run. */
ER motion_init (void);
void motion_run (void);

/* Bind the single normal command producer. Call once from mission control. */
ER motion_bind (ID tid);

/* Decoded sensor intent; original sensed_us/valid_for_us is preserved.
 * Invalid/stale requests cancel pending motion and request an ordinary brake.
 * E_BUSY means a sensor/operator hold prevents motion. No sensor GPIO reads. */
ER motion_submit (const guidance_request_t * p_request);
/* Each owner sets/clears ONE of MOTION_HOLD_OBSTACLE/LINE_LOST/OPERATOR.
 * Clearing one never clears another. Last release requires a fresh request
 * sensed at/after release; cached old intent cannot restart motion. */
ER motion_hold (uint32_t reason, bool held);
ER motion_stop (
    void); /* Latches until reboot; next owner poll. */
ER motion_status (motion_status_t * p_status);

/* Read-only motion feedback copy. Caller cannot mutate the controller. */
ER motion_feedback_snapshot (motion_feedback_t * p_feedback);

/* Direct emergency path. FLG_EMG_STOP wakes the emergency service. */
#define FLG_EMG_STOP (1U << 0)
extern ID flg_ctrl_id;
void motion_emergency_service (void);

#endif /* MOTION_H */

/*** end of file ***/
