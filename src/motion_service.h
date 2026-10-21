/** @file motion_service.h
 * @brief Optional micro T-Kernel motor owner and cross-core command interface.
 */
#ifndef MOTION_SERVICE_H
#define MOTION_SERVICE_H

#include <tk/tkernel.h>
#include "motion_control.h"
#include "motion_request.h"
#include "motion_inbox.h"

typedef struct
{
    motion_control_config_t control;
    uint32_t poll_delay_ms; /* Nominal delay, not a guaranteed task period. */
} motion_service_config_t;

typedef struct
{
    motion_control_status_t control;
    ER init_error;
    bool ready;
    bool stop_requested;
    uint32_t hold_mask; /* Independent normal sensor holds; not emergency stop. */
} motion_service_status_t;

/* Lifecycle: call ONCE from core-0 startup task before launching producers
 * or status readers. Reserves encoder bank IRQ and motor PWM slices4/5.
 * Invalid configuration returns E_PAR without starting a task. E_OK means
 * task creation succeeded, not that hardware init/tuning passed: read status.
 * No default gains, boot command, automatic arming or diagnostic button run.
 * Integrator must start only after agreeing pin/IRQ/resource ownership. */
ER motion_service_start (const motion_service_config_t * p_config);

/* Task-context APIs on either core after the start call returns. Latest
 * pending speeds replace the previous pending speeds; no unbounded queue.
 * E_OK means accepted, not applied. Freshness/quality of upstream IR/sonar
 * decisions remains the caller's responsibility. TTL starts at acceptance.
 * Zero/zero is an ordinary brake and can be followed by a fresh speed request.
 * Pending motion cannot overwrite stop. Stop/fault rejects later commands.
 * API struct widths are internal, not a final minimum-bit wire format. */
ER motion_service_set_speed (int32_t left_um_s, int32_t right_um_s,
                             uint32_t ttl_us);
/* Decoded sensor intent; original sensed_us/valid_for_us is preserved.
 * Invalid/stale requests cancel pending motion and request an ordinary brake.
 * E_BUSY means a sensor/operator hold prevents motion. No sensor GPIO reads. */
ER motion_service_submit_request (const motion_drive_request_t * p_request);
/* Each owner sets/clears ONE of MOTION_HOLD_OBSTACLE/LINE_LOST/OPERATOR.
 * Clearing one never clears another. Last release requires a fresh request
 * sensed at/after release; cached old intent cannot restart motion. */
ER motion_service_set_hold (uint32_t reason, bool held);
ER motion_service_request_stop (void); /* Latches until reboot; next owner poll. */
ER motion_service_read_status (motion_service_status_t * p_status);

#endif /* MOTION_SERVICE_H */
