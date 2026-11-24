/** @file controller.h
 * @brief Single-owner feedback-to-output controller, independent of the RTOS.
 */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "pid.h"
#include "platform/static_assert.h"

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_RUNNING,
    MOTION_REVERSING,
    MOTION_STOPPED,
    MOTION_FAULT
} motion_state_t;

typedef enum
{
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_COMMAND,
    MOTION_FAULT_EXPIRED,
    MOTION_FAULT_FEEDBACK,
    MOTION_FAULT_GAP,
    MOTION_FAULT_NO_EDGES, /* no longer raised; kept so wire codes hold */
    MOTION_FAULT_OUTPUT
} motion_fault_t;

/* Earliest release of the brake after an encoder or timing fault. Both encoders
 * must also supply a fresh valid interval. Driving needs a new command. [Default] */
#define MOTION_FAULT_CLEAR_US (500000U)

typedef struct
{
    uint32_t id;
    uint32_t issued_us;
    uint32_t ttl_us;
    int32_t  target_um_s[ENCODER_WHEELS];
    bool     b_scan_pivot;
} motion_command_t;

typedef struct
{
    motion_config_t    sampling;
    pid_config_t speed[ENCODER_WHEELS];
    uint32_t           max_command_ttl_us;
    uint32_t           max_control_gap_us;
    uint32_t           reversal_brake_us;
    uint16_t           pivot_duty_permille; /* Zero uses the normal limit. */
} controller_config_t;

typedef enum
{
    MOTION_APPLIED = 0,
    MOTION_BRAKED_RETRY,
    MOTION_OUTPUT_FAILED,
    MOTION_STOP_PENDING
} motion_apply_result_t;

/* Callbacks belong to the PWM owner. RETRY means the driver has braked both
 * wheels; the controller enforces a timed brake interval before retrying.
 * STOP_PENDING means drive was inhibited and the controller must latch brake. */
typedef struct
{
    void * context;
    motion_apply_result_t (*apply)(void * context, int16_t left, int16_t right);
    bool (*brake)(void * context, bool latch);
} motion_output_t;

typedef struct
{
    motion_sample_t sample;
    int32_t         target_um_s[ENCODER_WHEELS];
    uint32_t        command_id;
    uint32_t        timestamp_us;
    motion_state_t  state;
    motion_fault_t  fault;
    int16_t         duty[ENCODER_WHEELS];
    bool            sample_available;
    bool            brake_failed;
} controller_status_t;

/* Copied across tasks under a spinlock: widest fields first, so padding is at
 * the tail only. The enums are 1 byte on the firmware (96 bytes) and 4 on the
 * host (104 bytes). */
STATIC_ASSERT ((sizeof(controller_status_t) <= 104U) &&
                   ((sizeof(controller_status_t) % 8U) == 0U),
               "controller_status_t size");

typedef struct
{
    controller_config_t config;
    motion_output_t         output;
    odometry_t        sampler;
    pid_t             pid[ENCODER_WHEELS];
    motion_command_t        command;
    controller_status_t status;
    uint32_t                previous_tick_us;
    uint32_t                brake_started_us;
    uint8_t                 bad_samples[ENCODER_WHEELS];
    int8_t                  drive_sign[ENCODER_WHEELS];
    uint32_t                fault_us;
    bool                    braked;
    bool                    initialised;
    bool                    recovery_baseline;
} controller_t;

/* No default speed/gain/timing settings. Init issues brake through the output
 * callback; the hardware owner must already be initialised. One task owns all
 * calls. Stop and the COMMAND, EXPIRED, GAP and OUTPUT faults are terminal:
 * no public reset/rearm operation. FEEDBACK clears on a valid capture
 * interval once the brake has held for MOTION_FAULT_CLEAR_US. */
bool controller_init (controller_t *              p_control,
                          const controller_config_t * p_config,
                          const motion_input_t *          p_baseline,
                          const motion_output_t *         p_output);

/* FEEDBACK, EXPIRED and GAP brake without the output latch and clear after
 * MOTION_FAULT_CLEAR_US; the other faults latch. */
bool controller_fault_clears_in_place (motion_fault_t reason);

/* Configuration-only check: no output callback or hardware access. */
bool controller_config_valid (const controller_config_t * p_config);

/* A new command replaces the previous command; NULL keeps its original TTL.
 * Stop has priority over feedback, expiry and new commands. Both targets zero
 * is an ordinary, non-latched brake. Fault/stop always brakes; it latches the
 * hardware output unless the fault clears in place. No blocking/printing/capture-lock work in this function.
 * A long task gap is detected only on return, not by a hardware watchdog.
 * Output direction is limited to the target sign (overspeed requests brake,
 * not active reverse drive). Reversals preserve an explicit braked interval.
 * Timestamps and counters must be updated within one 2^32-us cycle. */
void controller_tick (controller_t *     p_control,
                          const motion_input_t * p_input, uint32_t now_us,
                          const motion_command_t * p_command,
                          bool                     stop_requested);

#endif /* MOTION_CONTROL_H */
