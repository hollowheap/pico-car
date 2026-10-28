/** @file motion_control.h
 * @brief Single-owner feedback-to-output controller, independent of the RTOS.
 */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "wheel_pid.h"

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
    MOTION_FAULT_NO_EDGES,
    MOTION_FAULT_OUTPUT
} motion_fault_t;

typedef struct
{
    uint32_t id;
    uint32_t issued_us;
    uint32_t ttl_us;
    int32_t  target_um_s[ENCODER_WHEELS];
} motion_command_t;

typedef struct
{
    motion_config_t    sampling;
    wheel_pid_config_t speed[ENCODER_WHEELS];
    uint32_t           max_command_ttl_us;
    uint32_t           max_control_gap_us;
    uint32_t           feedback_timeout_us;
    uint32_t           reversal_brake_us;
} motion_control_config_t;

typedef enum
{
    MOTION_APPLIED = 0,
    MOTION_BRAKED_RETRY,
    MOTION_OUTPUT_FAILED
} motion_apply_result_t;

/* Callbacks belong to the PWM owner. RETRY means the driver has braked both
 * wheels; the controller enforces a timed brake interval before retrying. */
typedef struct
{
    void * context;
    motion_apply_result_t (*apply)(void * context, int16_t left, int16_t right);
    bool (*brake)(void * context, bool latch);
} motion_output_t;

typedef struct
{
    motion_state_t  state;
    motion_fault_t  fault;
    uint32_t        command_id;
    uint32_t        timestamp_us;
    int32_t         target_um_s[ENCODER_WHEELS];
    int16_t         duty[ENCODER_WHEELS];
    bool            sample_available;
    bool            brake_failed;
    motion_sample_t sample;
} motion_control_status_t;

typedef struct
{
    motion_control_config_t config;
    motion_output_t         output;
    motion_sampler_t        sampler;
    wheel_pid_t             pid[ENCODER_WHEELS];
    motion_command_t        command;
    motion_control_status_t status;
    uint32_t                previous_tick_us;
    uint32_t                brake_started_us;
    uint32_t                progress_us[ENCODER_WHEELS];
    int64_t                 last_edges[ENCODER_WHEELS];
    int8_t                  drive_sign[ENCODER_WHEELS];
    bool                    braked;
    bool                    initialised;
} motion_control_t;

/* No default speed/gain/timing settings. Init issues brake through the output
 * callback; the hardware owner must already be initialised. One task owns all
 * calls. Stop/fault states are terminal: no public reset/rearm operation. */
bool motion_control_init (motion_control_t *              p_control,
                          const motion_control_config_t * p_config,
                          const motion_input_t *          p_baseline,
                          const motion_output_t *         p_output);

/* Configuration-only check: no output callback or hardware access. */
bool motion_control_config_valid (const motion_control_config_t * p_config);

/* A new command replaces the previous command; NULL keeps its original TTL.
 * Stop has priority over feedback, expiry and new commands. Both targets zero
 * is an ordinary, non-latched brake. Fault/stop always brakes and latches the
 * hardware output. No blocking/printing/capture-lock work in this function.
 * A long task gap is detected only on return, not by a hardware watchdog.
 * Output direction is limited to the target sign (overspeed requests brake,
 * not active reverse drive). Reversals preserve an explicit braked interval.
 * Timestamps and counters must be updated within one 2^32-us cycle. */
void motion_control_tick (motion_control_t *     p_control,
                          const motion_input_t * p_input, uint32_t now_us,
                          const motion_command_t * p_command,
                          bool                     stop_requested);

#endif /* MOTION_CONTROL_H */
