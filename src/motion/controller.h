/** @file controller.h
 * @brief Single-owner feedback-to-output controller, independent of the RTOS.
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pid.h"
#include "motion/types.h"

typedef struct
{
    uint32_t id;
    uint32_t issued_us;
    uint32_t ttl_us;
    int32_t  target_um_s[ENCODER_WHEELS];
} motion_command_t;

typedef struct
{
    odometry_config_t    sampling;
    pid_config_t speed[ENCODER_WHEELS];
    uint32_t           max_command_ttl_us;
    uint32_t           max_control_gap_us;
    uint32_t           feedback_timeout_us;
    uint32_t           reversal_brake_us;
} controller_config_t;

typedef enum
{
    CONTROLLER_APPLIED = 0,
    CONTROLLER_BRAKED_RETRY,
    CONTROLLER_OUTPUT_FAILED
} controller_apply_result_t;

/* Callbacks belong to the PWM owner. RETRY means the driver has braked both
 * wheels; the controller enforces a timed brake interval before retrying. */
typedef struct
{
    void * context;
    controller_apply_result_t (*apply)(void * context, int16_t left, int16_t right);
    bool (*brake)(void * context, bool latch);
} controller_output_t;

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
    odometry_sample_t sample;
} controller_status_t;

typedef struct
{
    controller_config_t config;
    controller_output_t         output;
    odometry_t        sampler;
    pid_state_t             pid[ENCODER_WHEELS];
    motion_command_t        command;
    controller_status_t status;
    uint32_t                previous_tick_us;
    uint32_t                brake_started_us;
    uint32_t                progress_us[ENCODER_WHEELS];
    int64_t                 last_edges[ENCODER_WHEELS];
    int8_t                  drive_sign[ENCODER_WHEELS];
    bool                    braked;
    bool                    initialised;
} controller_t;

/* No default speed/gain/timing settings. Init issues brake through the output
 * callback; the hardware owner must already be initialised. One task owns all
 * calls. Stop/fault states are terminal: no public reset/rearm operation. */
bool controller_init (controller_t *              p_control,
                          const controller_config_t * p_config,
                          const odometry_input_t *          p_baseline,
                          const controller_output_t *         p_output);

/* Configuration-only check: no output callback or hardware access. */
bool controller_config_valid (const controller_config_t * p_config);

/* A new command replaces the previous command; NULL keeps its original TTL.
 * Stop has priority over feedback, expiry and new commands. Both targets zero
 * is an ordinary, non-latched brake. Fault/stop always brakes and latches the
 * hardware output. No blocking/printing/capture-lock work in this function.
 * A long task gap is detected only on return, not by a hardware watchdog.
 * Output direction is limited to the target sign (overspeed requests brake,
 * not active reverse drive). Reversals preserve an explicit braked interval.
 * Timestamps and counters must be updated within one 2^32-us cycle. */
void controller_tick (controller_t *     p_control,
                          const odometry_input_t * p_input, uint32_t now_us,
                          const motion_command_t * p_command,
                          bool                     stop_requested);

#endif /* CONTROLLER_H */
