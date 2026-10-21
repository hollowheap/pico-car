/** @file speed_probe.h
 * @brief Explicit experimental settings and once-only PI bench-run policy.
 */
#ifndef SPEED_PROBE_H
#define SPEED_PROBE_H

#include "motion_control.h"
#include "motor_pulse.h"

#define SPEED_PROBE_TARGET_UM_S INT32_C(50000)
#define SPEED_PROBE_DUTY_CAP 200U
#define SPEED_PROBE_DURATION_US UINT32_C(3000000)
#define SPEED_PROBE_TTL_US UINT32_C(200000)
#define SPEED_PROBE_MAX_GAP_US UINT32_C(100000)

typedef enum
{
    SPEED_PROBE_WAIT = 0,
    SPEED_PROBE_RUN,
    SPEED_PROBE_FINISHED
} speed_probe_phase_t;

typedef struct
{
    motor_pulse_t gate;
    speed_probe_phase_t phase;
    uint32_t started_us;
    uint32_t previous_us;
    uint32_t max_gap_us;
    bool gap_fault;
    bool interrupted;
} speed_probe_t;

/* Bench-only initial settings. NOT validated/tuned motor operating limits.
 * Provisional 67mm wheel scales stay flagged unverified; D is disabled. */
motion_control_config_t speed_probe_configuration (void);
void speed_probe_init (speed_probe_t * p_probe);

/* Stop/fault wins even at the start/deadline. One released/pressed GP20
 * starts one run. No restart. Time/gap limits are software decisions checked
 * on the next call, not an independent hardware power cutoff. */
speed_probe_phase_t speed_probe_update (speed_probe_t * p_probe,
    uint32_t now_us, bool start_pressed, bool stop_or_fault);

#endif /* SPEED_PROBE_H */
