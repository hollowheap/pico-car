/** @file telem.c
 * @brief Supervisor event and acknowledgement handoff.
 */

#include "internal.h"

#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "platform/log.h"
#include "supervisor/edge_events.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/wire_schema.h"

/* Event p1 of cmd_accept and cmd_reject: reason << 12 | source << 8 | opcode.
   p2 is the cmd_id. */
#define CMD_EVENT_SOURCE_SHIFT (8U)
#define CMD_EVENT_REASON_SHIFT (12U)

#define STOP_TIMING_MAX_MS (UINT16_MAX)

#define BUILD_FLAGS_SHIFT     (12U)
#define BUILD_HASH_HIGH_SHIFT (16U)
#define BUILD_HASH_HIGH_MASK  (0xFFFU)
#define BUILD_HASH_LOW_MASK   (0xFFFFU)

static edge_watch_t g_edge;

typedef enum
{
    STOP_EPISODE_READY = 0,
    STOP_EPISODE_TIMING,
    STOP_EPISODE_REPORTED
} stop_episode_t;

typedef struct
{
    uint32_t accepted_ms;
    uint32_t duty_ms;
    uint32_t rest_stamp_us;
    uint32_t movement_sequence;
    uint8_t  state;
    bool     b_duty_zero : 1;
    bool     b_rest_sample : 1;
    bool     b_estop_seen : 1;
} stop_timing_t;
STATIC_ASSERT (sizeof (stop_timing_t) == 20U, "stop timing state size");
static stop_timing_t g_stop_timing;

void supervisor_telem_init (void)
{
    g_stop_timing = (stop_timing_t) {0};
    edge_init (&g_edge);
}

static uint32_t odometer_mm (void)
{
    motion_feedback_t feedback;
    uint32_t          mm = 0U;

    if (0 == motion_feedback_snapshot (&feedback))
    {
        mm = telemetry_odometer_mm (&feedback);
    }

    return mm;
}

void supervisor_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2)
{
    (void) telemetry_try_emit_event (source, type, p1, p2, odometer_mm ());
}

static void emit_cmd_ack (uint8_t type, int16_t p1, uint16_t cmd_id)
{
    (void) telemetry_post_cmd_ack (type, p1, cmd_id, odometer_mm ());
}

void supervisor_turn_refused (uint8_t opcode)
{
    supervisor_set_turning (false);
    emit_cmd_ack ((uint8_t) TELEMETRY_EVT_CMD_REJECT,
                  (int16_t) (((uint32_t) RUN_REJECT_BUSY << CMD_EVENT_REASON_SHIFT) | (uint32_t) opcode),
                  0U);
}

void supervisor_report_command (vehicle_cmd_t cmd, const run_result_t * p_res)
{
    uint32_t const what = ((uint32_t) cmd.source << CMD_EVENT_SOURCE_SHIFT) | (uint32_t) cmd.opcode;

    if (p_res->b_accepted)
    {
        emit_cmd_ack ((uint8_t) TELEMETRY_EVT_CMD_ACCEPT, (int16_t) what, cmd.cmd_id);
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "command %d from %d accepted, id=%d run=%d",
                  (int) cmd.opcode, (int) cmd.source, (int) cmd.cmd_id, (int) cmd.run_id);
    }
    else if (p_res->b_rejected)
    {
        emit_cmd_ack ((uint8_t) TELEMETRY_EVT_CMD_REJECT,
                      (int16_t) (what | ((uint32_t) p_res->reason << CMD_EVENT_REASON_SHIFT)),
                      cmd.cmd_id);
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "command %d from %d refused (%d), id=%d run=%d",
                  (int) cmd.opcode, (int) cmd.source, (int) p_res->reason, (int) cmd.cmd_id,
                  (int) cmd.run_id);
    }
    else
    {
        /* Nothing to report. */
    }
}

void supervisor_arm_stop_timing (void)
{
    SYSTIM time = {0, 0U};

    if ((STOP_EPISODE_READY == g_stop_timing.state) && (E_OK == tk_get_otm (&time)))
    {
        g_stop_timing = (stop_timing_t) {
            .accepted_ms = time.lo, .state = STOP_EPISODE_TIMING
        };
    }
}

void supervisor_watch_stop_timing (bool b_estop, bool b_reported)
{
    stop_timing_t * const p_timing = &g_stop_timing;
    motion_status_t   motion;
    motion_feedback_t feedback;
    motion_diag_t     diag;
    SYSTIM            time = {0, 0U};

    if (((STOP_EPISODE_REPORTED == p_timing->state) != b_reported) ||
        ((STOP_EPISODE_READY == p_timing->state) && !b_estop))
    {
        return;
    }
    bool const observed = (0 == motion_snapshot (&motion, &feedback, &diag));
    if (E_OK != tk_get_otm (&time))
    {
        return;
    }
    bool const valid = observed && feedback.fresh && feedback.speed_valid[0] && feedback.speed_valid[1];
    bool const at_rest = valid && (0 == feedback.left_speed_um_s) && (0 == feedback.right_speed_um_s);
    if (STOP_EPISODE_REPORTED == p_timing->state)
    {
        /* The motion owner retains measured travel even when GP21 latches
           before the supervisor sees the moving wheel snapshot. */
        if (!observed || (motion.movement_sequence == p_timing->movement_sequence))
        {
            return;
        }
        *p_timing = (stop_timing_t) {0};
    }
    if (observed && motion.stop_requested && !p_timing->b_estop_seen)
    {
        if ((STOP_EPISODE_READY == p_timing->state) ||
            (motion.stop_age_ms > (time.lo - p_timing->accepted_ms)))
        {
            p_timing->accepted_ms = time.lo - motion.stop_age_ms;
        }
        p_timing->b_estop_seen = true;
        p_timing->state = STOP_EPISODE_TIMING;
    }
    if (STOP_EPISODE_TIMING != p_timing->state)
    {
        return;
    }
    if (observed)
    {
        p_timing->movement_sequence = motion.movement_sequence;
    }
    uint32_t const elapsed_ms = time.lo - p_timing->accepted_ms;
    bool const zero_duty = observed && (0 == diag.left_duty_permille) &&
                           (0 == diag.right_duty_permille) &&
                           (TELEMETRY_MOTION_FAULT_OUTPUT != motion.fault);
    if (zero_duty && !p_timing->b_duty_zero)
    {
        p_timing->b_duty_zero = true;
        p_timing->duty_ms = time.lo;
    }
    bool const rest_sample = zero_duty && at_rest;
    bool const confirmed = rest_sample && p_timing->b_rest_sample &&
                           (feedback.timestamp_us != p_timing->rest_stamp_us);
    p_timing->b_rest_sample = rest_sample;
    if (rest_sample)
    {
        p_timing->rest_stamp_us = feedback.timestamp_us;
    }
    if (confirmed || (elapsed_ms >= STOP_TIMING_MAX_MS))
    {
        uint32_t const duty = p_timing->b_duty_zero
                                  ? (p_timing->duty_ms - p_timing->accepted_ms)
                                  : STOP_TIMING_MAX_MS;
        uint16_t const rest = (confirmed && (elapsed_ms < STOP_TIMING_MAX_MS))
                                  ? (uint16_t) elapsed_ms : STOP_TIMING_MAX_MS;
        p_timing->state = STOP_EPISODE_REPORTED;
        (void) telemetry_try_emit_event ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR,
                                         (uint8_t) TELEMETRY_EVT_STOP_TIMING,
                                         (int16_t) ((duty > INT16_MAX) ? INT16_MAX : duty),
                                         rest, observed ? telemetry_odometer_mm (&feedback) : 0U);
    }
}

/* Send the events a state change produced. */
static void emit_edges (uint8_t source, const edge_event_t * p_events, uint8_t count)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < count; idx++)
    {
        supervisor_emit_event (source, p_events[idx].type, p_events[idx].p1, p_events[idx].p2);
    }
}

/* Run start and stop. The legacy scanner reports its own obstacle cycle. */
void supervisor_watch_edges (run_state_t state, bool b_unverified_start)
{
    edge_event_t ev[EDGE_EVENTS_MAX];
    uint8_t const count = edge_run (&g_edge, (uint8_t) state, ev);

    if (b_unverified_start && (RUN_STATE_STOPPED == state) && (count != 0U))
    {
        ev[0].p2 = (uint16_t) TELEMETRY_RUN_STOP_REASON_STARTUP_UNVERIFIED;
    }
    emit_edges ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, ev, count);
    obstacle_status_t obstacle;

    if (E_OK == obstacle_snapshot (&obstacle))
    {
        obstacle_plan_summary_t plan = { 0U, 0U, 0U, 0U, 0U, 0U, false, 0U };
        edge_obstacle_in_t      in;

        (void) obstacle_copy_plan (&plan);
        in.state          = obstacle.state;
        in.cause          = obstacle.cause;
        /* The stop event's p2 is the give-up reason for BYPASS_GAVE_UP,
           else the echo class. */
        in.echo_class     = (TELEMETRY_OBSTACLE_CAUSE_BYPASS_GAVE_UP == obstacle.cause)
                                ? obstacle.stop_reason
                                : obstacle.echo_class;
        in.distance_mm    = obstacle.distance_mm;
        in.plan_status    = plan.status;
        in.plan_side      = plan.side;
        in.plan_start_deg = plan.start_deg;
        in.plan_width_mm  = plan.width_mm;
        emit_edges ((uint8_t) TELEMETRY_EVENT_SOURCE_OBSTACLE, ev, edge_obstacle (&g_edge, &in, ev));
    }
}

void supervisor_report_stall (uint8_t id, uint8_t misses)
{
    supervisor_emit_event ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR,
                           (uint8_t) TELEMETRY_EVT_TASK_STALL,
                           (int16_t) id, (uint16_t) misses);
    LOG_ERROR (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "stall id=%d, misses=%d", (int) id, (int) misses);
}

void supervisor_report_build (void)
{
    uint32_t flags = 0U;
    uint32_t hash  = 0U;

#ifdef PICO_CAR_BUILD_HASH
    hash = PICO_CAR_BUILD_HASH;
#if PICO_CAR_BUILD_DIRTY
    flags |= TELEMETRY_BUILD_FLAG_DIRTY;
#endif
#else
    flags |= TELEMETRY_BUILD_FLAG_UNKNOWN;
#endif
    flags |= TELEMETRY_BUILD_FLAG_PILOT;
#ifdef PICO_CAR_WATCHDOG
    flags |= TELEMETRY_BUILD_FLAG_WATCHDOG;
#endif

    uint32_t const high = (hash >> BUILD_HASH_HIGH_SHIFT) & BUILD_HASH_HIGH_MASK;

    supervisor_emit_event ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR,
                           (uint8_t) TELEMETRY_EVT_BUILD,
                           (int16_t) ((flags << BUILD_FLAGS_SHIFT) | high),
                           (uint16_t) (hash & BUILD_HASH_LOW_MASK));
}

/*** end of file ***/
