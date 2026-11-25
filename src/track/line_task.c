/** @file line_task.c
 *
 * @brief Line task body: follow the line, run the turns the supervisor
 *        allowed, and give the motors to the obstacle step when it asks.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>

#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "platform/drv_timer.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/units.h"
#include "physics/physics.h"
#include "pilot/drive.h"
#include "pilot/pilot.h"
#include "pilot/pilot_log.h"
#include "navigation/obstacle_ping.h"
#include "track/start_check.h"
#include "supervisor/command.h"
#include "supervisor/supervisor.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/hug_word.h"
#include "telemetry/queue.h"
#include "telemetry/wire_schema.h"
#include "track/barcode.h"
#include "track/line_control.h"
#include "track/line_task.h"
#include "track/track.h"
#include "ir_pins.h"

/* Line following. The cruise speed, the search limits and the turn speeds
 * live in track/line_control.h (tune them there on the car). The duty cap in
 * motion_setup() must leave headroom for them. The follower steers by a
 * heading target and stops only when the search finds nothing; one-shot turns
 * pivot in place.
 * LINE_POLL_MS is the control period. */
#define LINE_POLL_MS             (10U)
/* The sampler runs every 5 ms. Allow for scheduling delays before braking. */
#define LINE_SAMPLE_MAX_AGE_US    (25000U)
/* Upper bound on one turn, so a blocked car cannot pivot for ever. Tune on
 * the car: it must exceed the longest U-turn, including the slow phase. */
#define LINE_TURN_MAX_MS         (4000U)
/* Crossing back to the only verified exit can take one more quarter turn
 * at tape-capture speed. The limit still runs from the start of the scan. */
#define LINE_FALLBACK_ALIGN_MS   (1000U)

static line_route_t g_route;

/* Wheel displacement units. */
#define DECI_PER_DEG (10)

/* line_lost p2 flag of a retry; the low bits are the retry number. */
#define LINE_RETRY_EVENT (0x8000U)
/* An event p2 that packs two bytes: high byte * LINE_EVENT_HIGH_BYTE + low byte. */
#define LINE_EVENT_HIGH_BYTE (256U)

/* Map a vehicle command opcode to a turn. */
static line_turn_kind_t line_turn_of (uint8_t opcode)
{
    line_turn_kind_t turn = LINE_TURN_KIND_NONE;

    switch (opcode)
    {
        case VEHICLE_CMD_LEFT:
            turn = LINE_TURN_KIND_LEFT;
            break;
        case VEHICLE_CMD_RIGHT:
            turn = LINE_TURN_KIND_RIGHT;
            break;
        case VEHICLE_CMD_UTURN:
            turn = LINE_TURN_KIND_UTURN;
            break;
        case VEHICLE_CMD_STRAIGHT:
            turn = LINE_TURN_KIND_STRAIGHT;
            break;
        default:
            break;
    }
    return turn;
}

/* Mean wheel travel divides the sum of both wheels. */
#define LINE_WHEEL_COUNT         (2)

/* Each speed lease must outlast one loop pass plus scheduling delay. The
 * motion task (priority 1) shares core 0 with line (3), obstacle (4) and
 * physics (5), so a 3-pass lease can be stale by the time the controller
 * validates it and latches MOTION_FAULT_COMMAND. Keep the lease below
 * max_command_ttl_us (200 ms). */
#define LINE_US_PER_MS           (1000U)
#define LINE_LEASE_US            (PILOT_COMMAND_LEASE_MS * LINE_US_PER_MS)

static bool          g_line_lost;
static int64_t       g_line_travel_um;

/* Pilot, drive layer and the last decision. Only the line task touches them. */
static pilot_state_t g_pilot;
static drive_state_t g_drive;
static pilot_log_t   g_pilot_log;
static uint32_t      g_pilot_last_ms;
static uint32_t      g_pilot_stamp_ms;
static line_watch_t  g_pilot_watch;       /* line-lost bookkeeping */
static uint32_t      g_pilot_ping_sent;   /* last ping request sent, 0 for none */
static bool          g_pilot_ping_warned;
static uint8_t       g_pilot_reset_seen;  /* reset count last acted on */
static bool          g_continuous_floor;
static bool          g_hug_leg_reported;
static uint8_t       g_pilot_start_seen;  /* start count last acted on */
static uint8_t       g_recovery_seen;
static uint32_t      g_hug_seq; /* line task only; the field keeps the low 2 bits */

/* The hug snapshot for the obstacle frame. The line task writes both words
 * together; the telemetry task reads both, so one frame never mixes two
 * ticks. The lock needs no set-up call: a spinlock is free at zero, and a
 * static starts at zero before any task runs. */
static T_SPLOCK      g_hug_lock;
static uint32_t      g_hug_words[HUG_WORD_COUNT];

/* Read by the telemetry task (diag packet): line-lost entries since boot, and
 * whether a one-shot turn is running. Written by the line task only. */
static volatile uint16_t g_line_lost_count;
static volatile bool     g_turning;

/* Odometer for an event, from the motion snapshot (0 if it is not running). */
static uint32_t odometer_mm (void)
{
    motion_feedback_t feedback;
    uint32_t          mm = 0U;

    if (E_OK == motion_feedback_snapshot(&feedback))
    {
        mm = telemetry_odometer_mm(&feedback);
    }

    return mm;
}

/* Kernel time in ms (low 32 bits; line_control handles the wrap). If the
 * kernel call fails the last good time is reused, so the clock never jumps
 * back to 0. */
static uint32_t line_now_ms (void)
{
    static uint32_t s_last_ms;
    SYSTIM          tim = {0, 0U};

    if (E_OK == tk_get_otm(&tim))
    {
        s_last_ms = tim.lo;
    }

    return s_last_ms;
}

/* Only the line task consumes this indication. A raw edge can begin grace
 * but only progress on a framed barcode can renew blind travel. */
static bool line_barcode_advanced (uint8_t * p_prefix_mm)
{
    static uint8_t s_seen;
    uint8_t const progress = barcode_progress_count(p_prefix_mm);
    bool const advanced = s_seen != progress;
    s_seen = progress;
    return advanced;
}

static bool line_sample_fresh (uint32_t timestamp_us)
{
    static uint32_t s_last_timestamp_us;
    static bool     s_stale;
    uint32_t const  age_us = now_us() - timestamp_us;
    /* Once stale, the same sample cannot become fresh on timer rollover. */
    bool const b_fresh = (age_us <= LINE_SAMPLE_MAX_AGE_US) &&
                        (!s_stale || (timestamp_us != s_last_timestamp_us));

    if (!b_fresh && !s_stale)
    {
        LOG_WARN(LOG_TASK_LINE, LOG_MOD_NONE, "line sample timed out");
    }
    s_last_timestamp_us = timestamp_us;
    s_stale = !b_fresh;
    return b_fresh;
}

/* The follower a turn hands its side to: the pilot's own with the pilot on. */
static line_follow_t * line_follower (void)
{
    return &g_pilot.follow;
}

/* Mean wheel travel at the start of the present line loss. */
static int64_t g_line_lost_at_um;

/* Set FAULT_LINE_LOST_BIT on entering line lost, clear it on leaving. The
 * events carry the remembered side and, on leaving, the distance driven while
 * lost. */
static void line_update_fault (bool b_lost)
{
    if (b_lost && !g_line_lost)
    {
        system_fault_assign(FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, FAULT_LINE_LOST_BIT);
        g_line_lost_count++;
        g_line_lost_at_um = g_line_travel_um;
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                       (int16_t)line_follower()->side, 0U, odometer_mm());
    }
    else if (!b_lost && g_line_lost)
    {
        int64_t  const driven_um = g_line_travel_um - g_line_lost_at_um;
        uint32_t const driven_mm = (uint32_t)(((driven_um < 0) ? -driven_um : driven_um) / UM_PER_MM);
        uint16_t const driven_u16 = (driven_mm > UINT16_MAX) ? UINT16_MAX : (uint16_t)driven_mm;

        system_fault_assign(FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, 0U);
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_LINE_FOUND,
                                       (int16_t)line_follower()->side, driven_u16,
                                       odometer_mm());
    }
    else
    {
        /* No change. */
    }
    g_line_lost = b_lost;
}

/* At a crest the follower bridges a longer gap. The pilot also reads the
 * roll from the same snapshot when the attitude is known, the pivot witness,
 * and the gated magnetometer heading. */
static bool line_on_hump (int16_t * p_roll_cdeg, pilot_meas_t * p_meas)
{
    physics_status_t physics;
    bool             b_hump = false;

    if ((E_OK == physics_snapshot(&physics)) && physics.ready)
    {
        b_hump = physics.hump_active || (TELEMETRY_HUMP_STATE_FLAT != physics.state);
        if ((NULL != p_roll_cdeg) && physics.calibrated && !physics.attitude_unknown &&
            (TELEMETRY_ATTITUDE_STATE_OK == physics.attitude_state))
        {
            *p_roll_cdeg = physics.roll_cdeg;
        }
        if (NULL != p_meas)
        {
            p_meas->pivot_scale_pct = physics.pivot_scale_pct;
            p_meas->b_mag_heading = physics.heading_gated;
            p_meas->mag_heading_ddeg = physics.heading_gated ? physics.heading_ddeg : 0U;
        }
    }

    return b_hump;
}

/* A retry of the search leaves a line_lost event with 0x8000 | the retry number
 * in p2 (the first search sends 0, the stop sends the search turn in degrees),
 * so a retry is not read as a stop. */
static void line_note_retry (void)
{
    static uint8_t s_retry_seen;
    uint8_t const  retry = line_follower()->retry;

    if ((retry != s_retry_seen) && (0U != retry))
    {
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                       (int16_t)line_follower()->side,
                                       (uint16_t)(LINE_RETRY_EVENT | (uint16_t)retry),
                                       odometer_mm());
    }
    s_retry_seen = retry;
}

/* The wheels stand after a turn: the drive layer ramps up from rest. */
static void line_turn_ended (void)
{
    drive_init(&g_drive);
    g_pilot_last_ms = line_now_ms();
}

static bool line_pilot_measure (ir_sensors_state_t state, bool b_line_fresh, uint32_t now_ms,
                                motion_feedback_t * p_feedback, pilot_meas_t * p_meas);
static void line_pilot_note (pilot_cmd_t const * p_cmd);
static void line_step_with_profile (pilot_meas_t const * p_meas, pilot_cmd_t * p_cmd);

static bool line_sonar_fault (void)
{
    return 0U != (system_fault_get() & (FAULT_SONAR_NO_ECHO_BIT | FAULT_ECHO_LINE_BIT));
}

static void line_finish_turn (line_turn_kind_t turn, line_turn_state_t const * p_state,
                               line_turn_status_t status, uint32_t elapsed_ms, bool b_interrupted)
{
    g_turning = false;
    supervisor_set_turning(false);
    (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)((LINE_TURN_DONE == status) ? TELEMETRY_EVT_TURN_DONE
                                                                        : TELEMETRY_EVT_TURN_ABORT),
                                   (int16_t)turn, (uint16_t)elapsed_ms, odometer_mm());

    if (LINE_TURN_DONE == status)
    {
        line_follow_set_side(line_follower(), (LINE_TURN_KIND_UTURN == turn) ? LINE_SIDE_NONE : line_turn_side(p_state));
        line_update_fault(false);
    }
    else if (!b_interrupted)
    {
        line_follow_start_search(line_follower());
    }
    line_turn_ended();
}

static bool line_turn_odometry_valid (motion_feedback_t const * p_feedback)
{
    return p_feedback->fresh && p_feedback->odometry_valid[0] && p_feedback->odometry_valid[1];
}

/* The line task alone samples the motion feedback for the pending direction. */
static bool line_route_at_junction (line_turn_kind_t turn, ir_sensors_state_t sensors)
{
    motion_feedback_t feedback;
    if (E_OK != motion_feedback_snapshot(&feedback) || !line_turn_odometry_valid(&feedback) ||
        !feedback.speed_valid[0] || !feedback.speed_valid[1] ||
        (feedback.left_speed_um_s < 0) || (feedback.right_speed_um_s < 0) ||
        ((0 == feedback.left_speed_um_s) && (0 == feedback.right_speed_um_s)))
    {
        g_route = (line_route_t){0};
        return false;
    }
    bool const b_side_tape = ((uint8_t)sensors.barcode != BARCODE_LEVEL_SPACE);
    return line_route_observe(&g_route, turn, sensors.left, sensors.right, b_side_tape,
                              feedback.left_displacement_um, feedback.right_displacement_um);
}

STATIC_ASSERT(LINE_STRIPE_LEFT == (1U << TELEMETRY_JUNCTION_STRIPE_BIT_LEFT), "junction stripe left bit");
STATIC_ASSERT(LINE_STRIPE_RIGHT == (1U << TELEMETRY_JUNCTION_STRIPE_BIT_RIGHT), "junction stripe right bit");
STATIC_ASSERT(LINE_STRIPE_CORROBORATED == (1U << TELEMETRY_JUNCTION_STRIPE_BIT_CORROBORATED),
              "junction stripe corroborated bit");

static void line_report_stripe (void)
{
    uint8_t const stripe = line_follow_take_stripe(line_follower());
    if (0U != stripe)
    {
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_JUNCTION,
                                       (int16_t)TELEMETRY_JUNCTION_KIND_STRIPE, (uint16_t)stripe,
                                       odometer_mm());
    }
}

/* A probe that selects the branch reports TURN_START instead. */
static void line_report_probe (line_turn_kind_t turn, line_turn_status_t status)
{
    uint32_t outcome = TELEMETRY_JUNCTION_PROBE_ABORTED;
    if (LINE_TURN_NO_BRANCH == status)
    {
        outcome = TELEMETRY_JUNCTION_PROBE_NO_BRANCH;
    }
    else if (LINE_TURN_FALLBACK_DONE == status)
    {
        outcome = TELEMETRY_JUNCTION_PROBE_FALLBACK_EXIT;
    }
    else
    {
        /* Aborted. */
    }
    (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_JUNCTION,
                                   (int16_t)TELEMETRY_JUNCTION_KIND_PROBE,
                                   (uint16_t)(((uint32_t)turn * LINE_EVENT_HIGH_BYTE) + outcome),
                                   odometer_mm());
}

/* The follower can take an available exit with the requested direction pending. */
static void line_route_unselected (line_turn_status_t status)
{
    g_turning = false;
    supervisor_set_turning(false);
    if (LINE_TURN_FALLBACK_DONE == status)
    {
        line_follow_set_side(line_follower(), LINE_SIDE_NONE);
    }
    else
    {
        line_follow_reanchor(line_follower());
    }
    line_turn_ended();
}

typedef struct
{
    uint32_t start_ms;
    uint8_t  route_opcode;
    bool     started;
} line_turn_event_t;

STATIC_ASSERT(sizeof(line_turn_event_t) == 8U, "line_turn_event_t size");

static void line_report_turn_start (line_turn_event_t * p_event, line_turn_kind_t turn)
{
    p_event->start_ms = line_now_ms();
    p_event->started = true;
    (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_TURN_START, (int16_t)turn, 0U,
                                   odometer_mm());
}

/* A deferred route can be canceled by the supervisor while the sensors scan. */
static line_turn_status_t line_turn_targets (line_turn_state_t * p_state,
                                             motion_feedback_t const * p_feedback,
                                             ir_sensors_state_t sensors, line_targets_t * p_targets,
                                             line_turn_event_t * p_event)
{
    if (!line_turn_odometry_valid(p_feedback))
    {
        return LINE_TURN_ABORTED;
    }
    if (SUPERVISOR_NO_TURN == p_event->route_opcode)
    {
        return line_turn_update(p_state, p_feedback->left_displacement_um,
                                p_feedback->right_displacement_um, p_feedback->fresh,
                                sensors.left, sensors.right, p_targets);
    }
    if (!p_event->started && !supervisor_route_pending(p_event->route_opcode, false))
    {
        return LINE_TURN_ABORTED;
    }
    line_turn_status_t const status = line_route_probe(&g_route, p_feedback->left_displacement_um,
                                                       p_feedback->right_displacement_um,
                                                       sensors.left || sensors.right, p_targets);
    if (LINE_TURN_BRANCH_SELECTED != status)
    {
        return status;
    }
    if (!supervisor_route_pending(p_event->route_opcode, true))
    {
        return LINE_TURN_ABORTED;
    }
    line_report_turn_start(p_event, line_turn_of(p_event->route_opcode));
    return LINE_TURN_TURNING;
}

static bool line_turn_origin (line_turn_state_t * p_state, motion_feedback_t * p_feedback,
                              line_turn_kind_t turn, bool b_keep_approach)
{
    if ((E_OK != motion_feedback_snapshot(p_feedback)) || !line_turn_odometry_valid(p_feedback))
    {
        return false;
    }
    /* A U that interrupts a scan must return down the original approach tape. */
    int64_t const left_um = b_keep_approach ? p_state->start_left_um : p_feedback->left_displacement_um;
    int64_t const right_um = b_keep_approach ? p_state->start_right_um : p_feedback->right_displacement_um;
    line_turn_start(p_state, turn, left_um, right_um);
    return true;
}

static uint32_t line_turn_limit_ms (line_turn_event_t const * p_event)
{
    bool const b_returning = (SUPERVISOR_NO_TURN != p_event->route_opcode) && !p_event->started &&
                             line_route_returning(&g_route);
    return b_returning ? LINE_TURN_MAX_MS + LINE_FALLBACK_ALIGN_MS : LINE_TURN_MAX_MS;
}

static void line_begin_turn (line_turn_event_t * p_event, line_turn_kind_t turn)
{
    g_turning = true;
    supervisor_set_turning(true);
    if (SUPERVISOR_NO_TURN == p_event->route_opcode)
    {
        g_route = (line_route_t){0};
        line_report_turn_start(p_event, turn);
    }
}

/* Send the turn's wheel targets. The drive layer holds the last pilot
 * targets, so a pivot rises from the speed the car had. */
static int line_turn_drive (line_targets_t * p_targets, uint32_t * p_rise_ms)
{
    uint32_t const now_ms = line_now_ms();

    drive_rise(&g_drive, p_targets->left_um_s, p_targets->right_um_s, now_ms - *p_rise_ms,
               &p_targets->left_um_s, &p_targets->right_um_s);
    *p_rise_ms = now_ms;
    return motion_set_speed(p_targets->left_um_s, p_targets->right_um_s, LINE_LEASE_US);
}

/* One-shot pivot by encoder heading. Every pass reads the feedback
 * snapshot and the line sensors, then line_turn_update() decides the wheel
 * targets. A pilot safety or obstacle decision ends the turn before it can
 * renew the wheel targets. */
static bool line_execute_turn (line_turn_state_t * p_state, line_turn_kind_t turn,
                               uint8_t route_opcode, bool b_keep_approach)
{
    line_targets_t     targets = {0, 0};
    motion_feedback_t  feedback = {0};
    line_turn_event_t  event = {.route_opcode = route_opcode};
    uint32_t const     start_ms   = line_now_ms();
    bool               b_snapshot = false;
    bool               b_recovered = false;
    uint32_t           rise_ms    = start_ms;
    pilot_cmd_t        cmd = {0};
    uint8_t const      recovery_at_entry = g_recovery_seen;

    line_begin_turn(&event, turn);
    bool const b_origin_valid = line_turn_origin(p_state, &feedback, turn, b_keep_approach);
    line_turn_status_t status = b_origin_valid ? LINE_TURN_TURNING : LINE_TURN_ABORTED;
    while ((LINE_TURN_TURNING == status) && (line_now_ms() - start_ms < line_turn_limit_ms(&event)))
    {
        line_sample_t const sample = ir_get_line_sample();
        ir_sensors_state_t const sensors = sample.state;

        if (!line_sample_fresh(sample.timestamp_us))
        {
            status = LINE_TURN_ABORTED;
            break;
        }

        pilot_meas_t meas;

        b_snapshot = line_pilot_measure(sensors, true, line_now_ms(), &feedback, &meas);
        /* The line task can miss the entire hold between two turn updates. */
        b_recovered = recovery_at_entry != g_recovery_seen;
        if (b_recovered)
        {
            status = LINE_TURN_ABORTED;
            break;
        }
        meas.b_commanded_turn = true;
        line_step_with_profile(&meas, &cmd);
        if ((uint8_t)PILOT_BEHAVIOUR_NONE != cmd.behaviour)
        {
            status = LINE_TURN_ABORTED;
            break;
        }
        feedback.fresh = b_snapshot && feedback.fresh;
        status = line_turn_targets(p_state, &feedback, sensors, &targets, &event);
        if (LINE_TURN_TURNING == status)
        {
            if (E_OK != line_turn_drive(&targets, &rise_ms))
            {
                status = LINE_TURN_ABORTED;
            }
            else
            {
                /* The 5 ms line alarm can end the wait early, so measure the
                 * turn by the clock, not by counting waits. */
                (void)ir_wait_change(IR_TYPE_LINE, (TMO)LINE_POLL_MS);
            }
        }
        liveness_kick(LIVE_LINE);
    }
    if (LINE_TURN_TURNING == status)
    {
        status = LINE_TURN_ABORTED;
    }
    (void)motion_set_speed(0, 0, LINE_LEASE_US);
    if ((uint8_t)PILOT_BEHAVIOUR_NONE != cmd.behaviour)
    {
        line_pilot_note(&cmd);
    }
    if (SUPERVISOR_NO_TURN != route_opcode)
    {
        g_route = (line_route_t){.was_crossing = true};
        if (!event.started)
        {
            line_report_probe(turn, status);
            line_route_unselected(status);
            return b_origin_valid && !b_recovered && (LINE_TURN_ABORTED == status);
        }
    }
    line_finish_turn(turn, p_state, status, line_now_ms() - event.start_ms,
                     b_recovered || ((uint8_t)PILOT_BEHAVIOUR_NONE != cmd.behaviour));
    return false;
}

#define PILOT_SCAN_CLEARANCE_UNVERIFIED (4)

static void line_pilot_profile (look_t const * p_look, uint8_t sequence, int8_t probe_side,
                                uint8_t flags, int16_t origin_deci, int16_t endpoint_deci)
{
    telemetry_profile_t profile = {0};
    uint8_t             frame[TELEMETRY_FRAME_MAX];

    profile.req_id = sequence;
    profile.flags = flags;
    profile.status = (uint8_t)TELEMETRY_PROFILE_STATUS_PILOT_SPARSE;
    if (0 != probe_side)
    {
        profile.flags |= (uint8_t)(1U << TELEMETRY_PROFILE_FLAGS_BIT_PROBE_SIDE_VALID);
        profile.side = (probe_side > 0) ? (uint8_t)TELEMETRY_HUG_SIDE_LEFT : (uint8_t)TELEMETRY_HUG_SIDE_RIGHT;
    }
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        uint8_t const            echo  = look_ray_echo(p_ray);

        int32_t const bearing = look_ray_angle_deci(p_ray);
        uint8_t const fan_slot = obstacle_profile_record_ray(profile.slots, bearing - origin_deci,
                                                             p_ray->range_mm, echo);
        if ((LOOK_ECHO_VALID == echo) && (bearing == endpoint_deci))
        {
            profile.flags |= (uint8_t)(fan_slot << TELEMETRY_PROFILE_FLAGS_BIT_ENDPOINT_LSB);
        }
        if ((LOOK_ECHO_VALID == echo) && ((0U == profile.closest_mm) || (p_ray->range_mm < profile.closest_mm)))
        {
            profile.closest_mm = p_ray->range_mm;
        }
    }
    frame[0] = (uint8_t)TELEMETRY_TOPIC_PROFILE;
    (void)telemetry_pack_profile(&frame[1], &profile);
    (void)telemetry_queue_try_push(frame, (uint16_t)sizeof(frame));
}

static void line_step_with_profile (pilot_meas_t const * p_meas, pilot_cmd_t * p_cmd)
{
    bypass_t const * const p_bypass = &g_pilot.avoidance.hug.bypass;
    bool const b_front_fan = (p_bypass->phase == BYPASS_CONTACT) && (0U == p_bypass->sector);
    bool const b_side_fan = (p_bypass->phase == BYPASS_SIDE_FAN) && p_bypass->b_pending;
    int16_t const heading = p_bypass->requested_head;
    pilot_step(&g_pilot, p_meas, p_cmd);
    if (b_side_fan && p_bypass->phase != BYPASS_SIDE_FAN && g_pilot.avoidance.look.count > 0U)
    {
        int16_t const endpoint = (p_bypass->phase == BYPASS_LEVEL && g_pilot.avoidance.hug.breadth_mm > 0U)
                                 ? p_bypass->last_hit_deci : INT16_MAX;
        line_pilot_profile(&g_pilot.avoidance.look, p_bypass->seq, g_pilot.avoidance.hug.side,
                           (uint8_t)(1U << TELEMETRY_PROFILE_FLAGS_BIT_STATIONARY_CONTACT), heading, endpoint);
    }
    if (b_front_fan && ((p_bypass->phase != BYPASS_CONTACT) || (0U != p_bypass->sector)) &&
        (p_bypass->phase != BYPASS_IDLE) && (g_pilot.avoidance.look.count > 0U))
    {
        uint8_t const flags = (uint8_t)((1U << TELEMETRY_PROFILE_FLAGS_BIT_STATIONARY_CONTACT) |
            (p_bypass->b_wall ? (1U << TELEMETRY_PROFILE_FLAGS_BIT_WALL) : 0U));
        line_pilot_profile(&g_pilot.avoidance.look, p_bypass->seq, 0, flags, 0, INT16_MAX);
    }
}

/*!
 * @brief Publish a measured contact and record changes to the pilot decision.
 */
static void line_pilot_note (pilot_cmd_t const * p_cmd)
{
    if (0U != p_cmd->contact_mm)
    {
        LOG_INFO(LOG_TASK_LINE, LOG_MOD_NONE, "pilot: obstacle detected at %u mm", (unsigned)p_cmd->contact_mm);
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_OBSTACLE_SEEN,
                                       (int16_t)p_cmd->contact_mm, 0U, odometer_mm());
    }
    if (0U != p_cmd->course_scan_samples)
    {
        LOG_INFO(LOG_TASK_LINE, LOG_MOD_NONE, "pilot: stationary scan captured %u usable rays; course clearance unverified",
                 (unsigned)(p_cmd->course_scan_samples - 1U));
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_SCAN_DONE,
                                       PILOT_SCAN_CLEARANCE_UNVERIFIED, 0U, odometer_mm());
        line_pilot_profile(&g_pilot.avoidance.look, g_pilot.avoidance.course_scan_seq,
                           g_pilot.avoidance.course_side, 0U, 0, INT16_MAX);
    }
    /* The log still holds the reason of the stop this attempt clears. */
    uint8_t const attempt = pilot_log_recovery(&g_pilot_log, g_pilot.recoveries);
    if (0U != attempt)
    {
        LOG_INFO(LOG_TASK_LINE, LOG_MOD_NONE, "pilot: recovery %u of %u from reason %u", (unsigned)attempt,
                 (unsigned)PILOT_RECOVERIES_MAX, (unsigned)g_pilot_log.reason);
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_RECOVERY,
                                       (int16_t)g_pilot_log.reason,
                                       (uint16_t)((PILOT_RECOVERIES_MAX * LINE_EVENT_HIGH_BYTE) + attempt),
                                       odometer_mm());
    }
    if (pilot_log_note(&g_pilot_log, p_cmd))
    {
        if ((uint8_t)PILOT_REASON_COURSE_RECOVERY == p_cmd->reason)
        {
            LOG_INFO(LOG_TASK_LINE, LOG_MOD_NONE, "pilot: checking course clearance");
        }
        else
        {
            LOG_INFO(LOG_TASK_LINE, LOG_MOD_NONE, "pilot: behaviour %d reason %d speed %d turn %d",
                     (int)p_cmd->behaviour, (int)p_cmd->reason, (int)p_cmd->speed_mm_s,
                     (int)p_cmd->turn_deci_s);
        }
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_PILOT_DECISION,
                                       p_cmd->speed_mm_s,
                                       (uint16_t)((p_cmd->behaviour * LINE_EVENT_HIGH_BYTE) +
                                                  p_cmd->reason),
                                       odometer_mm());
    }
    if (!hug_active(&g_pilot.avoidance.hug))
    {
        g_hug_leg_reported = false;
    }
    else if (!g_hug_leg_reported && (p_cmd->speed_mm_s > 0) &&
             ((uint8_t)PILOT_BEHAVIOUR_HUG == p_cmd->behaviour) &&
             (TELEMETRY_HUG_STATE_LEG == hug_word_state(&g_pilot.avoidance.hug)))
    {
        /* This event means the first forward side leg was commanded.
         * The wheel frames establish whether the car actually travelled. */
        int16_t const side = (g_pilot.avoidance.hug.side > 0) ? (int16_t)TELEMETRY_HUG_SIDE_LEFT :
                                                      (int16_t)TELEMETRY_HUG_SIDE_RIGHT;
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_BYPASS_START, side, 0U,
                                       odometer_mm());
        g_hug_leg_reported = true;
    }
    if (p_cmd->b_rejoined)
    {
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_BYPASS_END,
                                       (int16_t)g_pilot.avoidance.exit_width_mm,
                                       g_pilot.avoidance.exit_breadth_mm, odometer_mm());
    }
    if (p_cmd->b_rejoin_check)
    {
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_REJOIN, p_cmd->rejoin_head_deci,
                                       (uint16_t)p_cmd->rejoin_y_mm, odometer_mm());
    }
}

/*!
 * @brief Fill the ping fields of the measurements from the latest ping word
 *        and the sonar fault bits.
 */
static void line_pilot_ping_meas (pilot_meas_t * p_meas)
{
    uint32_t const word = obstacle_ping_read();

    p_meas->ping_mm        = obstacle_pack_ping_mm(word);
    p_meas->ping_servo_deci = (int16_t)obstacle_pack_ping_servo(word);
    p_meas->ping_echo      = obstacle_pack_ping_echo(word);
    p_meas->ping_seq       = obstacle_pack_ping_seq(word);
    p_meas->b_ping_reply   = obstacle_pack_ping_is_reply(word);
    p_meas->b_sonar_fault  = line_sonar_fault();
}

/*!
 * @brief Send the ping request of the command, once. The same request every
 *        tick would take one ping each tick.
 */
static void line_pilot_ping_request (pilot_cmd_t const * p_cmd)
{
    uint32_t const req = obstacle_pack_ping_req(p_cmd->ping_servo_deci, p_cmd->ping_seq);

    if (0U == p_cmd->b_ping_req)
    {
        /* The request is over; the same word may be wanted again (a new hug
         * counts its sequence from the start). */
        g_pilot_ping_sent = 0U;
    }
    else if (req != g_pilot_ping_sent)
    {
        ER const result = obstacle_ping_request(p_cmd->ping_servo_deci, p_cmd->ping_seq);

        if (E_OK == result)
        {
            g_pilot_ping_sent   = req;
            g_pilot_ping_warned = false;
        }
        else if (!g_pilot_ping_warned)
        {
            g_pilot_ping_warned = true;
            LOG_WARN(LOG_TASK_LINE, LOG_MOD_NONE, "pilot: ping request refused %d", (int)result);
        }
        else
        {
            /* Warned already; the request is tried again next tick. */
        }
    }
}

/*!
 * @brief Publish the hug snapshot for the telemetry task.
 */
static void line_pilot_publish_hug (int16_t servo_deci)
{
    uint32_t words[HUG_WORD_COUNT];
    UINT     saved = 0U;
    uint32_t idx   = 0U;

    hug_word_pack(words, &g_pilot.avoidance.hug, servo_deci, hug_heading_deci(&g_pilot.avoidance.hug));
    /* The count moves only when the hug content does, so a changed value
     * marks a new sample. g_hug_words is written by this task only. */
    if ((0U == ((words[0] ^ g_hug_words[0]) & ~(HUG_WORD_SEQ_MK << HUG_WORD_SEQ_SH))) &&
        (words[1] == g_hug_words[1]))
    {
        return;
    }
    g_hug_seq++;
    words[0] |= hug_word_put(g_hug_seq, HUG_WORD_SEQ_SH, HUG_WORD_SEQ_MK);
    (void)ISpinLock(&g_hug_lock, &saved);
    for (idx = 0U; idx < HUG_WORD_COUNT; idx++)
    {
        g_hug_words[idx] = words[idx];
    }
    (void)ISpinUnlock(&g_hug_lock, saved);
}

/*!
 * @brief Line events and the line-lost fault, as the old path raised them.
 *        Only a follower that has the wheels counts; a hug's detour is not a
 *        lost line.
 */
static void line_pilot_watch (pilot_cmd_t const * p_cmd, int64_t left_um, int64_t right_um)
{
    bool const    b_follow = ((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_cmd->behaviour);
    uint8_t const bits     = line_watch_step(&g_pilot_watch, b_follow,
                                             ((uint8_t)PILOT_BEHAVIOUR_HUG == p_cmd->behaviour),
                                             (line_follow_status_t)g_pilot.follow.status);

    g_line_travel_um = (left_um + right_um) / LINE_WHEEL_COUNT;
    if (0U != (bits & LINE_WATCH_QUIET))
    {
        system_fault_assign(FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, 0U);
        g_line_lost = false;
    }
    if (b_follow)
    {
        line_update_fault(0U != g_pilot_watch.b_lost);
        line_note_retry();
    }
    if (0U != (bits & LINE_WATCH_STOP))
    {
        /* A second line_lost event, with the search turn in p2, marks the stop. */
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                       (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                       (int16_t)g_pilot.follow.side,
                                       (uint16_t)(LINE_SEARCH_SWEEP_DECI / DECI_PER_DEG),
                                       odometer_mm());
    }
}

static bool line_pilot_measure (ir_sensors_state_t state, bool b_line_fresh, uint32_t now_ms,
                                motion_feedback_t * p_feedback, pilot_meas_t * p_meas)
{
    uint8_t const resets = supervisor_reset_count();
    bool b_continuous_floor = false;
    uint8_t const starts = supervisor_start_snapshot(&b_continuous_floor);
    static int64_t    s_left_um;
    static int64_t    s_right_um;
    static int64_t    s_previous_left;
    static int64_t    s_previous_right;
    static uint8_t    s_invalid[MOTION_WHEELS];
    static bool       s_contiguous;
    motion_status_t   motion;
    bool const        b_motion   = (E_OK == motion_snapshot(&motion, p_feedback, NULL));
    /* A motion fault that clears in place refuses commands like a hold. */
    bool const        b_held     = !b_motion || (0U != motion.hold_mask) ||
                                   ((uint8_t)MOTION_STATE_FAULT == motion.state);
    bool const        b_feedback_fresh = b_motion && line_turn_odometry_valid(p_feedback);

    if (b_motion && motion.stop_requested)
    {
        pilot_estop(&g_pilot);
    }
    if (pilot_count_moved(&g_pilot_reset_seen, resets))
    {
        /* RESET: the ENCODER and HUG_TIMEOUT latches end; the ESTOP stays. The
         * supervisor holds the wheels until START. */
        pilot_clear_latches(&g_pilot);
        pilot_change_floor(&g_pilot, b_continuous_floor);
        drive_init(&g_drive);
        g_route = (line_route_t){0};
        g_hug_leg_reported = false;
        g_pilot_ping_sent = 0U;
    }
    bool const b_new_start = pilot_count_moved(&g_pilot_start_seen, starts);
    if ((g_continuous_floor != b_continuous_floor) || (b_new_start && b_continuous_floor))
    {
        pilot_change_floor(&g_pilot, b_continuous_floor);
        g_hug_leg_reported = false;
        drive_init(&g_drive);
        g_pilot_ping_sent = 0U;
        g_continuous_floor = b_continuous_floor;
    }
    if (b_new_start)
    {
        pilot_start_run(&g_pilot);
        g_pilot_ping_sent = 0U;
    }
    bool clear = false;
    if (pilot_count_moved(&g_recovery_seen, supervisor_recovery_snapshot(&clear)) && !b_new_start)
    {
        pilot_recover_pose(&g_pilot, clear, now_ms);
        drive_init(&g_drive);
        g_route = (line_route_t){0};
        g_pilot_ping_sent = 0U;
        g_hug_leg_reported = false;
    }

    if (b_feedback_fresh)
    {
        /* Absolute counts include transitions across invalid intervals. Restart
         * the delta at the first good pair, including gaps between polls. */
        if (s_contiguous && s_invalid[0] == p_feedback->invalid_samples[0] &&
            s_invalid[1] == p_feedback->invalid_samples[1])
        {
            s_left_um += p_feedback->left_displacement_um - s_previous_left;
            s_right_um += p_feedback->right_displacement_um - s_previous_right;
        }
        s_previous_left = p_feedback->left_displacement_um;
        s_previous_right = p_feedback->right_displacement_um;
        s_invalid[0] = p_feedback->invalid_samples[0];
        s_invalid[1] = p_feedback->invalid_samples[1];
        g_pilot_stamp_ms = now_ms - (p_feedback->age_us / LINE_US_PER_MS);
    }

    s_contiguous = b_feedback_fresh;
    *p_meas = (pilot_meas_t){0};
    p_meas->left_um       = s_left_um;
    p_meas->right_um      = s_right_um;
    p_meas->now_ms        = now_ms;
    p_meas->stamp_ms      = g_pilot_stamp_ms;
    p_meas->b_wheel_invalid = !b_feedback_fresh;
    p_meas->b_line_left   = state.left;
    p_meas->b_line_right  = state.right;
    p_meas->b_held        = b_held || !b_line_fresh;
    p_meas->b_barcode     = ir_barcode_recent();
    p_meas->b_marker_progress = !g_turning && line_barcode_advanced(&p_meas->marker_prefix_mm);
    p_meas->b_bounded_course = !b_continuous_floor;
    if (b_feedback_fresh && p_feedback->speed_valid[0] && p_feedback->speed_valid[1])
    {
        p_meas->b_stationary = (0 == p_feedback->left_speed_um_s) && (0 == p_feedback->right_speed_um_s);
        p_meas->wheel_radius_mm = drive_forward_radius_mm(p_feedback->left_speed_um_s, p_feedback->right_speed_um_s);
    }
    line_pilot_ping_meas(p_meas);
    return b_feedback_fresh;
}

/*!
 * @brief One pass of the pilot: measure, decide, drive.
 */
static void line_pilot_step (ir_sensors_state_t state, bool b_line_fresh, uint8_t route_opcode)
{
    pilot_meas_t      meas;
    pilot_cmd_t       cmd = {0};
    motion_feedback_t feedback;
    uint32_t const    now_ms = line_now_ms();
    uint32_t const    dt_ms = now_ms - g_pilot_last_ms;
    int32_t           left_um_s = 0;
    int32_t           right_um_s = 0;
    int16_t           roll_cdeg = 0;
    bool const        b_feedback_fresh = line_pilot_measure(state, b_line_fresh, now_ms, &feedback, &meas);

    bool const        b_hump = line_on_hump(&roll_cdeg, &meas);

    line_follow_set_hump(&g_pilot.follow, b_hump);
    /* A climb also loads the wheels: only level ground tells a push. */
    meas.b_loaded = !b_hump && supervisor_wheels_loaded();
    line_step_with_profile(&meas, &cmd);
    if (((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE != cmd.behaviour) && (SUPERVISOR_NO_TURN != route_opcode))
    {
        /* The approach evidence belongs to this location. The choice remains
         * in the supervisor until the requested branch has been selected. */
        g_route = (line_route_t){0};
    }
    line_pilot_watch(&cmd, meas.left_um, meas.right_um);
    line_pilot_ping_request(&cmd);
    line_pilot_publish_hug(meas.ping_servo_deci);

    pilot_drive(&g_drive, &cmd, dt_ms, &left_um_s, &right_um_s);
    if (b_feedback_fresh && feedback.speed_valid[0] && feedback.speed_valid[1] &&
        ((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour) &&
        ((uint8_t)PILOT_REASON_NONE == cmd.reason) &&
        (cmd.speed_mm_s > 0) && (0 == cmd.turn_deci_s) &&
        ((uint8_t)LINE_FOLLOW_FOLLOWING == g_pilot.follow.status) &&
        !meas.b_barcode && !hug_reapproaching(&g_pilot.avoidance.hug))
    {
        drive_compensate_roll(&g_drive, roll_cdeg, feedback.left_speed_um_s,
                              feedback.right_speed_um_s);
        left_um_s  = g_drive.left_um_s;
        right_um_s = g_drive.right_um_s;
    }
    g_pilot_last_ms = now_ms;
    if (cmd.b_scan_pivot)
    {
        (void)motion_set_scan_pivot(left_um_s, right_um_s, LINE_LEASE_US);
    }
    else
    {
        (void)motion_set_speed(left_um_s, right_um_s, LINE_LEASE_US);
    }
    line_pilot_note(&cmd);
}

/* Keep the line task as the owner of wheel commands while verifying START. */
static bool line_start_clearance (uint8_t * p_seq, uint32_t * p_since_us,
                                  bool * p_clear, uint32_t now_us)
{
    uint32_t const ping = obstacle_ping_read();
    if (line_sonar_fault() || obstacle_pack_ping_is_reply(ping) || obstacle_pack_ping_servo(ping) != 0)
    {
        *p_clear = false;
        return false;
    }
    uint8_t const seq = obstacle_pack_ping_seq(ping);
    if (seq != *p_seq)
    {
        uint8_t const echo = obstacle_pack_ping_echo(ping);
        *p_seq = seq;
        *p_since_us = now_us;
        *p_clear = echo == LOOK_ECHO_OPEN ||
                   (echo == LOOK_ECHO_VALID && obstacle_pack_ping_mm(ping) >= HUG_STANDOFF_MM);
    }
    return *p_clear && now_us - *p_since_us <= PILOT_FORWARD_PING_STALE_MS * LINE_US_PER_MS;
}

static bool line_start_check (bool b_line_fresh)
{
    static start_check_t s_check;
    static uint32_t s_ping_us;
    static uint8_t s_ping_seq;
    static uint8_t s_generation;
    static bool s_armed;
    static bool s_clear;
    bool armed = false;
    uint8_t const generation = supervisor_armed_start(&armed);
    if (!armed)
    {
        s_armed = false;
        return false;
    }
    if (!s_armed || s_generation != generation)
    {
        start_check_begin(&s_check, now_us());
        s_ping_seq = obstacle_pack_ping_seq(obstacle_ping_read());
        s_clear = false;
        s_generation = generation;
        s_armed = true;
    }
    if (s_check.phase == START_CHECK_PASSED)
    {
        return false;
    }
    if (s_check.phase == START_CHECK_FAILED)
    {
        (void)motion_set_speed(0, 0, LINE_LEASE_US);
        return true;
    }

    motion_status_t motion;
    motion_feedback_t wheels = {0};
    physics_status_t physics = {0};
    bool const motion_valid = (E_OK == motion_snapshot(&motion, &wheels, NULL)) &&
        motion.ready && !motion.stop_requested && motion.hold_mask == 0U && motion.fault == 0U &&
        line_turn_odometry_valid(&wheels) && wheels.speed_valid[0] && wheels.speed_valid[1];
    bool const imu_valid = (E_OK == physics_snapshot(&physics)) && physics.ready &&
        physics.calibrated && physics.attitude_sample_valid && !physics.attitude_unknown &&
        physics.attitude_state == TELEMETRY_ATTITUDE_STATE_OK;
    uint32_t const sample_us = now_us();
    bool const clear = line_start_clearance(&s_ping_seq, &s_ping_us, &s_clear, sample_us);
    bool const ready = b_line_fresh && clear && motion_valid && imu_valid &&
        sample_us - physics.forward_sample_us <= START_CHECK_IMU_AGE_US;
    start_check_phase_t const phase = start_check_step(&s_check, sample_us, ready, &wheels,
                                                      physics.forward_sample_us, physics.forward_specific_mg);
    int32_t const speed = phase == START_CHECK_PROBING ? START_CHECK_SPEED_UM_S : 0;
    g_pilot_last_ms = line_now_ms();
    (void)motion_set_speed(speed, speed, LINE_LEASE_US);
    if (phase == START_CHECK_FAILED)
    {
        supervisor_start_check_failed(generation);
    }
    return true;
}

void line_task_stats (line_task_stats_t * p_stats)
{
    p_stats->lost_count = g_line_lost_count;
    p_stats->b_turning  = g_turning;
}

void line_task_hug (uint32_t * p_words)
{
    UINT     saved = 0U;
    uint32_t idx   = 0U;

    (void)ISpinLock(&g_hug_lock, &saved);
    for (idx = 0U; idx < HUG_WORD_COUNT; idx++)
    {
        p_words[idx] = g_hug_words[idx];
    }
    (void)ISpinUnlock(&g_hug_lock, saved);
}

void line_task_run (void)
{
    line_turn_state_t turn_state = {0};
    bool b_interrupted_probe = false;
    /* Set up before the loop: a turn that ends first sets the side on the
     * follower, and a later init would wipe it. */
    pilot_init(&g_pilot);
    drive_init(&g_drive);
    pilot_log_init(&g_pilot_log);
    g_pilot_stamp_ms = line_now_ms();
    g_pilot_reset_seen = supervisor_reset_count();
    g_pilot_start_seen = supervisor_start_snapshot(&g_continuous_floor);
    for (;;)
    {
        line_sample_t const sample       = ir_get_line_sample();
        bool const          b_line_fresh = line_sample_fresh(sample.timestamp_us);
        liveness_kick(LIVE_LINE);
        if (line_start_check(b_line_fresh))
        {
            ir_wait_change(IR_TYPE_LINE, LINE_POLL_MS);
            continue;
        }
        uint8_t             source       = 0U;
        uint8_t const       taken        = supervisor_poll_turn(&source);
        uint8_t const       route_opcode = ((VEHICLE_SRC_BARCODE == source) && vehicle_is_junction_choice(taken)) ?
                                           taken : SUPERVISOR_NO_TURN;
        bool const b_keep_approach = b_interrupted_probe && (VEHICLE_SRC_BARCODE == source) &&
                                     (VEHICLE_CMD_UTURN == taken);
        b_interrupted_probe = false;

        line_follow_set_route_candidate(line_follower(),
                                        (SUPERVISOR_NO_TURN != route_opcode) && line_route_has_candidate(&g_route));
        if (SUPERVISOR_NO_TURN != taken)
        {
            bool const b_available = pilot_log_is_following(&g_pilot_log);
            if (!b_line_fresh || !b_available)
            {
                g_route = (line_route_t){0};
                if (SUPERVISOR_NO_TURN == route_opcode)
                {
                    supervisor_turn_refused(taken);
                }
            }
            else if ((SUPERVISOR_NO_TURN == route_opcode) || line_route_at_junction(line_turn_of(taken), sample.state))
            {
                b_interrupted_probe = line_execute_turn(&turn_state, line_turn_of(taken), route_opcode, b_keep_approach);
                continue;
            }
        }
        else
        {
            g_route = (line_route_t){0};
        }

        line_pilot_step(sample.state, b_line_fresh, route_opcode);
        line_report_stripe();
        ir_wait_change(IR_TYPE_LINE, LINE_POLL_MS);
    }
}

