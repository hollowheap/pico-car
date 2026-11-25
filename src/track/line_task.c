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
#include "navigation/obstacle_flags.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/units.h"
#include "physics/physics.h"
#if PICO_CAR_PILOT
#include "pilot/drive.h"
#include "pilot/pilot.h"
#include "pilot/pilot_log.h"
#include "navigation/obstacle_ping.h"
#endif
#include "supervisor/command.h"
#include "supervisor/supervisor.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/hug_word.h"
#include "telemetry/wire_schema.h"
#include "track/line_control.h"
#include "track/line_task.h"
#include "track/track.h"

/* Line following. The cruise speed, the search limits and the turn speeds
 * live in track/line_control.h (tune them there on the car). The duty cap in
 * motion_setup() must leave headroom for them. The follower steers by a
 * heading target and stops only when the search finds nothing; one-shot turns
 * pivot in place.
 * LINE_POLL_MS is the control period. */
#define LINE_POLL_MS             (10U)
/* Upper bound on one turn, so a blocked car cannot pivot for ever. Tune on
 * the car: it must exceed the longest U-turn, including the slow phase. */
#define LINE_TURN_MAX_MS         (4000U)

/* The distance a barcode turn waits after the barcode, so the pivot happens at the
 * junction and not at the first stripe. Tune on the car. 0 turns at once. */
#define BARCODE_TURN_DELAY_MM        (0U)

/* A turn the supervisor allowed and the line task has not run yet. */
static uint8_t          g_turn_wait       = (uint8_t)LINE_TURN_KIND_NONE; /* line_turn_kind_t */
static uint32_t         g_turn_wait_at_mm = 0U;

/* Wheel displacement units. */
#define DECI_PER_DEG (10)

/* line_lost p2 flag of a retry; the low bits are the retry number. */
#define LINE_RETRY_EVENT (0x8000U)

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
#define LINE_LEASE_PASSES        (10U)
#define LINE_US_PER_MS           (1000U)
#define LINE_LEASE_US \
    (LINE_LEASE_PASSES * LINE_POLL_MS * LINE_US_PER_MS)

static bool          g_line_lost;
static int64_t       g_line_travel_um;
#if !PICO_CAR_PILOT
static line_follow_t g_line_follow;
static int32_t       g_line_heading_deci;
#endif

#if PICO_CAR_PILOT
/* Pilot, drive layer and the last decision. Only the line task touches them. */
static pilot_state_t g_pilot;
static drive_state_t g_drive;
static pilot_log_t   g_pilot_log;
static uint32_t      g_pilot_last_ms;
static uint32_t      g_pilot_stamp_ms;
static bool          g_pilot_turn_ended;
static line_watch_t  g_pilot_watch;       /* line-lost bookkeeping */
static uint32_t      g_pilot_ping_sent;   /* last ping request sent, 0 for none */
static bool          g_pilot_ping_warned;
static uint8_t       g_pilot_reset_seen;  /* reset count last acted on */
static uint8_t       g_pilot_start_seen;  /* start count last acted on */

/* The sonar faults that stop the pilot: a stuck echo line, no echo at all, or
 * an echo line that is too noisy to read. */
#define LINE_SONAR_FAULTS \
    (OBSTACLE_FAULT_ECHO_STUCK | OBSTACLE_FAULT_NO_ECHO | OBSTACLE_FAULT_ECHO_NOISE)
#endif

/* The hug snapshot for the obstacle frame. The line task writes both words
 * together; the telemetry task reads both, so one frame never mixes two
 * ticks. The lock needs no set-up call: a spinlock is free at zero, and a
 * static starts at zero before any task runs. */
static T_SPLOCK      g_hug_lock;
static uint32_t      g_hug_words[HUG_WORD_COUNT];
static uint32_t      g_hug_seq; /* line task only; the field keeps the low 2 bits */

/* Read by the telemetry task (diag packet): line-lost entries since boot, and
 * whether a one-shot turn is running. Written by the line task only. */
static volatile uint16_t g_line_lost_count;
static volatile uint16_t g_line_search_mm; /* longest stretch driven while lost */
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

/* The follower a turn hands its side to: the pilot's own with the pilot on. */
static line_follow_t * line_follower (void)
{
#if PICO_CAR_PILOT
    return &g_pilot.follow;
#else
    return &g_line_follow;
#endif
}

/* Mean wheel travel at the start of the present line loss. */
static int64_t g_line_lost_at_um;

/* Set FAULT_LINE_LOST_BIT on entering line lost, clear it on leaving. The
 * events carry the remembered side and, on leaving, the distance driven while
 * lost. */
static void line_update_fault (bool b_lost)
{
    int64_t  const driven_um = g_line_travel_um - g_line_lost_at_um;
    uint32_t const driven_mm = (uint32_t)(((driven_um < 0) ? -driven_um : driven_um) / UM_PER_MM);
    uint16_t const driven_u16 = (driven_mm > UINT16_MAX) ? UINT16_MAX : (uint16_t)driven_mm;

    if (b_lost && !g_line_lost)
    {
        system_fault_assign(FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, FAULT_LINE_LOST_BIT);
        g_line_lost_count++;
        g_line_lost_at_um = g_line_travel_um;
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                   (int16_t)line_follower()->side, 0U, odometer_mm());
    }
    else if (!b_lost && g_line_lost)
    {
        system_fault_assign(FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, 0U);
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_FOUND,
                                   (int16_t)line_follower()->side, driven_u16,
                                   odometer_mm());
    }
    else
    {
        /* No change. */
    }
    if (b_lost && (driven_u16 > g_line_search_mm))
    {
        g_line_search_mm = driven_u16;
    }
    g_line_lost = b_lost;
}

/* The hump signal of the supervisor: the sensors lift off the floor at the
 * crest, so the follower bridges a longer gap. */
static bool line_on_hump (void)
{
    physics_status_t physics;
    bool             b_hump = false;

    if ((E_OK == physics_snapshot(&physics)) && physics.ready)
    {
        b_hump = physics.hump_active || (TELEMETRY_HUMP_STATE_FLAT != physics.state);
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
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                   (int16_t)line_follower()->side,
                                   (uint16_t)(LINE_RETRY_EVENT | (uint16_t)retry),
                                   odometer_mm());
    }
    s_retry_seen = retry;
}

#if !PICO_CAR_PILOT
/* Forget the search state, e.g. when the obstacle step took the motors. */
static void line_follow_reset (void)
{
    line_follow_init(&g_line_follow);
    line_update_fault(false);
}
#endif

/* The wheels stand after a turn: the drive layer ramps up from rest. */
static void line_turn_ended (void)
{
#if PICO_CAR_PILOT
    drive_init(&g_drive);
    /* The pilot did not run during the turn, so its stuck window is stale. */
    g_pilot_turn_ended = true;
#endif
}

/* True while the sonar reports a fault that stops the pilot. The old path
 * has no such stop: its obstacle step runs on every pass of a turn. */
static bool line_sonar_fault (void)
{
#if PICO_CAR_PILOT
    return 0U != ((uint32_t)obstacle_get_faults() & LINE_SONAR_FAULTS);
#else
    return false;
#endif
}

/* One-shot pivot by encoder heading. Every pass reads the feedback
 * snapshot and the line sensors, then line_turn_update() decides the wheel
 * targets. The turn ends on line capture, an abort, LINE_TURN_MAX_MS, or a
 * sonar fault: the pilot does not run during a turn, and after it the pilot
 * holds the car with SONAR. */
static void line_execute_turn (line_turn_kind_t turn)
{
    line_turn_state_t  state;
    line_targets_t     targets = {0, 0};
    motion_feedback_t  feedback;
    line_turn_status_t status = LINE_TURN_ABORTED;
    uint32_t           elapsed_ms = 0U;
    uint32_t const     start_ms   = line_now_ms();
    bool               b_snapshot = false;
#if !PICO_CAR_PILOT
    bool               b_obstacle = false;
#endif

    if (LINE_TURN_KIND_NONE == turn)
    {
        return;
    }

    g_turning  = true;
    supervisor_set_turning(true);
    (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                               (uint8_t)TELEMETRY_EVT_TURN_START, (int16_t)turn, 0U,
                               odometer_mm());
    b_snapshot = (E_OK == motion_feedback_snapshot(&feedback));
    if (b_snapshot)
    {
        line_turn_start(&state, turn, feedback.left_displacement_um,
                        feedback.right_displacement_um);
        status = LINE_TURN_TURNING;
    }
    while ((LINE_TURN_TURNING == status) && (elapsed_ms < LINE_TURN_MAX_MS) && !line_sonar_fault())
    {
        ir_sensors_state_t const sensors = ir_get_state();

#if !PICO_CAR_PILOT
        /* The obstacle step runs on every pass of a turn: when it takes the
         * motors, the turn ends and the obstacle task drives. */
        if (obstacle_line_step(sensors.left, sensors.right))
        {
            b_obstacle = true;
            status     = LINE_TURN_ABORTED;
            break;
        }
#endif

        b_snapshot = (E_OK == motion_feedback_snapshot(&feedback));
        status = line_turn_update(&state, feedback.left_displacement_um,
                                feedback.right_displacement_um,
                                b_snapshot && feedback.fresh, sensors.left,
                                sensors.right, &targets);
        if (LINE_TURN_TURNING == status)
        {
            if (E_OK != motion_set_speed(targets.left_um_s, targets.right_um_s,
                                         LINE_LEASE_US))
            {
                status = LINE_TURN_ABORTED;
            }
            else
            {
                /* The 5 ms line alarm can end the wait early, so measure the
                 * turn by the clock, not by counting waits. */
                (void)ir_wait_change(IR_TYPE_LINE, (TMO)LINE_POLL_MS);
                elapsed_ms = line_now_ms() - start_ms;
            }
        }
        /* A turn lasts longer than the liveness miss limit (0.5 s). */
        liveness_kick(LIVE_LINE);
    }
#if !PICO_CAR_PILOT
    if (!b_obstacle)
#endif
    {
        (void)motion_set_speed(0, 0, LINE_LEASE_US);
    }
    g_turning = false;
    supervisor_set_turning(false);
    (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                               (uint8_t)((LINE_TURN_DONE == status) ? TELEMETRY_EVT_TURN_DONE
                                                                    : TELEMETRY_EVT_TURN_ABORT),
                               (int16_t)turn, (uint16_t)elapsed_ms, odometer_mm());

#if !PICO_CAR_PILOT
    if (b_obstacle)
    {
        /* The obstacle task has the motors; forget the search state. */
        line_follow_reset();
    }
    else
#endif
    if (LINE_TURN_DONE == status)
    {
        line_follow_set_side(line_follower(), line_turn_side(&state));
        line_update_fault(false);
    }
    else
    {
        /* Stopped above. The car looks for the line on the spot. The search
         * sets the line-lost fault on the next pass. */
        line_follow_start_search(line_follower());
    }
    line_turn_ended();
}

#if !PICO_CAR_PILOT
/*!
 * @brief Speed cap of the sonar bumper: the car slows as an obstacle ahead
 *        comes closer, so it reaches the obstacle stop distance slow.
 */
static int32_t line_bumper_cap (void)
{
    obstacle_status_t obstacle;
    int32_t           cap = LINE_CRUISE_UM_S;

    if (E_OK == obstacle_snapshot(&obstacle))
    {
        bool const b_ranged =
            ((uint8_t)TELEMETRY_ECHO_CLASS_VALID == obstacle.echo_class) ||
            ((uint8_t)TELEMETRY_ECHO_CLASS_TOO_CLOSE == obstacle.echo_class);

        cap = line_bumper_cap_um_s(b_ranged, obstacle.distance_mm);
    }

    return cap;
}

/*!
 * @brief One pass of the line follower. A sensor hit slews a heading
 *        target toward the line and the encoder heading follows it. With
 *        neither sensor the car holds its heading for LINE_GAP_MM (on a
 *        hump LINE_HUMP_GAP_MM), then turns on the spot to find the line. A
 *        failed search is retried from a reverse; when the last retry finds
 *        nothing it stops and waits for a sensor hit.
 */
static void line_follow_step (ir_sensors_state_t state)
{
    line_targets_t       targets = {0, 0};
    motion_feedback_t    feedback;
    line_follow_status_t status = LINE_FOLLOW_FOLLOWING;
    bool const           b_was_stopped =
        ((uint8_t)LINE_FOLLOW_STOPPED == g_line_follow.status);
    bool const           b_straight =
        ((uint8_t)LINE_FOLLOW_FOLLOWING == g_line_follow.status) ||
        ((uint8_t)LINE_FOLLOW_BRIDGING == g_line_follow.status);

    /* A stopped car stays stopped: a stray barcode edge must not drive it. A
     * searching car does not drive on either: the straight would run over its
     * reverse. */
    if (ir_barcode_recent() && b_straight)
    {
        /* Barcode stripes under the line sensors look like a lost line. For 250 ms
         * after any barcode edge the car drives straight and the search state does
         * not change. */
        targets.left_um_s  = LINE_CRUISE_UM_S;
        targets.right_um_s = LINE_CRUISE_UM_S;
        line_follow_reanchor(&g_line_follow);
        line_targets_cap(&targets, line_bumper_cap());
        (void)motion_set_speed(targets.left_um_s, targets.right_um_s, LINE_LEASE_US);
        return;
    }

    if (E_OK == motion_feedback_snapshot(&feedback))
    {
        g_line_travel_um = (feedback.left_displacement_um +
                            feedback.right_displacement_um) /
                           LINE_WHEEL_COUNT;
        g_line_heading_deci = line_heading_deci(feedback.left_displacement_um,
                                                feedback.right_displacement_um);
    }

    line_follow_set_hump(&g_line_follow, line_on_hump());
    status = line_follow_update(&g_line_follow, state.left, state.right,
                                g_line_heading_deci, g_line_travel_um,
                                line_now_ms(), &targets);
    /* Line lost means the gap is over; the car turns on the spot, then stops
     * when both turns find nothing. A sensor hit ends it. */
    line_update_fault((LINE_FOLLOW_SEARCHING == status) ||
                      (LINE_FOLLOW_STOPPED == status));
    line_note_retry();
    if ((LINE_FOLLOW_STOPPED == status) && !b_was_stopped)
    {
        /* A second line_lost event, with the search turn in p2, marks the stop. */
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                   (int16_t)g_line_follow.side,
                                   (uint16_t)(LINE_SEARCH_SWEEP_DECI / DECI_PER_DEG), odometer_mm());
    }
    line_targets_cap(&targets, line_bumper_cap());
    (void)motion_set_speed(targets.left_um_s, targets.right_um_s,
                           LINE_LEASE_US);
}
#endif

#if PICO_CAR_PILOT
/* The decision event packs behaviour << 8 | reason. */
#define PILOT_EVENT_BEHAVIOUR_MUL (256U)

/*!
 * @brief Tell the pilot about the operator emergency stop. The motion service
 *        latches the stop until reboot (no call clears it), so the pilot's
 *        ESTOP holds until reboot too.
 * @return true while motion holds the wheels.
 */
static bool line_pilot_stop_state (void)
{
    motion_status_t status = {0};

    if ((0 == motion_status(&status)) && status.stop_requested)
    {
        pilot_estop(&g_pilot);
    }

    return 0U != status.hold_mask;
}

/*!
 * @brief Record a changed decision: print it and post the event.
 */
static void line_pilot_note (pilot_cmd_t const * p_cmd)
{
    if (pilot_log_note(&g_pilot_log, p_cmd))
    {
        LOG_INFO(LOG_TASK_LINE, LOG_MOD_NONE, "pilot: behaviour %d reason %d speed %d turn %d",
                 (int)p_cmd->behaviour, (int)p_cmd->reason, (int)p_cmd->speed_mm_s,
                 (int)p_cmd->turn_deci_s);
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_PILOT_DECISION,
                                   p_cmd->speed_mm_s,
                                   (uint16_t)((p_cmd->behaviour * PILOT_EVENT_BEHAVIOUR_MUL) +
                                              p_cmd->reason),
                                   odometer_mm());
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

    hug_word_pack(words, &g_pilot.hug, servo_deci, hug_heading_deci(&g_pilot.hug));
    /* The count moves only when the hug content does, so a changed value
     * marks a new sample. g_hug_words is written by this task only. */
    if ((0U != ((words[0] ^ g_hug_words[0]) & ~(HUG_WORD_SEQ_MK << HUG_WORD_SEQ_SH))) ||
        (words[1] != g_hug_words[1]))
    {
        g_hug_seq++;
    }
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
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                   (int16_t)g_pilot.follow.side,
                                   (uint16_t)(LINE_SEARCH_SWEEP_DECI / DECI_PER_DEG), odometer_mm());
    }
}

/*!
 * @brief One pass of the pilot: measure, decide, drive.
 */
static void line_pilot_step (ir_sensors_state_t state)
{
    uint8_t const resets = supervisor_reset_count();
    uint8_t const starts = supervisor_start_count();

    static int64_t    s_left_um;
    static int64_t    s_right_um;
    pilot_meas_t      meas;
    pilot_cmd_t       cmd;
    motion_feedback_t feedback;
    uint32_t const    now_ms = line_now_ms();
    uint32_t const    dt_ms  = now_ms - g_pilot_last_ms;
    int32_t           left_um_s  = 0;
    int32_t           right_um_s = 0;
    bool              b_held     = false;

    b_held = line_pilot_stop_state();
    if (pilot_count_moved(&g_pilot_reset_seen, resets))
    {
        /* RESET: the ENCODER and HUG_TIMEOUT latches end; the ESTOP stays. The
         * supervisor holds the wheels until START. */
        pilot_clear_latches(&g_pilot);
        g_pilot_ping_sent = 0U;
    }
    if (pilot_count_moved(&g_pilot_start_seen, starts))
    {
        /* START ends a latched sonar or stale stop, as RESET does. */
        pilot_release_sonar(&g_pilot);
    }

    if ((E_OK == motion_feedback_snapshot(&feedback)) && feedback.fresh)
    {
        s_left_um  = feedback.left_displacement_um;
        s_right_um = feedback.right_displacement_um;
        g_pilot_stamp_ms = now_ms - (feedback.age_us / LINE_US_PER_MS);
    }

    meas = (pilot_meas_t){0};
    meas.left_um       = s_left_um;
    meas.right_um      = s_right_um;
    meas.now_ms        = now_ms;
    meas.stamp_ms      = g_pilot_stamp_ms;
    meas.b_line_left   = state.left;
    meas.b_line_right  = state.right;
    meas.b_held        = b_held || g_pilot_turn_ended;
    meas.b_barcode     = ir_barcode_recent();
    g_pilot_turn_ended = false;
    line_pilot_ping_meas(&meas);

    cmd = (pilot_cmd_t){0};
    line_follow_set_hump(&g_pilot.follow, line_on_hump());
    pilot_step(&g_pilot, &meas, &cmd);
    line_pilot_watch(&cmd, s_left_um, s_right_um);
    line_pilot_ping_request(&cmd);
    line_pilot_publish_hug(meas.ping_servo_deci);
    line_pilot_note(&cmd);

    pilot_drive(&g_drive, &cmd, dt_ms, &left_um_s, &right_um_s);
    g_pilot_last_ms = now_ms;
    (void)motion_set_speed(left_um_s, right_um_s, LINE_LEASE_US);
}
#endif

void line_task_stats (line_task_stats_t * p_stats)
{
    p_stats->lost_count = g_line_lost_count;
    p_stats->search_mm  = g_line_search_mm;
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
#if PICO_CAR_PILOT
    /* Set up before the loop: a turn that ends first sets the side on the
     * follower, and a later init would wipe it. */
    pilot_init(&g_pilot);
    drive_init(&g_drive);
    pilot_log_init(&g_pilot_log);
    g_pilot_stamp_ms = line_now_ms();
    g_pilot_reset_seen = supervisor_reset_count();
    g_pilot_start_seen = supervisor_start_count();
#endif
    for (;;)
    {
        const ir_sensors_state_t state = ir_get_state();
        uint8_t                  source = 0U;
        uint8_t const            taken  = supervisor_take_turn(&source);

        liveness_kick(LIVE_LINE);
        if (SUPERVISOR_NO_TURN != taken)
        {
#if PICO_CAR_PILOT
            /* The supervisor checked before it allowed the turn. A turn runs
             * only while the pilot follows the line. */
            if (!pilot_log_is_following(&g_pilot_log))
#else
            obstacle_status_t obstacle;

            /* The supervisor checked before it allowed the turn. Check again: the
             * obstacle task can have started a scan in the meantime. */
            if ((E_OK == obstacle_snapshot(&obstacle)) &&
                (TELEMETRY_OBSTACLE_STATE_MONITOR != obstacle.state))
#endif
            {
                supervisor_turn_refused(taken);
            }
            else
            {
                g_turn_wait       = (uint8_t)line_turn_of(taken);
                g_turn_wait_at_mm = odometer_mm() +
                    ((VEHICLE_SRC_BARCODE == source) ? BARCODE_TURN_DELAY_MM : 0U);
            }
        }
        if (((uint8_t)LINE_TURN_KIND_NONE != g_turn_wait) &&
            ((int32_t)(odometer_mm() - g_turn_wait_at_mm) >= 0))
        {
            line_turn_kind_t const turn = (line_turn_kind_t)g_turn_wait;

            g_turn_wait = (uint8_t)LINE_TURN_KIND_NONE;
            line_execute_turn(turn);
            continue;
        }

#if PICO_CAR_PILOT
        line_pilot_step(state);
#else
        if (obstacle_line_step(state.left, state.right))
        {
            /* obstacle is holding the car or driving round an obstacle. A
             * search that was running would otherwise time out on return. */
            line_follow_reset();
        }
        else
        {
            line_follow_step(state);
        }
#endif
        ir_wait_change(IR_TYPE_LINE, LINE_POLL_MS);
    }
}

