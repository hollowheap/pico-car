/** @file line_task.c
 *
 * @brief Line task body: follow the line, run the turns the supervisor
 *        allowed, and give the motors to the obstacle step when it asks.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "supervisor/command.h"
#include "supervisor/supervisor.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
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
#define UM_PER_MM (1000)
#define DECI_PER_DEG (10)

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

static line_follow_t g_line_follow;
static bool          g_line_lost;
static int64_t       g_line_travel_um;
static int32_t       g_line_heading_deci;

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
        system_fault_set(FAULT_LINE_LOST_BIT);
        g_line_lost_count++;
        g_line_lost_at_um = g_line_travel_um;
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                   (int16_t)g_line_follow.side, 0U, odometer_mm());
    }
    else if (!b_lost && g_line_lost)
    {
        system_fault_clear(FAULT_LINE_LOST_BIT);
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_FOUND,
                                   (int16_t)g_line_follow.side, driven_u16,
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

/* Forget the search state, e.g. when the obstacle step took the motors. */
static void line_follow_reset (void)
{
    line_follow_init(&g_line_follow);
    line_update_fault(false);
}

/* One-shot pivot by encoder heading. Every pass reads the feedback
 * snapshot and the line sensors, then line_turn_update() decides the wheel
 * targets. The turn ends on line capture, an abort, or LINE_TURN_MAX_MS. */
static void line_execute_turn (line_turn_kind_t turn)
{
    line_turn_state_t  state;
    line_targets_t     targets = {0, 0};
    motion_feedback_t  feedback;
    line_turn_status_t status = LINE_TURN_ABORTED;
    uint32_t           elapsed_ms = 0U;
    uint32_t const     start_ms   = line_now_ms();
    bool               b_snapshot = false;
    bool               b_obstacle = false;

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

    while ((LINE_TURN_TURNING == status) && (elapsed_ms < LINE_TURN_MAX_MS))
    {
        ir_sensors_state_t const sensors = ir_get_state();

        /* The obstacle step runs on every pass of a turn (F2). When it takes the
         * motors, the turn ends and the obstacle task drives. */
        if (obstacle_line_step(sensors.left, sensors.right))
        {
            b_obstacle = true;
            status     = LINE_TURN_ABORTED;
            break;
        }

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
    if (!b_obstacle)
    {
        (void)motion_set_speed(0, 0, LINE_LEASE_US);
    }
    g_turning = false;
    supervisor_set_turning(false);
    (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                               (uint8_t)((LINE_TURN_DONE == status) ? TELEMETRY_EVT_TURN_DONE
                                                                    : TELEMETRY_EVT_TURN_ABORT),
                               (int16_t)turn, (uint16_t)elapsed_ms, odometer_mm());

    if (b_obstacle)
    {
        /* The obstacle task has the motors; forget the search state. */
        line_follow_reset();
    }
    else if (LINE_TURN_DONE == status)
    {
        line_follow_set_side(&g_line_follow, line_turn_side(&state));
        line_update_fault(false);
    }
    else
    {
        /* Stopped above. The car looks for the line on the spot. The search
         * sets the line-lost fault on the next pass. */
        line_follow_start_search(&g_line_follow);
    }
}

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
 *        neither sensor the car holds its heading for LINE_GAP_MM, then
 *        turns on the spot to find the line. When both search turns find
 *        nothing it stops and waits for a sensor hit.
 */
static void line_follow_step (ir_sensors_state_t state)
{
    line_targets_t       targets = {0, 0};
    motion_feedback_t    feedback;
    line_follow_status_t status = LINE_FOLLOW_FOLLOWING;
    bool const           b_was_stopped =
        ((uint8_t)LINE_FOLLOW_STOPPED == g_line_follow.status);

    /* A stopped car stays stopped: a stray barcode edge must not drive it. */
    if (ir_barcode_recent() && !b_was_stopped)
    {
        /* Barcode stripes under the line sensors look like a lost line. For 250 ms
         * after any barcode edge the car drives straight and the search state does
         * not change. */
        targets.left_um_s  = LINE_CRUISE_UM_S;
        targets.right_um_s = LINE_CRUISE_UM_S;
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

    status = line_follow_update(&g_line_follow, state.left, state.right,
                                g_line_heading_deci, g_line_travel_um,
                                line_now_ms(), &targets);
    /* Line lost means the gap is over; the car turns on the spot, then stops
     * when both turns find nothing. A sensor hit ends it. */
    line_update_fault((LINE_FOLLOW_SEARCHING == status) ||
                      (LINE_FOLLOW_STOPPED == status));
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

void line_task_stats (line_task_stats_t * p_stats)
{
    p_stats->lost_count = g_line_lost_count;
    p_stats->search_mm  = g_line_search_mm;
    p_stats->b_turning  = g_turning;
}

void line_task_run (void)
{
    for (;;)
    {
        const ir_sensors_state_t state = ir_get_state();
        uint8_t                  source = 0U;
        uint8_t const            taken  = supervisor_take_turn(&source);

        liveness_kick(LIVE_LINE);
        if (SUPERVISOR_NO_TURN != taken)
        {
            obstacle_status_t obstacle;

            /* The supervisor checked before it allowed the turn. Check again: the
             * obstacle task can have started a scan since (F3). */
            if ((E_OK == obstacle_snapshot(&obstacle)) &&
                (TELEMETRY_OBSTACLE_STATE_MONITOR != obstacle.state))
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
        ir_wait_change(IR_TYPE_LINE, LINE_POLL_MS);
    }
}

