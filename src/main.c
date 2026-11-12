#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "track/track.h"
#include "track/line_control.h"
#include "motion/motion.h"
#include "motion/motion_request.h"
#include "motion/motion_service.h"
#include "motion/motor_setup.h"
#include "motion/encoder.h"
#include "navigation/obstacle.h"
#include "track/barcode.h"
#include "physics/physics.h"
#include "platform/faults.h"
#include "platform/isr.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/static_assert.h"
#include "supervisor/supervisor.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/cmd.h"
#include "telemetry/connection.h"
#include "telemetry/net_queue.h"
#include "telemetry/packet.h"
#include "telemetry/queue.h"
#include "telemetry/worker.h"

/* Every task uses 4096 bytes.  The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ 4096
#define STACK_ALIGN 8U

/* Line following. The cruise speed, the search limits and the turn speeds
 * live in track/line_control.h (tune them there on the car). The duty cap in
 * motion_setup() must leave headroom for them. The follower steers by a
 * heading target and never stops a wheel; one-shot turns pivot in place.
 * LINE_POLL_MS is the control period. */
#define LINE_POLL_MS             (10U)
/* Upper bound on one turn, so a blocked car cannot pivot for ever. Tune on
 * the car: it must exceed the longest U-turn, including the slow phase. */
#define LINE_TURN_MAX_MS         (4000U)

/* Width of the fault word (bits 0..15, see platform/faults.h). */
#define FAULT_WORD_BITS              (16U)

/* Startup barrier event flag bits. */
#define STARTUP_FLAG_READY           (0x01U)
#define STARTUP_FLAG_FAILED          (0x02U)

/* Telemetry pacing, in 50 ms passes of telemetry_task (D110): status, motion
 * and physics every 200 ms (5 Hz, R01), obstacle every 500 ms, diag every 1 s.
 * The heartbeat is sent by the connection task. */
#define TELEMETRY_PASS_MS            (50U)
#define TELEMETRY_FAST_PASSES        (4U)
#define TELEMETRY_OBSTACLE_PASSES    (10U)
#define TELEMETRY_DIAG_PASSES        (20U)
#define TELEMETRY_SLEEP_TIMEOUT_MS   (250U)

/* Line bits of the status packet (protocol/telemetry.yaml, line_bits). */
#define LINE_BIT_LEFT                (1U << 0U)
#define LINE_BIT_RIGHT               (1U << 1U)
#define LINE_BIT_BARCODE             (1U << 2U)

/* Telemetry reads the obstacle profile with the wire's slot count. */
STATIC_ASSERT (OBSTACLE_PROFILE_SLOTS == TELEMETRY_PROFILE_COUNT_SLOTS,
               "profile slots match the wire");

/* Startup barrier. This file creates every task; one startup task owns the
 * shared hardware init and the others wait for it. Subsystems never call
 * tk_cre_tsk() or tk_sta_tsk(). */
typedef struct
{
    ID h_flg;
    ID motion_tid;
    ID physics_tid;
    ID telemetry_tid;
    ID supervisor_tid;
} startup_state_t;

static startup_state_t g_startup = {0, 0, 0, 0, 0};
static ID              g_telemetry_cyc_id;
static uint32_t        g_telemetry_overruns;

LOCAL void telemetry_cyclic_wakeup (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_startup.telemetry_tid);
}

LOCAL void supervisor_cyclic_wakeup (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_startup.supervisor_tid);
}

/*!
 * @brief Wait until the startup task has finished. Returns false on a
 *        startup failure; the caller must then park.
 */
LOCAL bool wait_startup (void)
{
    UINT flgptn = 0U;
    ER const er = tk_wai_flg(g_startup.h_flg,
                             STARTUP_FLAG_READY | STARTUP_FLAG_FAILED,
                             TWF_ORW, &flgptn, TMO_FEVR);

    if (E_OK != er)
    {
        return false;
    }

    return (0U != (flgptn & STARTUP_FLAG_READY));
}

/*!
 * @brief Park a task whose service could not start.
 */
LOCAL void park (void)
{
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

LOCAL void startup_task (INT stacd, void * exinf)
{
    ER result = gpio_irq_init();

    (void)stacd;
    (void)exinf;

    if (E_OK == result)
    {
        result = motion_setup(g_startup.motion_tid);
    }
    if (E_OK == result)
    {
        result = physics_init(g_startup.physics_tid);
    }
    if (E_OK == result)
    {
        result = ir_capture_init();
    }
    if (E_OK == result)
    {
        /* Obstacle avoidance. If it fails, line following still
         * runs, but this build stops instead of driving blind. */
        result = obstacle_line_init();
    }

    if (E_OK == result)
    {
        (void)tk_set_flg(g_startup.h_flg, STARTUP_FLAG_READY);
    }
    else
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "startup failed, er=%d",
                  (int)result);
        /* The telemetry task parks on a failed startup, so print now. */
        log_flush();
        (void)tk_set_flg(g_startup.h_flg, STARTUP_FLAG_FAILED);
    }

    (void)tk_exd_tsk();
}

LOCAL void motion_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    motion_service_run();
}

LOCAL void physics_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    physics_run();
}

LOCAL void obstacle_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    obstacle_run();
}

typedef enum
{
    LINE_TURN_NONE = 0,
    LINE_TURN_LEFT,
    LINE_TURN_RIGHT,
    LINE_TURN_UTURN,
    LINE_TURN_STRAIGHT
} line_turn_t;

/* The distance a barcode turn waits after the barcode, so the pivot happens at the
 * junction and not at the first stripe. Tune on the car. 0 turns at once. */
#define BARCODE_TURN_DELAY_MM        (0U)

/* A turn the supervisor allowed and the line task has not run yet. */
static line_turn_t g_turn_wait      = LINE_TURN_NONE;
static uint32_t    g_turn_wait_at_mm = 0U;
static uint8_t     g_turn_wait_op   = 0U;

LOCAL uint32_t current_odometer_mm (void);
LOCAL uint16_t clamp_u16 (uint32_t value);

/* Wheel displacement units. */
#define UM_PER_MM (1000)

/* Map a vehicle command opcode to a turn. */
static line_turn_t line_turn_of (uint8_t opcode)
{
    line_turn_t turn = LINE_TURN_NONE;

    switch (opcode)
    {
        case VEHICLE_CMD_LEFT:
            turn = LINE_TURN_LEFT;
            break;
        case VEHICLE_CMD_RIGHT:
            turn = LINE_TURN_RIGHT;
            break;
        case VEHICLE_CMD_UTURN:
            turn = LINE_TURN_UTURN;
            break;
        case VEHICLE_CMD_STRAIGHT:
            turn = LINE_TURN_STRAIGHT;
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
 * lost. The car never stops for it (D180). */
static void line_update_fault (bool b_lost)
{
    int64_t  const driven_um = g_line_travel_um - g_line_lost_at_um;
    uint32_t const driven_mm = (uint32_t)(((driven_um < 0) ? -driven_um : driven_um) / UM_PER_MM);

    if (b_lost && !g_line_lost)
    {
        system_fault_set(FAULT_LINE_LOST_BIT);
        g_line_lost_count++;
        g_line_lost_at_um = g_line_travel_um;
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_LOST,
                                   (int16_t)g_line_follow.side, 0U, current_odometer_mm());
    }
    else if (!b_lost && g_line_lost)
    {
        system_fault_clear(FAULT_LINE_LOST_BIT);
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                                   (uint8_t)TELEMETRY_EVT_LINE_FOUND,
                                   (int16_t)g_line_follow.side, clamp_u16(driven_mm),
                                   current_odometer_mm());
    }
    else
    {
        /* No change. */
    }
    if (b_lost && (clamp_u16(driven_mm) > g_line_search_mm))
    {
        g_line_search_mm = clamp_u16(driven_mm);
    }
    g_line_lost = b_lost;
}

/* Forget the search state, e.g. when the obstacle step took the motors. */
static void line_follow_reset (void)
{
    line_follow_init(&g_line_follow);
    line_update_fault(false);
}

/* Map the CLI turn request to the line_control turn kind. */
static bool line_turn_kind (line_turn_t turn, line_turn_kind_t * p_kind)
{
    bool b_valid = true;

    switch (turn)
    {
        case LINE_TURN_LEFT:
            *p_kind = LINE_TURN_KIND_LEFT;
            break;
        case LINE_TURN_RIGHT:
            *p_kind = LINE_TURN_KIND_RIGHT;
            break;
        case LINE_TURN_UTURN:
            *p_kind = LINE_TURN_KIND_UTURN;
            break;
        case LINE_TURN_STRAIGHT:
            *p_kind = LINE_TURN_KIND_STRAIGHT;
            break;
        default:
            b_valid = false;
            break;
    }

    return b_valid;
}

/* One-shot pivot by encoder heading (D93). Every pass reads the feedback
 * snapshot and the line sensors, then line_turn_update() decides the wheel
 * targets. The turn ends on line capture, an abort, or LINE_TURN_MAX_MS. */
static void line_execute_turn (line_turn_t turn)
{
    line_turn_kind_t   kind = LINE_TURN_KIND_LEFT;
    line_turn_state_t  state;
    line_targets_t     targets = {0, 0};
    motion_feedback_t  feedback;
    line_turn_status_t status = LINE_TURN_ABORTED;
    uint32_t           elapsed_ms = 0U;
    bool               b_snapshot = false;
    bool               b_obstacle = false;

    if (!line_turn_kind(turn, &kind))
    {
        return;
    }

    g_turning  = true;
    supervisor_set_turning(true);
    (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_LINE,
                               (uint8_t)TELEMETRY_EVT_TURN_START, (int16_t)turn, 0U,
                               current_odometer_mm());
    b_snapshot = (E_OK == motion_feedback_snapshot(&feedback));
    if (b_snapshot)
    {
        line_turn_start(&state, kind, feedback.left_displacement_um,
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
                (void)ir_wait_change(IR_TYPE_LINE, (TMO)LINE_POLL_MS);
                elapsed_ms += LINE_POLL_MS;
            }
        }
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
                               (int16_t)turn, (uint16_t)elapsed_ms, current_odometer_mm());

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
        /* Stopped above. The car looks for the line; it does not stay stopped
         * (D180). The search sets the line-lost fault on the next pass. */
        line_follow_start_search(&g_line_follow);
    }
}

/*!
 * @brief One pass of the line follower (D181). A sensor hit slews a heading
 *        target toward the line and the encoder heading follows it. With
 *        neither sensor the car holds its heading for LINE_GAP_MM, then
 *        weaves at reduced speed. It never stops (D180).
 */
LOCAL void line_follow_step (ir_sensors_state_t state)
{
    line_targets_t       targets = {0, 0};
    motion_feedback_t    feedback;
    line_follow_status_t status = LINE_FOLLOW_FOLLOWING;

    if (ir_barcode_recent())
    {
        /* Barcode stripes under the line sensors look like a lost line. For 250 ms
         * after any barcode edge the car drives straight and the search state does
         * not change (D130). */
        (void)motion_set_speed(LINE_CRUISE_UM_S, LINE_CRUISE_UM_S, LINE_LEASE_US);
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
    /* Line lost means the gap is over and the car weaves. A sensor hit ends it. */
    line_update_fault(LINE_FOLLOW_SEARCHING == status);
    (void)motion_set_speed(targets.left_um_s, targets.right_um_s,
                           LINE_LEASE_US);
}

LOCAL void line_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

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
                g_turn_wait       = line_turn_of(taken);
                g_turn_wait_op    = taken;
                g_turn_wait_at_mm = current_odometer_mm() +
                    ((VEHICLE_SRC_BARCODE == source) ? BARCODE_TURN_DELAY_MM : 0U);
            }
        }
        if ((LINE_TURN_NONE != g_turn_wait) &&
            ((int32_t)(current_odometer_mm() - g_turn_wait_at_mm) >= 0))
        {
            line_turn_t const turn = g_turn_wait;

            g_turn_wait = LINE_TURN_NONE;
            (void)g_turn_wait_op;
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

/* Odometer for an event, from the motion snapshot (0 if it is not running). */
LOCAL uint32_t current_odometer_mm (void)
{
    motion_feedback_t feedback;

    if (E_OK != motion_feedback_snapshot(&feedback))
    {
        return 0U;
    }
    return telemetry_odometer_mm(&feedback);
}

LOCAL void supervisor_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    supervisor_run();
}

LOCAL void barcode_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    barcode_task_run();
}

/*!
 * @brief Queue one telemetry frame for the connection task.
 */
LOCAL int net_sink (const uint8_t * frame, uint16_t len)
{
    (void)telemetry_net_queue_push(frame, len);
    return 0;
}

/* What the telemetry task remembers between passes. Only that task uses it. */
typedef struct
{
    uint32_t edges;          /* ir_line_edges at the last diag frame     */
    uint32_t raw_edges;      /* ir_line_raw_edges at the last diag frame */
    uint32_t echo[OBSTACLE_ECHO_CLASSES]; /* ping counts at the last diag frame */
    uint16_t fault_level;    /* fault word at the last pass              */
    uint16_t seq;            /* status frame counter                     */
    uint8_t  obstacle_state; /* obstacle state at the last pass          */
} telemetry_memory_t;

static telemetry_memory_t g_tel;

LOCAL uint16_t clamp_u16 (uint32_t value)
{
    return (value > UINT16_MAX) ? (uint16_t)UINT16_MAX : (uint16_t)value;
}

LOCAL uint8_t clamp_u8 (uint32_t value)
{
    return (value > UINT8_MAX) ? (uint8_t)UINT8_MAX : (uint8_t)value;
}

LOCAL uint8_t current_line_bits (void)
{
    ir_sensors_state_t const ir   = ir_get_state();
    uint8_t                  bits = 0U;

    if (ir.left)
    {
        bits |= (uint8_t)LINE_BIT_LEFT;
    }
    if (ir.right)
    {
        bits |= (uint8_t)LINE_BIT_RIGHT;
    }
    if (ir.barcode)
    {
        bits |= (uint8_t)LINE_BIT_BARCODE;
    }
    return bits;
}

/*!
 * @brief Queue the 5 Hz frames: status, motion and physics. Each subsystem
 *        snapshot is read once.
 */
LOCAL void telemetry_send_fast (void)
{
    uint8_t              packet[TELEMETRY_WIRE_MAX];
    motion_status_t      motion;
    motion_feedback_t    feedback;
    motion_diag_t        diag;
    physics_status_t     physics;
    obstacle_status_t    obstacle;
    telemetry_status_t   status;
    telemetry_motion_t   moving;
    telemetry_physics_t  imu;
    bool const           b_motion   = (E_OK == motion_status(&motion));
    bool const           b_feedback = (E_OK == motion_feedback_snapshot(&feedback));
    bool const           b_diag     = (E_OK == motion_diag_snapshot(&diag));
    bool const           b_physics  = (E_OK == physics_snapshot(&physics));
    bool const           b_obstacle = (E_OK == obstacle_snapshot(&obstacle));

    telemetry_assemble_status(&status, b_motion ? &motion : NULL,
                              b_feedback ? &feedback : NULL,
                              b_physics ? &physics : NULL,
                              b_obstacle ? &obstacle : NULL, system_fault_get(),
                              g_tel.seq, current_line_bits(), g_turning);
    g_tel.seq++;
    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_STATUS, packet,
        telemetry_pack_status(packet, &status), 0U);

    telemetry_assemble_motion(&moving, b_motion ? &motion : NULL,
                              b_feedback ? &feedback : NULL,
                              b_diag ? &diag : NULL);
    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_MOTION, packet,
        telemetry_pack_motion(packet, &moving), 0U);

    telemetry_assemble_physics(&imu, b_physics ? &physics : NULL);
    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_PHYSICS, packet,
        telemetry_pack_physics(packet, &imu), 0U);
}

/*!
 * @brief Queue the obstacle frame (2 Hz).
 */
LOCAL void telemetry_send_obstacle (void)
{
    uint8_t              packet[TELEMETRY_WIRE_MAX];
    obstacle_status_t       obstacle;
    obstacle_plan_summary_t plan;
    telemetry_obstacle_t    frame;
    bool const              b_status = (E_OK == obstacle_snapshot(&obstacle));
    bool const              b_plan   = (E_OK == obstacle_copy_plan(&plan));

    telemetry_assemble_obstacle(&frame, b_status ? &obstacle : NULL,
                                b_plan ? &plan : NULL);
    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_OBSTACLE, packet,
        telemetry_pack_obstacle(packet, &frame), 0U);
}

/*!
 * @brief Queue the diag frame (1 Hz): counters and health.
 */
LOCAL void telemetry_send_diag (void)
{
    uint8_t           packet[TELEMETRY_WIRE_MAX];
    physics_status_t  physics;
    telemetry_diag_t  diag = {0};
    uint32_t const    edges     = ir_line_edges();
    uint32_t const    raw_edges = ir_line_raw_edges();
    uint32_t          echo[OBSTACLE_ECHO_CLASSES] = {0};
    barcode_stats_t   bstats;

    barcode_stats(&bstats);
    diag.sticky_faults       = system_fault_sticky_get();
    diag.line_lost_count     = g_line_lost_count;
    diag.line_search_mm      = g_line_search_mm;
    diag.barcode_ok          = bstats.ok;
    diag.barcode_bad         = bstats.bad;
    diag.barcode_edge_drops  = clamp_u16(ir_barcode_drops());
    diag.last_barcode        = bstats.last_letter;
    diag.last_barcode_action = bstats.last_action;
    diag.task_alive_mask     = supervisor_alive_mask();
    diag.supervisor_passes   = supervisor_pass_count();
    diag.telem_queue_drops   = clamp_u16((uint32_t)telemetry_queue_drop_count());
    diag.net_queue_drops     = clamp_u16((uint32_t)telemetry_net_queue_drop_count());
    diag.telemetry_overruns  = clamp_u16(g_telemetry_overruns);
    if (E_OK == physics_snapshot(&physics))
    {
        diag.physics_overruns = clamp_u16(physics.overruns);
    }
    /* Flips since the last diag frame, which is about one second ago. */
    diag.line_edges_per_s     = clamp_u8(edges - g_tel.edges);
    diag.line_raw_edges_per_s = clamp_u8(raw_edges - g_tel.raw_edges);
    g_tel.edges     = edges;
    g_tel.raw_edges = raw_edges;

    /* Pings per class since the last diag frame. */
    obstacle_echo_counts(echo);
    diag.echo_valid     = clamp_u8(echo[OBSTACLE_ECHO_IDX_VALID] - g_tel.echo[OBSTACLE_ECHO_IDX_VALID]);
    diag.echo_open      = clamp_u8(echo[OBSTACLE_ECHO_IDX_OPEN] - g_tel.echo[OBSTACLE_ECHO_IDX_OPEN]);
    diag.echo_too_close = clamp_u8(echo[OBSTACLE_ECHO_IDX_TOO_CLOSE] - g_tel.echo[OBSTACLE_ECHO_IDX_TOO_CLOSE]);
    diag.echo_no_echo   = clamp_u8(echo[OBSTACLE_ECHO_IDX_NO_ECHO] - g_tel.echo[OBSTACLE_ECHO_IDX_NO_ECHO]);
    diag.echo_glitch    = clamp_u8(echo[OBSTACLE_ECHO_IDX_GLITCH] - g_tel.echo[OBSTACLE_ECHO_IDX_GLITCH]);
    diag.echo_stuck     = clamp_u8(echo[OBSTACLE_ECHO_IDX_STUCK] - g_tel.echo[OBSTACLE_ECHO_IDX_STUCK]);
    for (uint8_t idx = 0U; idx < OBSTACLE_ECHO_CLASSES; idx++)
    {
        g_tel.echo[idx] = echo[idx];
    }

    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_DIAG, packet,
        telemetry_pack_diag(packet, &diag), 0U);
}

/*!
 * @brief Send one event for every fault bit that changed since the last pass.
 */
LOCAL void telemetry_check_faults (void)
{
    uint16_t const level   = system_fault_get();
    uint16_t const changed = (uint16_t)(level ^ g_tel.fault_level);

    if (0U != changed)
    {
        uint32_t const odometer = current_odometer_mm();

        for (uint16_t bit = 0U; bit < FAULT_WORD_BITS; bit++)
        {
            uint16_t const mask = (uint16_t)(1U << bit);

            if (0U != (changed & mask))
            {
                (void)telemetry_emit_event(
                    (uint8_t)TELEMETRY_EVENT_SOURCE_TELEMETRY,
                    (0U != (level & mask)) ? (uint8_t)TELEMETRY_EVT_FAULT_SET
                                           : (uint8_t)TELEMETRY_EVT_FAULT_CLEAR,
                    (int16_t)bit, level, odometer);
            }
        }
        g_tel.fault_level = level;
    }
}

/*!
 * @brief When a scan finishes (SCANNING to PLAN), send its profile.
 */
LOCAL void telemetry_check_profile (void)
{
    obstacle_status_t   obstacle;
    uint8_t             packet[TELEMETRY_WIRE_MAX];
    telemetry_profile_t frame;
    uint8_t             previous = g_tel.obstacle_state;

    if (E_OK != obstacle_snapshot(&obstacle))
    {
        return;
    }
    g_tel.obstacle_state = obstacle.state;
    if ((previous == (uint8_t)TELEMETRY_OBSTACLE_STATE_SCANNING) &&
        (obstacle.state == (uint8_t)TELEMETRY_OBSTACLE_STATE_PLAN) &&
        (E_OK == obstacle_copy_profile(frame.slots)))
    {
        frame.closest_mm = 0U;
        frame.width_mm   = 0U;
        frame.req_id     = obstacle.req_id;
        frame.status     = 0U;
        frame.side       = 0U;
        frame.flags      = 0U;
        (void)telemetry_queue_push_packet(
            (uint8_t)TELEMETRY_TOPIC_PROFILE, packet,
            telemetry_pack_profile(packet, &frame), 0U);
    }
}

/*!
 * @brief Drain the queue, then build the frames that are due this pass. Every
 *        pass is 50 ms. Status, motion and physics go out every fourth pass
 *        (200 ms), the obstacle frame every tenth, the diag frame every
 *        twentieth.
 */
LOCAL void telemetry_task (INT stacd, void * exinf)
{
    uint32_t pass = 0U;

    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    for (;;)
    {
        liveness_kick(LIVE_TELEMETRY);
        telemetry_worker_run(telemetry_queue_pop, net_sink, TMO_POL);
        log_drain();

        pass++;
        telemetry_check_faults();
        telemetry_check_profile();
        if (0U == (pass % TELEMETRY_FAST_PASSES))
        {
            telemetry_send_fast();
        }
        if (0U == (pass % TELEMETRY_OBSTACLE_PASSES))
        {
            telemetry_send_obstacle();
        }
        if (0U == (pass % TELEMETRY_DIAG_PASSES))
        {
            telemetry_send_diag();
        }

        INT queued = 0;
        ER const er = tk_slp_tsk((TMO)TELEMETRY_SLEEP_TIMEOUT_MS);

        queued = tk_can_wup(TSK_SELF);
        if ((E_OK == er) && (queued > 0))
        {
            g_telemetry_overruns += (uint32_t)queued;
        }
    }
}

/*!
 * @brief Hand a telemetry command to the supervisor (D120).
 *
 * The supervisor decides. The emergency stop also acts here, at once: it
 * never waits for the next supervisor pass (D75). The latch needs a reboot.
 */
LOCAL void command_dispatch (const telemetry_cmd_t * cmd, void * context)
{
    (void)context;

    if ((uint8_t)TELEMETRY_CMD_ESTOP == cmd->opcode)
    {
        (void)motion_request_stop();
    }
    (void)vehicle_command_post(cmd->opcode, VEHICLE_SRC_TELEMETRY);
}

LOCAL void connection_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    connection_run();
}

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 * ------------------------------------------------------------------ */

static uint8_t g_startup_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_motion_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_physics_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_obstacle_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_line_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_telemetry_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_connection_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_barcode_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_supervisor_stack[STACK_SZ] __attribute__((aligned(STACK_ALIGN)));

/* Stack marks (Q25). Before any task exists, every stack is filled with a
 * known byte. The supervisor reads how much of each stack is still untouched,
 * counted from the deep end, and logs the smallest margins. */
#define STACK_FILL_BYTE   (0xA5U)
#define STACK_MIN_FREE    (512U)
#define STACK_REPORT_CALLS (30U)
#define STACK_COUNT       (9U)

static uint8_t * const g_stack_base[STACK_COUNT] = {
    g_startup_stack,   g_motion_stack,    g_physics_stack,
    g_obstacle_stack,  g_line_stack,      g_telemetry_stack,
    g_connection_stack, g_barcode_stack,  g_supervisor_stack,
};

LOCAL void stack_fill (void)
{
    uint32_t idx  = 0U;
    uint32_t byte = 0U;

    for (idx = 0U; idx < STACK_COUNT; idx++)
    {
        for (byte = 0U; byte < STACK_SZ; byte++)
        {
            g_stack_base[idx][byte] = STACK_FILL_BYTE;
        }
    }
}

/* Untouched bytes of one stack, from the deep (low) end. */
LOCAL uint32_t stack_free (uint32_t idx)
{
    uint32_t free_bytes = 0U;

    while ((free_bytes < STACK_SZ) && (STACK_FILL_BYTE == g_stack_base[idx][free_bytes]))
    {
        free_bytes++;
    }

    return free_bytes;
}

/* The supervisor calls this once a second. Every STACK_REPORT_CALLS calls it
 * logs the free bytes of every stack, and a warning for a thin margin. */
LOCAL void stack_report (void)
{
    static uint32_t s_calls = 0U;
    uint32_t        idx     = 0U;
    uint32_t        thin    = 0U;

    s_calls++;
    if (0U != (s_calls % STACK_REPORT_CALLS))
    {
        return;
    }
    for (idx = 0U; idx < STACK_COUNT; idx++)
    {
        if (stack_free(idx) < STACK_MIN_FREE)
        {
            thin |= (1UL << idx);
        }
    }
    LOG_INFO(LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
             "stack free: startup %d motion %d physics %d obstacle %d",
             (int)stack_free(0U), (int)stack_free(1U), (int)stack_free(2U), (int)stack_free(3U));
    LOG_INFO(LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
             "stack free: line %d telemetry %d connection %d barcode %d",
             (int)stack_free(4U), (int)stack_free(5U), (int)stack_free(6U), (int)stack_free(7U));
    LOG_INFO(LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "stack free: supervisor %d", (int)stack_free(8U));
    if (0U != thin)
    {
        LOG_WARN(LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "thin stack margin, mask=%d", (int)thin);
    }
}

LOCAL T_CTSK g_ctsk_startup = {
    .itskpri = 1,
    .stksz   = STACK_SZ,
    .task    = startup_task,
    .bufptr  = g_startup_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

LOCAL T_CTSK g_ctsk_motion = {
    .itskpri = 1,
    .stksz   = STACK_SZ,
    .task    = motion_task,
    .bufptr  = g_motion_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

LOCAL T_CTSK g_ctsk_physics = {
    .itskpri = 5, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_SZ,
    .task    = physics_task,
    .bufptr  = g_physics_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

LOCAL T_CTSK g_ctsk_supervisor = {
    .itskpri = 2, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_SZ,
    .task    = supervisor_task,
    .bufptr  = g_supervisor_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

LOCAL T_CTSK g_ctsk_obstacle = {
    .itskpri = 4, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_SZ,
    .task    = obstacle_task,
    .bufptr  = g_obstacle_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

LOCAL T_CTSK g_ctsk_line = {
    .itskpri = 3, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_SZ,
    .task    = line_task,
    .bufptr  = g_line_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

LOCAL T_CTSK g_ctsk_telemetry = {
    .itskpri = 7, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_SZ,
    .task    = telemetry_task,
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF,
    .bufptr  = g_telemetry_stack,
};

LOCAL T_CTSK g_ctsk_connection = {
    .itskpri = 6, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_SZ,
    .task    = connection_task,
    .bufptr  = g_connection_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

LOCAL T_CTSK g_ctsk_barcode = {
    .itskpri = 7, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_SZ,
    .task    = barcode_task,
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF,
    .bufptr  = g_barcode_stack,
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or print failure and keep going. */
LOCAL bool made (const char * what, ID id)
{
    bool b_made = (id > E_OK);

    if (!b_made)
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "tk_cre_%s failed, er=%d", what,
                  (int)id);
    }
    return b_made;
}

/* Start a task, or print failure and keep going. */
LOCAL bool started (const char * what, ID tid)
{
    ER   result = tk_sta_tsk(tid, 0);
    bool b_started = (E_OK == result);

    if (!b_started)
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "tk_sta_%s failed, er=%d", what,
                  (int)result);
    }
    return b_started;
}

/* Task IDs and the optional-service flags shared by the usermain helpers. */
typedef struct
{
    ID   startup_tid;
    ID   motion_tid;
    ID   physics_tid;
    ID   obstacle_tid;
    ID   line_tid;
    ID   supervisor_tid;
    ID   telemetry_tid;
    ID   connection_tid;
    ID   barcode_tid;
    bool b_telemetry;
    bool b_connection;
    bool b_barcode;
} task_set_t;

/*!
 * @brief Prepare the optional telemetry and connection services.
 */
LOCAL void init_services (task_set_t * p_tasks)
{
    ER result = E_OK;

    /* Telemetry is optional: the car still runs when the queues or the
     * network initialisation fail. */
    result = telemetry_queue_create();
    p_tasks->b_telemetry = (E_OK == result);
    result = telemetry_net_queue_create();
    p_tasks->b_telemetry = p_tasks->b_telemetry && (E_OK == result);
    if (!p_tasks->b_telemetry)
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "telemetry queues failed, er=%d",
                  (int)result);
    }

    result = connection_init(command_dispatch, NULL);
    p_tasks->b_connection = (E_OK == result);
    if (!p_tasks->b_connection)
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "connection_init failed, er=%d",
                  (int)result);
    }
}

/*!
 * @brief Create the supervisor task and the cyclic handler that wakes it.
 */
LOCAL bool create_supervisor (task_set_t * p_tasks)
{
    bool b_ok = made("tsk(supervisor)", (p_tasks->supervisor_tid = tk_cre_tsk(&g_ctsk_supervisor)));

    if (b_ok)
    {
        T_CCYC ccyc = {0};

        g_startup.supervisor_tid = p_tasks->supervisor_tid;
        ccyc.cycatr = TA_HLNG | TA_STA;
        ccyc.cychdr = (FP)supervisor_cyclic_wakeup;
        ccyc.cyctim = (RELTIM)SUPERVISOR_PASS_MS;
        ccyc.cycphs = (RELTIM)SUPERVISOR_PASS_MS;
        (void)tk_cre_cyc(&ccyc);
    }

    return b_ok;
}

/*!
 * @brief Give the supervisor the tasks that must show a pulse. A task that did
 *        not start is left out.
 */
LOCAL bool init_supervisor (task_set_t const * p_tasks)
{
    supervisor_config_t config = {0};

    config.required_mask = (uint16_t)((1U << LIVE_MOTION) | (1U << LIVE_LINE) |
                                      (1U << LIVE_OBSTACLE) | (1U << LIVE_PHYSICS));
    if (p_tasks->b_telemetry)
    {
        config.required_mask |= (uint16_t)(1U << LIVE_TELEMETRY);
    }
    if (p_tasks->b_connection)
    {
        config.required_mask |= (uint16_t)(1U << LIVE_CONNECTION);
    }
    if (p_tasks->b_barcode)
    {
        config.required_mask |= (uint16_t)(1U << LIVE_BARCODE);
    }
    config.p_background = stack_report;
    config.p_line_edges = ir_line_edges;
    if (E_OK != supervisor_init(&config))
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "supervisor_init failed");
        return false;
    }

    return true;
}

/*!
 * @brief Create every task. Returns false when a required task failed.
 */
LOCAL bool create_tasks (task_set_t * p_tasks)
{
    bool b_ok = true;

    /* D62/D70: this composition root creates every task. Subsystems only
     * prepare their resources and expose a task body. */
    p_tasks->startup_tid = tk_cre_tsk(&g_ctsk_startup);
    if (!made("tsk(startup)", p_tasks->startup_tid))
    {
        b_ok = false;
    }

    p_tasks->motion_tid = tk_cre_tsk(&g_ctsk_motion);
    if (!made("tsk(motion)", p_tasks->motion_tid))
    {
        b_ok = false;
    }
    else
    {
        g_startup.motion_tid = p_tasks->motion_tid;
    }

    p_tasks->physics_tid = tk_cre_tsk(&g_ctsk_physics);
    if (!made("tsk(physics)", p_tasks->physics_tid))
    {
        b_ok = false;
    }
    else
    {
        g_startup.physics_tid = p_tasks->physics_tid;
    }

    p_tasks->obstacle_tid = tk_cre_tsk(&g_ctsk_obstacle);
    if (!made("tsk(obstacle)", p_tasks->obstacle_tid))
    {
        b_ok = false;
    }

    p_tasks->line_tid = tk_cre_tsk(&g_ctsk_line);
    if (!made("tsk(line)", p_tasks->line_tid))
    {
        b_ok = false;
    }

    if (!create_supervisor(p_tasks))
    {
        b_ok = false;
    }

    if (p_tasks->b_telemetry)
    {
        p_tasks->telemetry_tid = tk_cre_tsk(&g_ctsk_telemetry);
        if (!made("tsk(telemetry)", p_tasks->telemetry_tid))
        {
            p_tasks->b_telemetry = false;
        }
        else
        {
            T_CCYC ccyc = {0};

            g_startup.telemetry_tid = p_tasks->telemetry_tid;
            ccyc.cycatr = TA_HLNG | TA_STA;
            ccyc.cychdr = (FP)telemetry_cyclic_wakeup;
            ccyc.cyctim = (RELTIM)TELEMETRY_PASS_MS;
            ccyc.cycphs = (RELTIM)TELEMETRY_PASS_MS;
            g_telemetry_cyc_id = tk_cre_cyc(&ccyc);
        }
    }

    p_tasks->barcode_tid = tk_cre_tsk(&g_ctsk_barcode);
    p_tasks->b_barcode   = made("tsk(barcode)", p_tasks->barcode_tid);

    if (p_tasks->b_connection)
    {
        p_tasks->connection_tid = tk_cre_tsk(&g_ctsk_connection);
        if (!made("tsk(connection)", p_tasks->connection_tid))
        {
            p_tasks->b_connection = false;
        }
    }

    if (b_ok && !init_supervisor(p_tasks))
    {
        b_ok = false;
    }

    return b_ok;
}

/*!
 * @brief Start the required tasks. Returns false when one failed to start.
 */
LOCAL bool start_required_tasks (task_set_t const * p_tasks)
{
    bool b_ok = true;

    if (!started("tsk(startup)", p_tasks->startup_tid))
    {
        b_ok = false;
    }
    if (!started("tsk(motion)", p_tasks->motion_tid))
    {
        b_ok = false;
    }
    if (!started("tsk(physics)", p_tasks->physics_tid))
    {
        b_ok = false;
    }
    if (!started("tsk(obstacle)", p_tasks->obstacle_tid))
    {
        b_ok = false;
    }
    if (!started("tsk(line)", p_tasks->line_tid))
    {
        b_ok = false;
    }
    if (!started("tsk(supervisor)", p_tasks->supervisor_tid))
    {
        b_ok = false;
    }

    return b_ok;
}

/*!
 * @brief Start the optional tasks. A failure is reported and ignored.
 */
LOCAL void start_optional_tasks (task_set_t const * p_tasks)
{
    if (p_tasks->b_telemetry)
    {
        (void)started("tsk(telemetry)", p_tasks->telemetry_tid);
    }
    if (p_tasks->b_connection)
    {
        (void)started("tsk(connection)", p_tasks->connection_tid);
    }
    if (p_tasks->b_barcode)
    {
        (void)started("tsk(barcode)", p_tasks->barcode_tid);
    }
}

EXPORT INT usermain (void)
{
    task_set_t tasks = {0};
    T_CFLG     cflg  = {0};

    log_init();
    stack_fill();

    cflg.flgatr     = TA_WMUL | TA_TFIFO;
    cflg.iflgptn    = 0U;
    g_startup.h_flg = tk_cre_flg(&cflg);

    init_services(&tasks);

    if (!create_tasks(&tasks))
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE,
                  "task creation failed, system stays stopped");
        log_flush();
        park();
    }

    if (!start_required_tasks(&tasks))
    {
        /* Tasks that already started wait on the startup barrier. */
        (void)tk_set_flg(g_startup.h_flg, STARTUP_FLAG_FAILED);
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE,
                  "task start failed, system stays stopped");
        log_flush();
        park();
    }

    start_optional_tasks(&tasks);

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
