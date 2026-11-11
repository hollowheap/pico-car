#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "track/track.h"
#include "track/line_control.h"
#include "motion/motion.h"
#include "motion/motion_request.h"
#include "motion/motion_service.h"
#include "motion/encoder.h"
#include "navigation/obstacle.h"
#include "track/barcode.h"
#include "physics/physics.h"
#include "platform/faults.h"
#include "platform/isr.h"
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
 * setup_motors() must leave headroom for them. A one-sensor correction stops
 * the inner wheel; one-shot turns pivot in place. LINE_POLL_MS is the control
 * period. */
#define LINE_POLL_MS             (10U)
#define LINE_MAX_WHEEL_UM_S      (300000U)
/* Upper bound on one turn, so a blocked car cannot pivot for ever. Tune on
 * the car: it must exceed the longest U-turn, including the slow phase. */
#define LINE_TURN_MAX_MS         (4000U)

/* Startup barrier event flag bits. */
#define STARTUP_FLAG_READY           (0x01U)
#define STARTUP_FLAG_FAILED          (0x02U)

/* Motor service configuration (setup_motors). */
#define MOTOR_WHEEL_COUNT            (2U)
#define MOTOR_WHEEL_TRAVEL_UM        (204204U)
#define MOTOR_EDGES_PER_REVOLUTION   (2530U)
#define MOTOR_SAMPLING_INTERVAL_US   (100000U)
#define MOTOR_PID_KP_Q16             (256U)
#define MOTOR_PID_KI_Q16             (256U)
#define MOTOR_PID_KD_Q16             (2U)
#define MOTOR_PID_INTERVAL_US        (100000U)
#define MOTOR_PID_SAMPLE_AGE_US      (600000U)
#define MOTOR_PID_DUTY_PERMILLE      (400U)
#define MOTOR_COMMAND_TTL_US         (200000U)
#define MOTOR_CONTROL_GAP_US         (100000U)
#define MOTOR_FEEDBACK_TIMEOUT_US    (2000000U)
#define MOTOR_REVERSAL_BRAKE_US      (100000U)
#define MOTOR_POLL_DELAY_MS          (10U)

/* Telemetry pacing, in 50 ms passes of telemetry_task. */
#define TELEMETRY_PASS_MS            (50U)
#define TELEMETRY_STATUS_PASSES      (4U)
#define TELEMETRY_SYSTEM_PASSES      (10U)
#define TELEMETRY_SLEEP_TIMEOUT_MS   (250U)

/* Dashboard state values (the dashboard schema owns these). */
#define TELEMETRY_STATE_RUNNING      (2U)
#define TELEMETRY_STATE_STOPPED      (3U)

/* Units per millimetre for the status packet. */
#define TELEMETRY_UM_PER_MM          (1000)

/* physics_flags bits of the system packet. */
#define TELEMETRY_PHYSICS_ACTIVE     (0x01U)
#define TELEMETRY_PHYSICS_FINISHED   (0x02U)
#define TELEMETRY_PHYSICS_HEIGHT_OK  (0x04U)
#define TELEMETRY_PHYSICS_CALIBRATED (0x08U)

/* Startup barrier. This file creates every task; one startup task owns the
 * shared hardware init and the others wait for it. Subsystems never call
 * tk_cre_tsk() or tk_sta_tsk(). */
typedef struct
{
    ID h_flg;
    ID motion_tid;
    ID physics_tid;
    ID telemetry_tid;
} startup_state_t;

static startup_state_t g_startup = {0, 0, 0, 0};
static ID              g_telemetry_cyc_id;
static uint32_t        g_telemetry_overruns;

LOCAL void telemetry_cyclic_wakeup (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_startup.telemetry_tid);
}

LOCAL ER setup_motors (ID motion_tid)
{
    motion_service_config_t config   = {0};
    config.control.sampling.wheel[0] =
        (motion_scale_t){MOTOR_WHEEL_TRAVEL_UM, MOTOR_EDGES_PER_REVOLUTION,
                         -1};
    config.control.sampling.wheel[1] =
        (motion_scale_t){MOTOR_WHEEL_TRAVEL_UM, MOTOR_EDGES_PER_REVOLUTION,
                         1};
    config.control.sampling.max_interval_us = MOTOR_SAMPLING_INTERVAL_US;
    for (uint8_t wheel = 0U; wheel < MOTOR_WHEEL_COUNT; wheel++)
    {
        config.control.speed[wheel] = (pid_config_t){
            .kp_q16            = MOTOR_PID_KP_Q16,
            .ki_q16            = MOTOR_PID_KI_Q16,
            .kd_q16            = MOTOR_PID_KD_Q16,
            .max_target_um_s   = LINE_MAX_WHEEL_UM_S,
            .max_interval_us   = MOTOR_PID_INTERVAL_US,
            .max_sample_age_us = MOTOR_PID_SAMPLE_AGE_US,
            .max_duty_permille = MOTOR_PID_DUTY_PERMILLE,
        };
    }
    config.control.max_command_ttl_us  = MOTOR_COMMAND_TTL_US;
    config.control.max_control_gap_us  = MOTOR_CONTROL_GAP_US;
    config.control.feedback_timeout_us = MOTOR_FEEDBACK_TIMEOUT_US;
    config.control.reversal_brake_us   = MOTOR_REVERSAL_BRAKE_US;
    config.poll_delay_ms               = MOTOR_POLL_DELAY_MS;

    return motion_service_init(&config, motion_tid);
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
        result = setup_motors(g_startup.motion_tid);
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
        tm_printf((UB *)"[init] startup failed: %d\n", result);
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
    LINE_TURN_UTURN
} line_turn_t;

static T_SPLOCK        g_turn_lock;
static volatile uint8_t g_turn_request;

static void line_request_turn (line_turn_t turn)
{
    UINT saved = 0U;

    (void)ISpinLock(&g_turn_lock, &saved);
    g_turn_request = (uint8_t)turn;
    (void)ISpinUnlock(&g_turn_lock, saved);
}

static line_turn_t line_take_turn (void)
{
    UINT        saved = 0U;
    line_turn_t turn = LINE_TURN_NONE;

    (void)ISpinLock(&g_turn_lock, &saved);
    turn = (line_turn_t)g_turn_request;
    g_turn_request = (uint8_t)LINE_TURN_NONE;
    (void)ISpinUnlock(&g_turn_lock, saved);
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

/* Set FAULT_LINE_LOST_BIT on entering line lost, clear it on leaving. */
static void line_update_fault (bool b_lost)
{
    if (b_lost && !g_line_lost)
    {
        system_fault_set(FAULT_LINE_LOST_BIT);
    }
    else if (!b_lost && g_line_lost)
    {
        system_fault_clear(FAULT_LINE_LOST_BIT);
    }
    else
    {
        /* No change. */
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

    if (!line_turn_kind(turn, &kind))
    {
        return;
    }

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
    (void)motion_set_speed(0, 0, LINE_LEASE_US);

    if (LINE_TURN_DONE == status)
    {
        line_follow_set_side(&g_line_follow, line_turn_side(&state));
        line_update_fault(false);
    }
    else
    {
        /* Stopped above. The follower holds the car until a sensor sees the
         * line, and the fault stays set until then. */
        line_follow_hold_lost(&g_line_follow);
        line_update_fault(true);
    }
}

/*!
 * @brief One pass of the line follower (D92). One sensor on the line stops
 *        the inner wheel and records its side; with neither sensor the
 *        correction continues toward the recorded side until the search
 *        limit; both sensors drive straight.
 */
LOCAL void line_follow_step (ir_sensors_state_t state)
{
    line_targets_t       targets = {0, 0};
    motion_feedback_t    feedback;
    line_follow_status_t status = LINE_FOLLOW_FOLLOWING;

    if (E_OK == motion_feedback_snapshot(&feedback))
    {
        g_line_travel_um = (feedback.left_displacement_um +
                            feedback.right_displacement_um) /
                           LINE_WHEEL_COUNT;
    }

    status = line_follow_update(&g_line_follow, state.left, state.right,
                                g_line_travel_um, line_now_ms(), &targets);
    if (LINE_FOLLOW_LOST == status)
    {
        line_update_fault(true);
    }
    else if (state.left || state.right)
    {
        /* The next sensor hit clears line lost, also after an aborted turn. */
        line_update_fault(false);
    }
    else
    {
        /* Still searching or driving blind: keep the fault as it is. */
    }
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
        line_turn_t const turn = line_take_turn();

        if (LINE_TURN_NONE != turn)
        {
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

static uint16_t g_event_id;

/*!
 * @brief Publish one decoded barcode character as a telemetry event.
 */
LOCAL void barcode_command (uint8_t code)
{
    uint8_t         event[TELEMETRY_EVENT_WIRE_SIZE];

    switch (code)
    {
        case ('L' - 'A'):
            line_request_turn(LINE_TURN_LEFT);
            break;
        case ('R' - 'A'):
            line_request_turn(LINE_TURN_RIGHT);
            break;
        case ('U' - 'A'):
            line_request_turn(LINE_TURN_UTURN);
            break;
        default:
            break;
    }

    (void)telemetry_pack_event(event, g_event_id, (uint8_t)'B', (int16_t)code,
                               0U);
    g_event_id++;
    (void)telemetry_queue_push(event, TELEMETRY_EVENT_QUEUED_SIZE, 1U);
    tm_printf((UB *)"[barcode] %c\n", (INT)(code + (uint8_t)'A'));
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

/*!
 * @brief Map the motion state to the dashboard state value.
 *
 * 0 INIT, 2 RUNNING, 3 STOPPED. The dashboard schema owns these values.
 */
LOCAL uint8_t telemetry_state (motion_status_t const * p_status)
{
    uint8_t state = 0U;

    if ((MOTION_STATE_RUNNING == p_status->state) ||
        (MOTION_STATE_REVERSING == p_status->state))
    {
        state = TELEMETRY_STATE_RUNNING;
    }
    else if ((MOTION_STATE_STOPPED == p_status->state) ||
             (MOTION_STATE_FAULT == p_status->state))
    {
        state = TELEMETRY_STATE_STOPPED;
    }
    else
    {
        /* IDLE. */
    }

    return (state);
}

/*!
 * @brief Build and queue one status packet.
 */
LOCAL void telemetry_send_status (void)
{
    uint8_t           payload[TELEMETRY_STATUS_WIRE_SIZE];
    motion_status_t   status;
    motion_feedback_t feedback;
    int16_t           speed_mms = 0;
    uint16_t          distance_mm = 0U;
    uint16_t          faults = system_fault_get();
    uint8_t           heading_deg = 0U;
    uint8_t           state = 0U;
    physics_status_t  physics;

    if (E_OK == motion_status(&status))
    {
        state = telemetry_state(&status);

        if (status.stop_requested)
        {
            faults |= (uint16_t)FAULT_ESTOP_LATCHED_BIT;
        }
    }

    if ((E_OK == motion_feedback_snapshot(&feedback)) && feedback.fresh)
    {
        int32_t const average_um_s =
            (feedback.left_speed_um_s + feedback.right_speed_um_s) / 2;
        int64_t const average_um =
            (feedback.left_displacement_um +
             feedback.right_displacement_um) / 2;
        int64_t distance = average_um / TELEMETRY_UM_PER_MM;

        speed_mms = (int16_t)(average_um_s / TELEMETRY_UM_PER_MM);
        if (distance < 0)
        {
            distance = 0;
        }
        else if (distance > UINT16_MAX)
        {
            distance = UINT16_MAX;
        }
        else
        {
            /* In range. */
        }
        distance_mm = (uint16_t)distance;
    }

    if ((E_OK == physics_snapshot(&physics)) && physics.heading_valid)
    {
        heading_deg = physics.heading_deg;
    }

    (void)telemetry_pack_status(payload, speed_mms, distance_mm,
                                heading_deg, state, faults);
    (void)telemetry_queue_push(payload, TELEMETRY_STATUS_QUEUED_SIZE, 0U);
}

/*!
 * @brief Build and queue one system packet.
 */
LOCAL void telemetry_send_system (void)
{
    uint8_t            system_payload[TELEMETRY_SYSTEM_WIRE_SIZE];
    obstacle_status_t  obstacle;
    physics_status_t   physics;
    telemetry_system_t system_state = {0};

    if (E_OK == obstacle_snapshot(&obstacle))
    {
        system_state.obstacle_state   = obstacle.state;
        system_state.obstacle_cause   = obstacle.cause;
        system_state.obstacle_faults  = obstacle.faults;
        system_state.echo_class       = obstacle.echo_class;
        system_state.obstacle_dist_mm = obstacle.distance_mm;
        system_state.obstacle_seq     = obstacle.seq;
        system_state.obstacle_req_id  = obstacle.req_id;
    }

    if (E_OK == physics_snapshot(&physics))
    {
        system_state.physics_state  = physics.state;
        system_state.physics_motion = physics.motion;
        system_state.physics_faults = physics.faults;
        system_state.pitch_cdeg     = physics.pitch_cdeg;
        system_state.hump_peak_mm   = physics.hump_peak_mm;

        if (physics.hump_active)
        {
            system_state.physics_flags |= TELEMETRY_PHYSICS_ACTIVE;
        }
        if (physics.finished)
        {
            system_state.physics_flags |= TELEMETRY_PHYSICS_FINISHED;
        }
        if (physics.height_valid)
        {
            system_state.physics_flags |= TELEMETRY_PHYSICS_HEIGHT_OK;
        }
        if (physics.calibrated)
        {
            system_state.physics_flags |= TELEMETRY_PHYSICS_CALIBRATED;
        }
    }

    (void)telemetry_pack_system(system_payload, &system_state);
    (void)telemetry_queue_push(system_payload,
                               TELEMETRY_SYSTEM_QUEUED_SIZE, 0U);
}

/*!
 * @brief Build and queue a status packet every four passes and a system
 *        packet every ten passes.
 */
LOCAL void telemetry_task (INT stacd, void * exinf)
{
    uint32_t ticks = 0U;
    uint32_t system_ticks = 0U;

    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    for (;;)
    {
        telemetry_worker_run(telemetry_queue_pop, net_sink, TMO_POL);

        ticks++;
        if (ticks >= TELEMETRY_STATUS_PASSES)
        {
            ticks = 0U;
            telemetry_send_status();
        }

        system_ticks++;
        if (system_ticks >= TELEMETRY_SYSTEM_PASSES)
        {
            system_ticks = 0U;
            telemetry_send_system();
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
 * @brief Join telemetry commands to motion and obstacle.
 *
 * NOTE: STOP uses the direct motion emergency latch (fault bit 9) and also
 * NOTE: STOP is a normal operator hold and can be released. ESTOP uses the
 * direct emergency latch (fault bit 9) and also stops obstacle. D75: the
 * latch needs a reboot; the dashboard disables Reset while bit 9 is set.
 */
LOCAL void command_dispatch (const telemetry_cmd_t * cmd, void * context)
{
    (void)context;

    switch (cmd->opcode)
    {
        case (uint8_t)TELEMETRY_CMD_STOP:
            line_request_turn(LINE_TURN_NONE);
            (void)motion_set_hold(MOTION_HOLD_OPERATOR, true);
            (void)obstacle_post_cmd(OBSTACLE_CMD_STOP);
        break;

        case (uint8_t)TELEMETRY_CMD_ESTOP:
            line_request_turn(LINE_TURN_NONE);
            (void)motion_request_stop();
            (void)obstacle_post_cmd(OBSTACLE_CMD_STOP);
        break;

        case (uint8_t)TELEMETRY_CMD_START:
            (void)motion_set_hold(MOTION_HOLD_OPERATOR, false);
            (void)obstacle_post_cmd(OBSTACLE_CMD_RESET);
            (void)obstacle_post_cmd(OBSTACLE_CMD_START);
        break;

        case (uint8_t)TELEMETRY_CMD_RESET:
            (void)motion_set_hold(MOTION_HOLD_OPERATOR, false);
            (void)obstacle_post_cmd(OBSTACLE_CMD_RESET);
        break;

        case (uint8_t)TELEMETRY_CMD_LEFT:
            line_request_turn(LINE_TURN_LEFT);
        break;

        case (uint8_t)TELEMETRY_CMD_RIGHT:
            line_request_turn(LINE_TURN_RIGHT);
        break;

        case (uint8_t)TELEMETRY_CMD_UTURN:
            line_request_turn(LINE_TURN_UTURN);
        break;

        default:
            /* Unknown opcode: ignore. */
        break;
    }
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
        tm_printf((UB *)"[init] tk_cre_%s failed, er=%d\n", what, id);
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
        tm_printf((UB *)"[init] tk_sta_%s failed, er=%d\n", what, result);
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
        tm_printf((UB *)"[init] telemetry queues failed: %d\n", result);
    }

    (void)InitSpinLock(&g_turn_lock);
    barcode_set_handler(barcode_command);

    result = connection_init(command_dispatch, NULL);
    p_tasks->b_connection = (E_OK == result);
    if (!p_tasks->b_connection)
    {
        tm_printf((UB *)"[init] connection_init failed: %d\n", result);
    }
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

    cflg.flgatr     = TA_WMUL | TA_TFIFO;
    cflg.iflgptn    = 0U;
    g_startup.h_flg = tk_cre_flg(&cflg);

    init_services(&tasks);

    if (!create_tasks(&tasks))
    {
        tm_printf((UB *)"[init] task creation failed, system stays stopped\n");
        park();
    }

    if (!start_required_tasks(&tasks))
    {
        /* Tasks that already started wait on the startup barrier. */
        (void)tk_set_flg(g_startup.h_flg, STARTUP_FLAG_FAILED);
        tm_printf((UB *)"[init] task start failed, system stays stopped\n");
        park();
    }

    start_optional_tasks(&tasks);

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
