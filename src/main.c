#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "demo_tasks.h"
#include "track/track.h"
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

/* Line following. 250 mm/s cruise and a 10 ms control period. Steering is a
 * filtered differential mixer: one sensor ramps the steer toward
 * +/-LINE_STEER_MAX_PERMILLE, both/neither decays it. The outer wheel
 * saturates at LINE_MAX_WHEEL_UM_S, so a tight curve reduces the average
 * speed instead of overdriving a wheel. Tune these on the car; the duty cap
 * in setup_motors() must leave headroom for the target. */
#define LINE_SPEED_UM_S          (250000U)
#define LINE_POLL_MS             (10U)
#define LINE_MAX_WHEEL_UM_S      (300000U)
#define LINE_STEER_MAX_PERMILLE  (1000)
#define LINE_STEER_SOFT_PERMILLE (250)
#define LINE_STEER_CONFIRM_TICKS (5U)
#define LINE_STEER_FILTER_SHIFT  (3U)
#define LINE_TURN_SPEED_UM_S     (180000)
#define LINE_TURN_90_MS          (600U)
#define LINE_TURN_180_MS         (1200U)

/* Startup barrier. This file creates every task; one startup task owns the
 * shared hardware init and the others wait for it. Subsystems never call
 * tk_cre_tsk() or tk_sta_tsk(). */
typedef struct
{
    bool volatile b_ready;
    bool volatile b_failed;
    ID            physics_tid;
} startup_state_t;

static startup_state_t g_startup = {false, false, 0};

ER setup_motors (void)
{
    motion_service_config_t config   = {0};
    config.control.sampling.wheel[0] = (motion_scale_t){204204U, 2530U, -1};
    config.control.sampling.wheel[1] = (motion_scale_t){204204U, 2530U, 1};
    config.control.sampling.max_interval_us = 100000U;
    for (uint8_t wheel = 0U; wheel < 2U; wheel++)
    {
        config.control.speed[wheel] = (pid_config_t){
            .kp_q16            = 256U,
            .ki_q16            = 256U,
            .kd_q16            = 2U,
            .max_target_um_s   = LINE_MAX_WHEEL_UM_S,
            .max_interval_us   = 100000U,
            .max_sample_age_us = 600000U,
            .max_duty_permille = 400U,
        };
    }
    config.control.max_command_ttl_us  = 200000U;
    config.control.max_control_gap_us  = 100000U;
    config.control.feedback_timeout_us = 2000000U;
    config.control.reversal_brake_us   = 100000U;
    config.poll_delay_ms               = 10U;

    return motion_service_init(&config);
}

/*!
 * @brief Wait until the startup task has finished. Returns false on a
 *        startup failure; the caller must then park.
 */
LOCAL bool wait_startup (void)
{
    while (!g_startup.b_ready)
    {
        if (g_startup.b_failed)
        {
            return false;
        }
        (void)tk_dly_tsk(10);
    }

    return true;
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
    ER result;

    (void)stacd;
    (void)exinf;

    result = gpio_irq_init();
    if (E_OK == result)
    {
        result = setup_motors();
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
        g_startup.b_ready = true;
    }
    else
    {
        tm_printf((UB *)"[init] startup failed: %d\n", result);
        g_startup.b_failed = true;
    }

    park();
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
static int32_t         g_line_steer_per_mille;
static uint8_t         g_line_error_ticks;

static void line_request_turn (line_turn_t turn)
{
    UINT saved;

    (void)ISpinLock(&g_turn_lock, &saved);
    g_turn_request = (uint8_t)turn;
    (void)ISpinUnlock(&g_turn_lock, saved);
}

static line_turn_t line_take_turn (void)
{
    UINT        saved;
    line_turn_t turn;

    (void)ISpinLock(&g_turn_lock, &saved);
    turn = (line_turn_t)g_turn_request;
    g_turn_request = (uint8_t)LINE_TURN_NONE;
    (void)ISpinUnlock(&g_turn_lock, saved);
    return turn;
}

/* One-shot pivot. The controller brakes a wheel that reverses, so the
 * duration includes margin; tune LINE_TURN_90_MS on the car. */
static void line_execute_turn (line_turn_t turn)
{
    int32_t  left;
    int32_t  right;
    uint32_t duration_ms;
    uint32_t elapsed_ms = 0U;
    uint32_t const lease_us = 10U * LINE_POLL_MS * 1000U;

    switch (turn)
    {
        case LINE_TURN_LEFT:
            left        = -(int32_t)LINE_TURN_SPEED_UM_S;
            right       = (int32_t)LINE_TURN_SPEED_UM_S;
            duration_ms = LINE_TURN_90_MS;
            break;
        case LINE_TURN_RIGHT:
            left        = (int32_t)LINE_TURN_SPEED_UM_S;
            right       = -(int32_t)LINE_TURN_SPEED_UM_S;
            duration_ms = LINE_TURN_90_MS;
            break;
        case LINE_TURN_UTURN:
            left        = (int32_t)LINE_TURN_SPEED_UM_S;
            right       = -(int32_t)LINE_TURN_SPEED_UM_S;
            duration_ms = LINE_TURN_180_MS;
            break;
        default:
            return;
    }

    while (elapsed_ms < duration_ms)
    {
        if (motion_set_speed(left, right, lease_us) != E_OK)
        {
            break;
        }
        (void)tk_dly_tsk(LINE_POLL_MS);
        elapsed_ms += LINE_POLL_MS;
    }
    (void)motion_set_speed(0, 0, lease_us);
    g_line_steer_per_mille = 0;
    g_line_error_ticks = 0U;
}

LOCAL void line_task (INT stacd, void * exinf)
{
    const uint32_t speed = LINE_SPEED_UM_S;
    /* Each speed lease must outlast one loop pass plus scheduling delay.
     * The motion task (priority 5) shares core 0 with physics (4) and the
     * obstacle task (6), so a 3-pass lease can be stale by the time the
     * controller validates it and latches MOTION_FAULT_COMMAND. Keep the
     * lease below max_command_ttl_us (200 ms). */
    const uint32_t lease_us = 10U * LINE_POLL_MS * 1000U;

    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    /* The hump detector needs 50 rest samples (1 s). Hold the drive until
     * it calibrates, or 2 s if the IMU is unavailable. */
    {
        uint32_t wait_ms = 0U;

        while (wait_ms < 2000U)
        {
            physics_status_t physics;

            if ((E_OK == physics_snapshot(&physics)) && physics.calibrated)
            {
                break;
            }
            (void)tk_dly_tsk(LINE_POLL_MS);
            wait_ms += LINE_POLL_MS;
        }
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
            /* obstacle is holding the car or driving round an obstacle. */
            g_line_steer_per_mille = 0;
            g_line_error_ticks = 0U;
        }
        else
        {
            int32_t target;
            int32_t delta;
            int32_t left;
            int32_t right;
            int32_t peak;

            if (state.left != state.right)
            {
                if (g_line_error_ticks < LINE_STEER_CONFIRM_TICKS)
                {
                    g_line_error_ticks++;
                }
            }
            else
            {
                g_line_error_ticks = 0U;
            }

            if (state.left && state.right)
            {
                target = 0;
            }
            else if (state.left)
            {
                target = (g_line_error_ticks >= LINE_STEER_CONFIRM_TICKS)
                             ? -LINE_STEER_MAX_PERMILLE
                             : -LINE_STEER_SOFT_PERMILLE;
            }
            else if (state.right)
            {
                target = (g_line_error_ticks >= LINE_STEER_CONFIRM_TICKS)
                             ? LINE_STEER_MAX_PERMILLE
                             : LINE_STEER_SOFT_PERMILLE;
            }
            else
            {
                target = 0;
            }

            g_line_steer_per_mille +=
                (target - g_line_steer_per_mille)
                / (1 << LINE_STEER_FILTER_SHIFT);

            delta = ((int32_t)speed * g_line_steer_per_mille) / 1000;
            left  = (int32_t)speed + delta;
            right = (int32_t)speed - delta;
            peak  = (left < 0) ? -left : left;
            if (((right < 0) ? -right : right) > peak)
            {
                peak = (right < 0) ? -right : right;
            }
            if (peak > (int32_t)LINE_MAX_WHEEL_UM_S)
            {
                left  = (int32_t)(((int64_t)left * LINE_MAX_WHEEL_UM_S)
                                  / peak);
                right = (int32_t)(((int64_t)right * LINE_MAX_WHEEL_UM_S)
                                  / peak);
            }
            (void)motion_set_speed(left, right, lease_us);
        }
        ir_wait_change(IR_TYPE_LINE, LINE_POLL_MS);
    }
}

/*!
 * @brief Publish one decoded barcode character as a telemetry event.
 */
LOCAL void barcode_command (uint8_t code)
{
    uint8_t         event[TELEMETRY_EVENT_WIRE_SIZE];
    static uint16_t event_id;

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

    (void)telemetry_pack_event(event, event_id, (uint8_t)'B', (int16_t)code,
                               0U);
    event_id++;
    (void)telemetry_queue_push(event, TELEMETRY_EVENT_WIRE_SIZE,
                               (uint8_t)'e', 1U);
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
        state = 2U;
    }
    else if ((MOTION_STATE_STOPPED == p_status->state) ||
             (MOTION_STATE_FAULT == p_status->state))
    {
        state = 3U;
    }
    else
    {
        /* IDLE. */
    }

    return (state);
}

/*!
 * @brief Build and queue a status packet every four passes.
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
        if (ticks >= 4U)
        {
            uint8_t           payload[TELEMETRY_STATUS_WIRE_SIZE];
            motion_status_t   status;
            motion_feedback_t feedback;
            int16_t                 speed_mms = 0;
            uint16_t                distance_mm = 0U;
            uint16_t                faults = system_fault_get();
            uint8_t                 heading_deg = 0U;
            uint8_t                 state = 0U;
            physics_status_t        physics;

            ticks = 0U;
            if (E_OK == motion_status(&status))
            {
                state = telemetry_state(&status);

                if (status.stop_requested)
                {
                    faults |= FAULT_ESTOP_LATCHED_BIT;
                }
            }

            if ((E_OK == motion_feedback_snapshot(&feedback)) && feedback.fresh)
            {
                int32_t const average_um_s =
                    (feedback.left_speed_um_s + feedback.right_speed_um_s) / 2;
                int64_t const average_um =
                    (feedback.left_displacement_um +
                     feedback.right_displacement_um) / 2;
                int64_t distance = average_um / 1000;

                speed_mms = (int16_t)(average_um_s / 1000);
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
            (void)telemetry_queue_push(payload, TELEMETRY_STATUS_WIRE_SIZE,
                                       (uint8_t)'s', 0U);
        }

        system_ticks++;
        if (system_ticks >= 10U)
        {
            uint8_t           system_payload[TELEMETRY_SYSTEM_WIRE_SIZE];
            obstacle_status_t obstacle;
            physics_status_t  physics;
            telemetry_system_t system_state = {0};

            system_ticks = 0U;

            if (E_OK == obstacle_snapshot(&obstacle))
            {
                system_state.obstacle_state  = obstacle.state;
                system_state.obstacle_cause  = obstacle.cause;
                system_state.obstacle_faults = obstacle.faults;
                system_state.echo_class      = obstacle.echo_class;
                system_state.obstacle_dist_mm = obstacle.distance_mm;
                system_state.obstacle_seq    = obstacle.seq;
                system_state.obstacle_req_id = obstacle.req_id;
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
                    system_state.physics_flags |= 0x01U;
                }
                if (physics.finished)
                {
                    system_state.physics_flags |= 0x02U;
                }
                if (physics.height_valid)
                {
                    system_state.physics_flags |= 0x04U;
                }
                if (physics.calibrated)
                {
                    system_state.physics_flags |= 0x08U;
                }
            }

            (void)telemetry_pack_system(system_payload, &system_state);
            (void)telemetry_queue_push(system_payload,
                                       TELEMETRY_SYSTEM_WIRE_SIZE,
                                       (uint8_t)'t', 0U);
        }

        (void)tk_dly_tsk(50);
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

LOCAL T_CTSK ctsk_blink = {
    .itskpri = 10,
    .stksz   = STACK_SZ,
    .task    = blink_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

LOCAL T_CTSK ctsk_startup = {
    .itskpri = 1,
    .stksz   = STACK_SZ,
    .task    = startup_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_motion = {
    .itskpri = 5,
    .stksz   = STACK_SZ,
    .task    = motion_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_physics = {
    .itskpri = 4,
    .stksz   = STACK_SZ,
    .task    = physics_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_obstacle = {
    .itskpri = 6,
    .stksz   = STACK_SZ,
    .task    = obstacle_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_line = {
    .itskpri = 9,
    .stksz   = STACK_SZ,
    .task    = line_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_telemetry = {
    .itskpri = 7,
    .stksz   = STACK_SZ,
    .task    = telemetry_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

LOCAL T_CTSK ctsk_connection = {
    .itskpri = 6,
    .stksz   = STACK_SZ,
    .task    = connection_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_barcode = {
    .itskpri = 7,
    .stksz   = STACK_SZ,
    .task    = barcode_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or print failure and keep going. */
LOCAL BOOL made (const char * what, ID id)
{
    if (id <= E_OK)
    {
        tm_printf((UB *)"[init] tk_cre_%s failed, er=%d\n", what, id);
        return FALSE;
    }
    return TRUE;
}

EXPORT INT usermain (void)
{
    bool b_ok = true;
    bool b_telemetry;
    bool b_connection;
    ER   result;
    ID   startup_tid;
    ID   motion_tid;
    ID   physics_tid;
    ID   obstacle_tid;
    ID   line_tid;
    ID   blink_tid;
    ID   telemetry_tid = 0;
    ID   connection_tid = 0;
    ID   barcode_tid = 0;
    bool b_barcode;

    /* Telemetry is optional: the car still runs when the queues or the
     * network initialisation fail. */
    result = telemetry_queue_create();
    b_telemetry = (E_OK == result);
    result = telemetry_net_queue_create();
    b_telemetry = b_telemetry && (E_OK == result);
    if (!b_telemetry)
    {
        tm_printf((UB *)"[init] telemetry queues failed: %d\n", result);
    }

    (void)InitSpinLock(&g_turn_lock);
    barcode_set_handler(barcode_command);

    result = connection_init(command_dispatch, NULL);
    b_connection = (E_OK == result);
    if (!b_connection)
    {
        tm_printf((UB *)"[init] connection_init failed: %d\n", result);
    }

    /* D62/D70: this composition root creates every task. Subsystems only
     * prepare their resources and expose a task body. */
    startup_tid = tk_cre_tsk(&ctsk_startup);
    if (!made("tsk(startup)", startup_tid))
    {
        b_ok = false;
    }

    motion_tid = tk_cre_tsk(&ctsk_motion);
    if (!made("tsk(motion)", motion_tid))
    {
        b_ok = false;
    }

    physics_tid = tk_cre_tsk(&ctsk_physics);
    if (!made("tsk(physics)", physics_tid))
    {
        b_ok = false;
    }
    else
    {
        g_startup.physics_tid = physics_tid;
    }

    obstacle_tid = tk_cre_tsk(&ctsk_obstacle);
    if (!made("tsk(obstacle)", obstacle_tid))
    {
        b_ok = false;
    }

    line_tid = tk_cre_tsk(&ctsk_line);
    if (!made("tsk(line)", line_tid))
    {
        b_ok = false;
    }

    blink_tid = tk_cre_tsk(&ctsk_blink);
    if (!made("tsk(blink)", blink_tid))
    {
        b_ok = false;
    }

    if (b_telemetry)
    {
        telemetry_tid = tk_cre_tsk(&ctsk_telemetry);
        if (!made("tsk(telemetry)", telemetry_tid))
        {
            b_telemetry = false;
        }
    }

    barcode_tid = tk_cre_tsk(&ctsk_barcode);
    b_barcode   = made("tsk(barcode)", barcode_tid);

    if (b_connection)
    {
        connection_tid = tk_cre_tsk(&ctsk_connection);
        if (!made("tsk(connection)", connection_tid))
        {
            b_connection = false;
        }
    }

    if (!b_ok)
    {
        tm_printf((UB *)"[init] task creation failed, system stays stopped\n");
        park();
    }

    tk_sta_tsk(startup_tid, 0);
    tk_sta_tsk(motion_tid, 0);
    tk_sta_tsk(physics_tid, 0);
    tk_sta_tsk(obstacle_tid, 0);
    tk_sta_tsk(line_tid, 0);
    tk_sta_tsk(blink_tid, 0);

    if (b_telemetry)
    {
        tk_sta_tsk(telemetry_tid, 0);
    }
    if (b_connection)
    {
        tk_sta_tsk(connection_tid, 0);
    }
    if (b_barcode)
    {
        tk_sta_tsk(barcode_tid, 0);
    }

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
