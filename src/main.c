#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "track/track.h"
#include "track/line_task.h"
#include "motion/motion.h"
#include "motion/motion_request.h"
#include "motion/motion_service.h"
#include "motion/motor_setup.h"
#include "motion/encoder.h"
#include "motion/pwm.h"
#include "navigation/obstacle.h"
#include "track/barcode.h"
#include "physics/physics.h"
#include "platform/faults.h"
#include "platform/isr.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/static_assert.h"
#include "supervisor/edge_events.h"
#include "supervisor/supervisor.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/hug_word.h"
#include "telemetry/cmd.h"
#include "telemetry/connection.h"
#include "telemetry/net_queue.h"
#include "telemetry/packet.h"
#include "telemetry/queue.h"
#include "telemetry/worker.h"

/* Task stacks. Each is the deepest call chain of the task, from the GCC
 * call graph (-fstack-usage -fcallgraph-info=su, function pointers followed
 * by hand), plus a quarter, plus 1152 bytes for what lands on top of it.
 * The chain includes the SMP kernel path, which runs the ready-queue
 * assignment of each critical section on the calling task's stack. On top:
 * three nested interrupt levels (SysTick 240 B, GPIO 344 B, I2C or UART
 * 288 B, a 32-byte frame each) and the dispatcher's saved registers. Physics
 * adds 400 bytes for newlib's sinf, atan2f and acosf, which the call graph
 * does not see. Rounded up to 256 bytes. Deepest chain per task: startup
 * 672, motion 984, physics 880, supervisor 1016, obstacle 568, line 936,
 * telemetry 1208, connection 680, barcode 808. */
#define STACK_STARTUP_SZ    (2048U)
#define STACK_MOTION_SZ     (2560U)
#define STACK_PHYSICS_SZ    (2816U)
#define STACK_SUPERVISOR_SZ (2560U)
#define STACK_OBSTACLE_SZ   (2048U)
#define STACK_LINE_SZ       (2560U)
#define STACK_TELEMETRY_SZ  (2816U)
#define STACK_CONNECTION_SZ (2048U)
#define STACK_BARCODE_SZ    (2304U)
#define STACK_ALIGN 8U


/* Width of the fault fields on the wire (bits 0..15, see platform/faults.h).
   A seventeenth fault bit needs wider status, diag and event fields. */
#define FAULT_WORD_BITS              (16U)
STATIC_ASSERT(FAULT_ALL_MASK <= UINT16_MAX, "fault bits fit the wire fields");

/* Kernel objects the firmware creates, against the limits in
   the pinned kernel config/config.h. Tasks: the nine below, plus the
   kernel's init, USB console and CYW43 tasks. Cyclic handlers: supervisor,
   motion and physics. Past a limit, tk_cre_* fails
   at boot with E_LIMIT. */
#define FIRMWARE_TASKS   (12U)
#define FIRMWARE_CYCLICS (3U)
STATIC_ASSERT(FIRMWARE_TASKS <= CNF_MAX_TSKID, "task limit");
STATIC_ASSERT(FIRMWARE_CYCLICS <= CNF_MAX_CYCID, "cyclic handler limit");

/* Startup barrier event flag bits. */
#define STARTUP_FLAG_READY           (0x01U)
#define STARTUP_FLAG_FAILED          (0x02U)

/* GP20 is the active-low operator START input in the main firmware. */
#define START_BUTTON_PIN             (20U)
#define START_BUTTON_PULL_UP         (1UL << 3U)
#define START_BUTTON_PULL_DOWN       (1UL << 2U)
#define START_BUTTON_SCHMITT         (1UL << 1U)

/* Periodic snapshots use a 50 ms timebase. Queue arrivals wake telemetry immediately. */
#define TELEMETRY_PASS_MS            (50U)
#define TELEMETRY_FAST_PASSES        (4U)
#define TELEMETRY_OBSTACLE_PASSES    (10U)
#define TELEMETRY_DIAG_PASSES        (20U)


/* Telemetry reads the obstacle profile with the wire's slot count. */
STATIC_ASSERT (OBSTACLE_PROFILE_SLOTS == TELEMETRY_PROFILE_COUNT_SLOTS,
               "profile slots match the wire");
STATIC_ASSERT (OBSTACLE_ECHO_CLASSES == TELEMETRY_DIAG_COUNTS_COUNT_ECHO_TOTAL,
               "echo counters match the wire");

/* Startup barrier. This file creates every task; one startup task owns the
 * shared hardware init and the others wait for it. Subsystems never call
 * tk_cre_tsk() or tk_sta_tsk(). */
typedef struct
{
    ID h_flg;
    ID motion_tid;
    ID physics_tid;
    ID connection_tid;
    ID supervisor_tid;
} startup_state_t;

static startup_state_t g_startup = {0, 0, 0, 0, 0};
static uint32_t        g_telemetry_overruns;

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

LOCAL void line_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    line_task_run();
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

LOCAL void telemetry_sonar_aim (int16_t commanded_deci)
{
    (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_OBSTACLE,
                                   (uint8_t)TELEMETRY_EVT_SONAR_AIM,
                                   commanded_deci, 0U, current_odometer_mm());
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
    if ((E_OK == telemetry_net_queue_push(frame, len)) && (g_startup.connection_tid > 0))
    {
        (void)tk_wup_tsk(g_startup.connection_tid);
    }
    return 0;
}

/* Only the telemetry task advances the status frame counter. */
static uint16_t g_status_seq;

LOCAL uint16_t clamp_u16 (uint32_t value)
{
    return (value > UINT16_MAX) ? (uint16_t)UINT16_MAX : (uint16_t)value;
}

LOCAL uint8_t current_line_bits (void)
{
    ir_sensors_state_t const ir   = ir_get_state();
    uint8_t                  bits = barcode_status_bits();

    if (ir.left)
    {
        bits |= (uint8_t)(1U << TELEMETRY_LINE_BITS_BIT_LEFT);
    }
    if (ir.right)
    {
        bits |= (uint8_t)(1U << TELEMETRY_LINE_BITS_BIT_RIGHT);
    }
    if (ir.barcode)
    {
        bits |= (uint8_t)(1U << TELEMETRY_LINE_BITS_BIT_BARCODE);
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
    bool const           b_motion   = (E_OK == motion_snapshot(&motion, &feedback, &diag));
    bool const           b_feedback = b_motion;
    bool const           b_diag     = b_motion;
    bool const           b_physics  = (E_OK == physics_snapshot(&physics));
    bool const           b_obstacle = (E_OK == obstacle_snapshot(&obstacle));
    line_task_stats_t    line;

    uint32_t hug[HUG_WORD_COUNT];
    line_task_hug(hug);
    line_task_stats(&line);
    telemetry_assemble_status(&status, b_motion ? &motion : NULL,
                              b_feedback ? &feedback : NULL,
                              b_physics ? &physics : NULL,
                              b_obstacle ? &obstacle : NULL, (uint16_t)system_fault_get(),
                              g_status_seq, current_line_bits(), line.b_turning, hug);
    g_status_seq++;
    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_STATUS, packet,
        telemetry_pack_status(packet, &status), 0U);

    telemetry_assemble_motion(&moving, b_motion ? &motion : NULL,
                              b_feedback ? &feedback : NULL,
                              b_diag ? &diag : NULL);
    /* Cast is safe: the frame keeps the low byte to pair with status. */
    moving.seq = (uint8_t)status.seq;
    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_MOTION, packet,
        telemetry_pack_motion(packet, &moving), 0U);

    telemetry_assemble_physics(&imu, b_physics ? &physics : NULL);
    imu.seq = (uint8_t)status.seq;
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
    uint32_t                hug[HUG_WORD_COUNT];
    bool const              b_status = (E_OK == obstacle_snapshot(&obstacle));
    bool const              b_plan   = (E_OK == obstacle_copy_plan(&plan));
    uint32_t const          ping     = obstacle_ping_read();

    line_task_hug(hug);
    telemetry_assemble_obstacle(&frame, b_status ? &obstacle : NULL,
                                b_plan ? &plan : NULL, ping, hug);
    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_OBSTACLE, packet,
        telemetry_pack_obstacle(packet, &frame), 0U);
}

/* Smallest free stack seen by stack_report (low 16 bits) and its task (bits
 * 16 to 23), for the diag packet. One word, written by the supervisor task
 * with one store and read once by the telemetry task, so a mark is never
 * paired with the task of another scan. */
#define STACK_MARK_FREE_MASK  (0xFFFFU)
#define STACK_MARK_TASK_SHIFT (16U)
#define STACK_MARK_TASK_MASK  (0xFFU)
static volatile uint32_t g_stack_mark = STACK_MARK_FREE_MASK;

/*!
 * @brief Queue the diag frame (1 Hz): counters and health.
 */
LOCAL void telemetry_send_diag (void)
{
    uint8_t           packet[TELEMETRY_WIRE_MAX];
    physics_status_t  physics;
    telemetry_diag_t  diag = {0};
    barcode_stats_t   bstats;
    line_task_stats_t lstats;
    uint32_t const    stack_mark = g_stack_mark;
    telemetry_cmd_state_t cmd_state;

    barcode_stats(&bstats);
    line_task_stats(&lstats);
    connection_cmd_state(&cmd_state);
    diag.sticky_faults       = (uint16_t)system_fault_sticky_get();
    diag.line_lost_count     = lstats.lost_count;
    diag.barcode_ok          = bstats.ok;
    diag.barcode_bad         = bstats.bad;
    diag.barcode_edge_drops  = clamp_u16(ir_barcode_drops());
    diag.last_barcode        = bstats.last_letter;
    diag.last_barcode_action = bstats.last_action;
    diag.task_alive_mask     = supervisor_alive_mask();
    diag.supervisor_passes   = supervisor_pass_count();
    diag.stack_min_free      = (uint16_t)(stack_mark & STACK_MARK_FREE_MASK);
    diag.stack_min_task      = (uint8_t)((stack_mark >> STACK_MARK_TASK_SHIFT) &
                                         STACK_MARK_TASK_MASK);
    /* Low byte on purpose: the wire field wraps and a client takes the
     * difference between frames (protocol/telemetry.yaml). */
    diag.cmd_ring_drops      = (uint8_t)supervisor_cmd_drops();
    diag.cmd_last_id         = cmd_state.last_cmd_id;
    diag.cmd_rej_length      = cmd_state.rejects.length;
    diag.cmd_rej_opcode      = cmd_state.rejects.opcode;
    diag.cmd_rej_run         = cmd_state.rejects.run;
    diag.cmd_rej_old_id      = cmd_state.rejects.old_id;
    diag.cmd_rej_age         = cmd_state.rejects.age;
    diag.telem_queue_drops   = clamp_u16((uint32_t)telemetry_queue_drop_count());
    diag.net_queue_drops     = clamp_u16((uint32_t)telemetry_net_queue_drop_count());
    diag.telemetry_overruns  = clamp_u16(g_telemetry_overruns);
    if (E_OK == physics_snapshot(&physics))
    {
        diag.physics_overruns = clamp_u16(physics.overruns);
    }
    diag.line_edges_total     = ir_line_edges();
    diag.line_raw_edges_total = ir_line_raw_edges();
    obstacle_echo_counts(diag.echo_total);
    diag.sensor_sample_us = in_w(TIMER_TIMERAWL);

    (void)telemetry_queue_push_packet(
        (uint8_t)TELEMETRY_TOPIC_DIAG, packet,
        telemetry_pack_diag(packet, &diag), 0U);
}

/*!
 * @brief Offer one event per changed bit without delaying the fault owner.
 */
LOCAL void telemetry_fault_changed (uint32_t changed, uint32_t level)
{
    uint32_t const odometer = current_odometer_mm();

    for (uint16_t bit = 0U; bit < FAULT_WORD_BITS; bit++)
    {
        uint32_t const mask = 1U << bit;

        if (0U != (changed & mask))
        {
            (void)telemetry_try_emit_event(
                (uint8_t)TELEMETRY_EVENT_SOURCE_TELEMETRY,
                (0U != (level & mask)) ? (uint8_t)TELEMETRY_EVT_FAULT_SET
                                      : (uint8_t)TELEMETRY_EVT_FAULT_CLEAR,
                (int16_t)bit, (uint16_t)level, odometer);
        }
    }
}

/*!
 * @brief Forward arriving events and check periodic deadlines between bursts.
 */
LOCAL void telemetry_task (INT stacd, void * exinf)
{
    SYSTIM now = {0};
    uint32_t pass = 0U;

    (void)stacd;
    (void)exinf;
    if (!wait_startup())
    {
        park();
    }

    (void)tk_get_otm(&now);
    uint32_t last_ms = now.lo;
    UW       last_ack_overflow = 0U;
    for (;;)
    {
        liveness_kick(LIVE_TELEMETRY);
        (void)telemetry_drain_cmd_acks();
        UW const ack_overflow = telemetry_cmd_ack_overflow_count();
        if (ack_overflow != last_ack_overflow)
        {
            LOG_WARN(LOG_TASK_TELEMETRY, LOG_MOD_NONE,
                     "cmd ack handoff overflow=%u", (unsigned)ack_overflow);
            last_ack_overflow = ack_overflow;
        }
        (void)tk_get_otm(&now);
        uint32_t const elapsed = now.lo - last_ms;
        uint32_t const ticks = elapsed / TELEMETRY_PASS_MS;
        if (ticks != 0U)
        {
            g_telemetry_overruns += ticks - 1U;
            if (ticks >= TELEMETRY_FAST_PASSES - pass % TELEMETRY_FAST_PASSES)
            {
                telemetry_send_fast();
            }
            if (ticks >= TELEMETRY_OBSTACLE_PASSES - pass % TELEMETRY_OBSTACLE_PASSES)
            {
                telemetry_send_obstacle();
            }
            if (ticks >= TELEMETRY_DIAG_PASSES - pass % TELEMETRY_DIAG_PASSES)
            {
                telemetry_send_diag();
            }
            pass = (pass + ticks) % TELEMETRY_DIAG_PASSES;
            last_ms += ticks * TELEMETRY_PASS_MS;
            log_drain();
            (void)telemetry_drain_cmd_acks();
        }
        (void)tk_get_otm(&now);
        uint32_t const used_ms = now.lo - last_ms;
        TMO const wait = (used_ms < TELEMETRY_PASS_MS) ? (TMO)(TELEMETRY_PASS_MS - used_ms) : TMO_POL;
        telemetry_worker_run(telemetry_queue_pop, net_sink, wait);
    }
}

/*!
 * @brief Hand a telemetry command to the supervisor.
 *
 * The supervisor decides. The emergency stop also acts here, at once: it
 * never waits for the next supervisor pass. The latch needs a reboot.
 */
LOCAL void command_dispatch (const telemetry_cmd_t * cmd, void * context)
{
    (void)context;

    if ((uint8_t)TELEMETRY_CMD_ESTOP == cmd->opcode)
    {
        (void)motion_request_stop();
    }
    (void)vehicle_command_post_ids(cmd->opcode, VEHICLE_SRC_TELEMETRY,
                                   cmd->cmd_id, cmd->run_id);
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

static uint8_t g_startup_stack[STACK_STARTUP_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_motion_stack[STACK_MOTION_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_physics_stack[STACK_PHYSICS_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_obstacle_stack[STACK_OBSTACLE_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_line_stack[STACK_LINE_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_telemetry_stack[STACK_TELEMETRY_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_connection_stack[STACK_CONNECTION_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_barcode_stack[STACK_BARCODE_SZ] __attribute__((aligned(STACK_ALIGN)));
static uint8_t g_supervisor_stack[STACK_SUPERVISOR_SZ] __attribute__((aligned(STACK_ALIGN)));

/* Stack marks. Before any task exists, every stack is filled with a
 * known byte. The supervisor reads how much of each stack is still untouched,
 * counted from the deep end, and logs the smallest margins. */
#define STACK_FILL_BYTE   (0xA5U)
#define STACK_MIN_FREE    (512U)
#define STACK_REPORT_CALLS (30U)
#define STACK_COUNT       (9U)
STATIC_ASSERT(STACK_COUNT <= (STACK_MARK_TASK_MASK + 1U), "task index fits the stack mark");

static uint8_t * const g_stack_base[STACK_COUNT] = {
    g_startup_stack,   g_motion_stack,    g_physics_stack,
    g_obstacle_stack,  g_line_stack,      g_telemetry_stack,
    g_connection_stack, g_barcode_stack,  g_supervisor_stack,
};
static uint32_t const g_stack_size[STACK_COUNT] = {
    STACK_STARTUP_SZ,    STACK_MOTION_SZ,  STACK_PHYSICS_SZ,
    STACK_OBSTACLE_SZ,   STACK_LINE_SZ,    STACK_TELEMETRY_SZ,
    STACK_CONNECTION_SZ, STACK_BARCODE_SZ, STACK_SUPERVISOR_SZ,
};

LOCAL void stack_fill (void)
{
    uint32_t idx  = 0U;
    uint32_t byte = 0U;

    for (idx = 0U; idx < STACK_COUNT; idx++)
    {
        for (byte = 0U; byte < g_stack_size[idx]; byte++)
        {
            g_stack_base[idx][byte] = STACK_FILL_BYTE;
        }
    }
}

/* Untouched bytes of one stack, from the deep (low) end. */
LOCAL uint32_t stack_free (uint32_t idx)
{
    uint32_t free_bytes = 0U;

    while ((free_bytes < g_stack_size[idx]) &&
           (STACK_FILL_BYTE == g_stack_base[idx][free_bytes]))
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
    uint32_t        free_bytes[STACK_COUNT];
    uint32_t        idx     = 0U;
    uint32_t        thin    = 0U;

    s_calls++;
    /* The first call scans too, so the diag packet has marks after 1 s. */
    if ((1U != s_calls) && (0U != (s_calls % STACK_REPORT_CALLS)))
    {
        return;
    }
    for (idx = 0U; idx < STACK_COUNT; idx++)
    {
        free_bytes[idx] = stack_free(idx);

        if (free_bytes[idx] < STACK_MIN_FREE)
        {
            thin |= (1UL << idx);
        }
        /* Keep the smallest mark for telemetry (no scan per packet). The
         * stored mark is at most 0xFFFF, so a smaller one fits 16 bits. */
        if (free_bytes[idx] < (g_stack_mark & STACK_MARK_FREE_MASK))
        {
            g_stack_mark = (idx << STACK_MARK_TASK_SHIFT) | free_bytes[idx];
        }
    }
    LOG_INFO(LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
             "stack free: startup %d motion %d physics %d obstacle %d",
             (int)free_bytes[0], (int)free_bytes[1], (int)free_bytes[2], (int)free_bytes[3]);
    LOG_INFO(LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
             "stack free: line %d telemetry %d connection %d barcode %d",
             (int)free_bytes[4], (int)free_bytes[5], (int)free_bytes[6], (int)free_bytes[7]);
    LOG_INFO(LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "stack free: supervisor %d", (int)free_bytes[8]);
    if (0U != thin)
    {
        LOG_WARN(LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "thin stack margin, mask=%d", (int)thin);
    }
}

/* Task table. Core 0 (TP_PRC1) runs real-time control at priorities 1 to 5
 * and owns the GPIO bank IRQ; core 1 (TP_PRC2) runs the network at 6. A task
 * is pinned only for hardware ownership or a deadline. Telemetry and barcode
 * touch no hardware, so they stay unpinned at 7: they fill idle time on
 * either core, run on core 0 only when every control task waits, and reach
 * the pinned tasks through SMP-safe queues (src/telemetry/queue.c).
 * The kernel tick is 10 ms (CNF_TIMER_PERIOD in
 * lib/mtk3smp-rp2040/config/config.h): every timeout and delay can wait up
 * to one tick longer than asked. */

/* Core 0, 1: hardware and ISR setup on the IRQ core before any task runs. */
LOCAL T_CTSK g_ctsk_startup = {
    .itskpri = 1,
    .stksz   = STACK_STARTUP_SZ,
    .task    = startup_task,
    .bufptr  = g_startup_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

/* Core 0, 1 (highest): the 10 ms wheel loop owns the PWM and encoders. */
LOCAL T_CTSK g_ctsk_motion = {
    .itskpri = 1,
    .stksz   = STACK_MOTION_SZ,
    .task    = motion_task,
    .bufptr  = g_motion_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

/* Core 0, 5 (lowest control): its I2C IMU waits never delay steering. */
LOCAL T_CTSK g_ctsk_physics = {
    .itskpri = 5, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_PHYSICS_SZ,
    .task    = physics_task,
    .bufptr  = g_physics_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

/* Core 0, 2: the 20 ms stop and fault pass preempts line, sonar and IMU. */
LOCAL T_CTSK g_ctsk_supervisor = {
    .itskpri = 2, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_SUPERVISOR_SZ,
    .task    = supervisor_task,
    .bufptr  = g_supervisor_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

/* Core 0, 4: 50 ms sonar pings and servo yield to the 5 ms line pass. */
LOCAL T_CTSK g_ctsk_obstacle = {
    .itskpri = 4, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_OBSTACLE_SZ,
    .task    = obstacle_task,
    .bufptr  = g_obstacle_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

/* Core 0, 3: the 5 ms line pass steers, below motion and the supervisor. */
LOCAL T_CTSK g_ctsk_line = {
    .itskpri = 3, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_LINE_SZ,
    .task    = line_task,
    .bufptr  = g_line_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

/* Unpinned, 7: packs snapshots into frames; no hardware (see above). */
LOCAL T_CTSK g_ctsk_telemetry = {
    .itskpri = 7, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_TELEMETRY_SZ,
    .task    = telemetry_task,
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF,
    .bufptr  = g_telemetry_stack,
};

/* Core 1, 6: MQTT session; the CYW43 and lwIP poll runs on core 0, at 4. */
LOCAL T_CTSK g_ctsk_connection = {
    .itskpri = 6, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_CONNECTION_SZ,
    .task    = connection_task,
    .bufptr  = g_connection_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

/* Unpinned, 7: decodes the ISR edge ring; no hardware (see above). */
LOCAL T_CTSK g_ctsk_barcode = {
    .itskpri = 7, /* NOLINT(readability-magic-numbers): AGENTS.md keeps task attributes literal */
    .stksz   = STACK_BARCODE_SZ,
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
        system_fault_assign(FAULT_OWNER_SUPERVISOR, FAULT_TASK_STALLED_BIT, FAULT_TASK_STALLED_BIT);
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
        system_fault_assign(FAULT_OWNER_SUPERVISOR, FAULT_TASK_STALLED_BIT, FAULT_TASK_STALLED_BIT);
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
    if (p_tasks->b_telemetry)
    {
        obstacle_bind_sonar_aim(telemetry_sonar_aim);
    }
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
        b_ok        = made("cyc(supervisor)", tk_cre_cyc(&ccyc));
    }

    return b_ok;
}

LOCAL bool start_button_pressed (void)
{
    return (0U == gpio_get_val(START_BUTTON_PIN));
}

/*!
 * @brief Give the supervisor the tasks that must show a pulse. A task that did
 *        not start is left out.
 */
LOCAL bool init_supervisor (task_set_t const * p_tasks)
{
    supervisor_config_t config = {0};
    ER const button_result = gpio_set_pin(START_BUTTON_PIN, GPIO_MODE_IN);

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
    config.p_telemetry_seen = connection_mqtt_seen;
    if (E_OK == button_result)
    {
        clr_w(GPIO(START_BUTTON_PIN), START_BUTTON_PULL_DOWN);
        set_w(GPIO(START_BUTTON_PIN), START_BUTTON_PULL_UP | START_BUTTON_SCHMITT);
        config.p_start_pressed = start_button_pressed;
    }
    else
    {
        LOG_WARN(LOG_TASK_BOOT, LOG_MOD_NONE, "GP20 START input setup failed, er=%d", (int)button_result);
    }
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

    /* This composition root creates every task. Subsystems only
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
        else
        {
            g_startup.connection_tid = p_tasks->connection_tid;
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

    /* The kernel boot table leaves GP8/GP9 (left motor) on I2C0 with
     * pull-ups. Hold every bridge input low before anything else runs; the
     * motion service repeats it when it takes the pins. */
    pwm_boot_brake();
    log_init();
    stack_fill();

    cflg.flgatr     = TA_WMUL | TA_TFIFO;
    cflg.iflgptn    = 0U;
    g_startup.h_flg = tk_cre_flg(&cflg);

    init_services(&tasks);
    if (tasks.b_telemetry)
    {
        /* Capture supervisor startup faults as well as task reports. */
        system_fault_bind_observer(telemetry_fault_changed);
    }

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
