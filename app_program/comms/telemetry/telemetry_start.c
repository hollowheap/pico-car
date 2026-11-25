/** @file telemetry_start.c
 * @brief The telemetry and connection tasks, the telemetry init and the
 *        creation of both tasks. See telemetry.h.
 */

#include <tk/tkernel.h>
#include <tk/smp.h>

#include "comms/telemetry/telemetry.h"

#include "motion/motion.h"
#include "obstacle/obstacle.h"
#include "imu/imu.h"
#include "platform/drv_timer.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/platform.h"
#include "platform/stack.h"
#include "platform/static_assert.h"
#include "supervisor/supervisor.h"
#include "comms/telemetry/assemble.h"
#include "comms/telemetry/cmd.h"
#include "comms/telemetry/connection.h"
#include "comms/telemetry/emit.h"
#include "comms/telemetry/hug_word.h"
#include "comms/telemetry/net_queue.h"
#include "comms/telemetry/packet.h"
#include "comms/telemetry/queue.h"
#include "comms/telemetry/worker.h"
#include "ir/ir.h"

#define STACK_TELEMETRY_SZ  (2816U)
#define STACK_CONNECTION_SZ (2048U)

/* Width of the fault fields on the wire (bits 0..15, see platform/faults.h).
   A seventeenth fault bit needs wider status, diag and event fields. */
#define FAULT_WORD_BITS              (16U)
STATIC_ASSERT(FAULT_ALL_MASK <= UINT16_MAX, "fault bits fit the wire fields");

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

static uint8_t g_telemetry_stack[STACK_TELEMETRY_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));
static uint8_t g_connection_stack[STACK_CONNECTION_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));

static bool     g_b_telemetry    = false;
static bool     g_b_connection   = false;
static ID       g_telemetry_tid  = 0;
static ID       g_connection_tid = 0;
static uint32_t g_telemetry_overruns;

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

/*!
 * @brief Queue one telemetry frame for the MQTT radio owner.
 */
LOCAL int net_sink (const uint8_t * frame, uint16_t len)
{
    (void)telemetry_net_queue_push(frame, len);
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
    uint32_t const    stack_mark = platform_stack_mark();
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
    diag.stack_min_free      = (uint16_t)(stack_mark & PLATFORM_STACK_MARK_FREE_MASK);
    diag.stack_min_task      = (uint8_t)((stack_mark >> PLATFORM_STACK_MARK_SLOT_SHIFT) &
                                         PLATFORM_STACK_MARK_SLOT_MASK);
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
    diag.sensor_sample_us = now_us();

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
    if (!platform_wait_startup())
    {
        platform_park();
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
                platform_stack_report();
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

LOCAL void connection_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!platform_wait_startup())
    {
        platform_park();
    }

    connection_run();
}

/* Unpinned, 7: packs snapshots into frames; no hardware (see platform.c). */
LOCAL T_CTSK g_ctsk_telemetry = {
    .itskpri = 7, /* NOLINT(readability-magic-numbers): the task table is kept literal by the project rules */
    .stksz   = STACK_TELEMETRY_SZ,
    .task    = telemetry_task,
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF,
    .bufptr  = g_telemetry_stack,
};

/* Core 1, 6: MQTT commands and heartbeat. The radio owner runs on core 0 at 4. */
LOCAL T_CTSK g_ctsk_connection = {
    .itskpri = 6, /* NOLINT(readability-magic-numbers): the task table is kept literal by the project rules */
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

void telemetry_init (connection_cmd_dispatch_t dispatch)
{
    ER result = E_OK;

    platform_stack_register(PLATFORM_STACK_TELEMETRY, g_telemetry_stack, STACK_TELEMETRY_SZ);
    platform_stack_register(PLATFORM_STACK_CONNECTION, g_connection_stack, STACK_CONNECTION_SZ);

    /* Telemetry is optional: the car still runs when the queues or the
     * network initialisation fail. */
    result = telemetry_queue_create();
    g_b_telemetry = (E_OK == result);
    result = telemetry_net_queue_create();
    g_b_telemetry = g_b_telemetry && (E_OK == result);
    if (g_b_telemetry)
    {
        obstacle_bind_sonar_aim(telemetry_sonar_aim);
    }
    if (!g_b_telemetry)
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "telemetry queues failed, er=%d",
                  (int)result);
    }

    result = connection_init(dispatch, NULL);
    g_b_connection = (E_OK == result);
    if (!g_b_connection)
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "connection_init failed, er=%d",
                  (int)result);
    }

    if (g_b_telemetry)
    {
        /* Capture supervisor startup faults as well as task reports. */
        system_fault_bind_observer(telemetry_fault_changed);
    }
}

void telemetry_start (void)
{
    if (g_b_telemetry)
    {
        g_telemetry_tid = tk_cre_tsk(&g_ctsk_telemetry);
        if (!platform_task_made("tsk(telemetry)", g_telemetry_tid))
        {
            g_b_telemetry = false;
        }
        else
        {
            (void)platform_task_started("tsk(telemetry)", g_telemetry_tid);
        }
    }

    if (g_b_connection)
    {
        g_connection_tid = tk_cre_tsk(&g_ctsk_connection);
        if (!platform_task_made("tsk(connection)", g_connection_tid))
        {
            g_b_connection = false;
            g_connection_tid = 0;
        }
        else
        {
            (void)platform_task_started("tsk(connection)", g_connection_tid);
        }
    }
}

uint16_t telemetry_live_mask (void)
{
    uint16_t mask = 0U;

    if (g_b_telemetry)
    {
        mask |= (uint16_t)(1U << LIVE_TELEMETRY);
    }
    if (g_b_connection)
    {
        mask |= (uint16_t)(1U << LIVE_CONNECTION);
    }
    return mask;
}

/*** end of file ***/
