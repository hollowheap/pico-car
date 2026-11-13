/** @file obstacle.c
 *
 * @brief Buddy 5 public API. It runs the start-up order and otherwise
 *        forwards to b5_comms, which owns the queues and published copies.
 *
 * NOTE: The kernel API types ID, ER, TMO and T_CTSK are used where the
 * kernel functions require them (deviation from BARR-C 5.2.a, see
 * docs/buddy5_obstacle.md).
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "b5_cfg.h"
#include "b5_pack.h"
#include "b5_comms.h"
#include "qr_obstacle.h"
#include "../obstacle_pack.h"
#include "scan_hw.h"
#include "scan_task.h"

/* Processor that owns Buddy 5's ISR and task (RP2040 core 0). */
#define PROCESSOR_1 (1)

/* Start-up state. NOTE: The Boolean global is a b_ member of a g_ struct,
 * which satisfies both BARR-C 7.1.m/7.1.o and the team .clang-tidy. */
typedef struct
{
    bool volatile b_ready;
} obstacle_state_t;

/* Set once start-up has finished; read by every API call. */
static obstacle_state_t g_obstacle = {false};

/* QR scans -60..+60; the dashboard protocol retains -90..+90. */
#define PROFILE_PAD ((OBSTACLE_PROFILE_SLOTS - B5_SLOT_COUNT) / 2U)

B5_CT_ASSERT(profile_window, (OBSTACLE_PROFILE_SLOTS >= B5_SLOT_COUNT) &&
             (0U == ((OBSTACLE_PROFILE_SLOTS - B5_SLOT_COUNT) % 2U)));
B5_CT_ASSERT(supervisor_commands, (OBSTACLE_CMD_STOP == B5_CMD_STOP) &&
             (OBSTACLE_CMD_RESET == B5_CMD_RESET) &&
             (OBSTACLE_CMD_START == B5_CMD_START));

static uint8_t wire_echo_class (uint32_t cls);

/*!
 * @brief Start Buddy 5: links, event flag and hardware.
 *
 * Must run on processor 1 after gpio_irq_init().
 *
 * @param[in] p_links Links to Buddy 2 and Navigation; NULL means none.
 *
 * @return E_OK; E_OBJ if already started or PWM slice 6 is in use; E_CTX
 *         on the wrong core; E_SYS if the 1 MHz TIMER is not ticking; or
 *         a kernel error.
 */
ER obstacle_init (obstacle_links_t const * p_links)
{
    ER result = E_OK;

    if (g_obstacle.b_ready)
    {
        result = E_OBJ;
    }
#if TK_SUPPORT_SMP
    else if (PROCESSOR_1 != tk_get_prc())
    {
        result = E_CTX;
    }
#endif
    else
    {
        result = b5_comms_init(p_links);
    }

    if ((E_OK == result) && (!g_obstacle.b_ready))
    {
        result = scan_hw_init(b5_comms_flag());
        if (E_OK != result)
        {
            b5_comms_set_init_failed();
        }
    }

    if ((E_OK == result) && (!g_obstacle.b_ready))
    {
        /* The dev-stack composition root starts the scan task. */
        g_obstacle.b_ready = true;
    }

    return (result);
}

/*!
 * @brief Post a command built with b5_pack_cmd(). Never blocks; STOP is
 *        never lost.
 *
 * @param[in] cmd Packed command byte.
 *
 * @return E_OK; E_LIMIT if the 4-slot queue is full; E_OBJ before init.
 */
ER obstacle_post_cmd (uint8_t cmd)
{
    ER result = E_OBJ;

    if (g_obstacle.b_ready)
    {
        result = b5_comms_post_cmd(cmd);
    }

    return (result);
}

/*!
 * @brief Next event, when Navigation did not supply an inbox.
 *
 * @param[out] p_evt The event.
 * @param[in]  tmout How long to wait.
 *
 * @return E_OK, E_TMOUT if none in time, or E_OBJ before init.
 */
ER obstacle_get_event (uint32_t * p_evt, TMO tmout)
{
    ER result = E_OBJ;

    if (g_obstacle.b_ready)
    {
        result = b5_comms_get_event(p_evt, tmout);
    }

    return (result);
}

/*!
 * @brief Latest STEER word; its sequence field changes on every decision.
 *
 * @return The packed STEER event word.
 */
uint32_t obstacle_get_steer (void)
{
    return (b5_comms_steer());
}

/*!
 * @brief Tell Buddy 5 where the servo should point during REACQUIRE
 *        (Navigation decides, as the bypass needs different angles in
 *        each part). Clamped to the servo limit.
 *
 * @param[in] deci Angle in deci-degrees, positive = left.
 */
void obstacle_set_look (int16_t deci)
{
    int16_t clamped = deci;

    if (clamped > (int16_t)B5_SERVO_LIMIT_DECI)
    {
        clamped = (int16_t)B5_SERVO_LIMIT_DECI;
    }
    else if (clamped < (int16_t)(-B5_SERVO_LIMIT_DECI))
    {
        clamped = (int16_t)(-B5_SERVO_LIMIT_DECI);
    }
    else
    {
        /* In range. */
    }

    b5_comms_set_look(clamped);
}

/*!
 * @brief Packed status word, read in one 32-bit load.
 *
 * @return The status word.
 */
uint32_t obstacle_get_status (void)
{
    return (b5_comms_status());
}

/*!
 * @brief Fault byte.
 *
 * @return The B5_FAULT_* bits.
 */
uint16_t obstacle_get_faults (void)
{
    return (b5_comms_faults());
}

/*!
 * @brief Events dropped because an inbox or the outbox was full.
 *
 * @return The count, saturating at 255.
 */
uint8_t obstacle_dropped_events (void)
{
    return (b5_comms_drops());
}

/*!
 * @brief Centre QR's 25 readings in the dashboard's 37-slot profile. Never waits.
 *
 * @param[out] p_profile Room for OBSTACLE_PROFILE_SLOTS packed slots.
 *
 * @return E_OK, or E_BUSY if Buddy 5 was writing it.
 */
ER obstacle_copy_profile (uint16_t * p_profile)
{
    uint16_t slots[B5_SLOT_COUNT];
    ER const result = (NULL != p_profile && g_obstacle.b_ready)
                         ? b5_comms_copy_profile(slots)
                         : ((NULL == p_profile) ? E_PAR : E_OBJ);

    if (E_OK == result)
    {
        uint32_t idx = 0U;

        for (idx = 0U; idx < OBSTACLE_PROFILE_SLOTS; idx++)
        {
            p_profile[idx] = 0U;
        }

        for (idx = 0U; idx < B5_SLOT_COUNT; idx++)
        {
            uint16_t const slot = slots[idx];

            if (b5_pack_slot_is_measured(slot))
            {
                p_profile[PROFILE_PAD + idx] = obstacle_pack_slot(
                    b5_pack_slot_dist(slot), wire_echo_class(b5_pack_slot_class(slot)));
            }
        }
    }

    return (result);
}

/*!
 * @brief Buddy 5's event flag, for waiting on the B5_FLG_NAV_* bits.
 *
 * @return The flag id, or 0 before init.
 */
ID obstacle_flag_id (void)
{
    return (b5_comms_flag());
}

ER obstacle_snapshot (obstacle_status_t * p_status)
{
    ER result = E_PAR;

    if (NULL != p_status)
    {
        result = E_OBJ;
        if (g_obstacle.b_ready)
        {
            uint32_t const word = obstacle_get_status();

            p_status->distance_mm = (uint16_t)b5_pack_get(word, B5_ST_DIST_SH, B5_ST_DIST_MK);
            p_status->faults = obstacle_get_faults();
            p_status->servo_deci = b5_comms_servo();
            p_status->body_deci = 0; /* Servo-only scan. */
            p_status->state = (uint8_t)b5_pack_get(word, B5_ST_STATE_SH, B5_ST_STATE_MK);
            p_status->cause = (uint8_t)b5_pack_get(word, B5_ST_CAUSE_SH, B5_ST_CAUSE_MK);
            p_status->echo_class = wire_echo_class(b5_pack_get(word, B5_ST_CLASS_SH, B5_ST_CLASS_MK));
            p_status->req_id = (uint8_t)b5_pack_get(word, B5_ST_ID_SH, B5_ST_ID_MK);
            p_status->seq = (uint8_t)(b5_pack_get(word, B5_ST_SEQ_SH, B5_ST_SEQ_MK) & OBSTACLE_ST_SEQ_MK);
            p_status->dropped_events = obstacle_dropped_events();
            p_status->flags = 0U;
            p_status->ready = true;
            result = E_OK;
        }
    }

    return (result);
}

ER obstacle_copy_plan (obstacle_plan_summary_t * p_plan)
{
    ER result = E_PAR;

    if (NULL != p_plan)
    {
        result = g_obstacle.b_ready ? b5_comms_copy_plan(p_plan) : E_OBJ;
    }

    return (result);
}

void obstacle_echo_counts (uint32_t * p_counts)
{
    if ((NULL != p_counts) && g_obstacle.b_ready)
    {
        b5_comms_echo_counts(p_counts);
    }
}

void obstacle_run (void)
{
    scan_task_obstacle_task(0, NULL);
}

static uint8_t wire_echo_class (uint32_t cls)
{
    /* QR combines absent and malformed echoes internally. */
    static uint8_t const g_classes[] = { OBSTACLE_CLASS_VALID, OBSTACLE_CLASS_OPEN,
                                        OBSTACLE_CLASS_NO_ECHO, OBSTACLE_CLASS_STUCK };

    return (g_classes[cls & B5_SLOT_CLASS_MK]);
}

/*** end of file ***/
