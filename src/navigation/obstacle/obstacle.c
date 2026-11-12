/** @file obstacle.c
 *
 * @brief Buddy 5 public API. It runs the start-up order and otherwise
 *        forwards to obstacle_comms, which owns the queues and published copies.
 *
 * NOTE: The kernel API types ID, ER, TMO and T_CTSK are used where the
 * kernel functions require them (deviation from BARR-C 5.2.a, see
 * docs/obstacle.md).
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "obstacle_config.h"
#include "comms.h"
#include "obstacle_internal.h"
#include "obstacle_pack.h"
#include "scan_hw.h"
#include "scan_task.h"
#include "navigation/obstacle.h"

/* The public profile size is the scan's slot count. */
OBSTACLE_CT_ASSERT(profile_slots_public, OBSTACLE_SLOT_COUNT == OBSTACLE_PROFILE_SLOTS);

/* Processor that owns Buddy 5's ISR and task (RP2040 core 0). */
#define PROCESSOR_1 (1)

/* The aim word holds the servo angle in the low half and the body offset in the high half. */
#define AIM_HALF_MASK  (0xFFFFU)
#define AIM_BODY_SHIFT (16U)

/* Start-up state. NOTE: The Boolean global is a b_ member of a g_ struct,
 * which satisfies both BARR-C 7.1.m/7.1.o and the team .clang-tidy. */
typedef struct
{
    bool volatile b_ready;
} obstacle_state_t;

/* Set once start-up has finished; read by every API call. */
static obstacle_state_t g_obstacle = {false};

/*!
 * @brief Start Buddy 5: links, event flag and hardware.
 *
 * Must run on processor 1 after gpio_irq_init(). The composition root
 * creates the task and calls obstacle_run() from it.
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
        result = obstacle_comms_init(p_links);
    }

    if ((E_OK == result) && (!g_obstacle.b_ready))
    {
        result = scan_hw_init(obstacle_comms_flag());
        if (E_OK != result)
        {
            obstacle_comms_set_init_failed();
        }
    }

    if ((E_OK == result) && (!g_obstacle.b_ready))
    {
        /* The composition root creates the task and calls obstacle_run();
         * init only prepares the links, flag, ISR and servo. */
        g_obstacle.b_ready = true;
    }

    return (result);
}

/*!
 * @brief Post a command built with obstacle_pack_cmd(). Never blocks; STOP is
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
        result = obstacle_comms_post_cmd(cmd);
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
        result = obstacle_comms_get_event(p_evt, tmout);
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
    return (obstacle_comms_steer());
}

/*!
 * @brief Packed status word, read in one 32-bit load.
 *
 * @return The status word.
 */
uint32_t obstacle_get_status (void)
{
    return (obstacle_comms_status());
}

/*!
 * @brief Fault byte.
 *
 * @return The OBSTACLE_FAULT_* bits.
 */
uint16_t obstacle_get_faults (void)
{
    return (obstacle_comms_faults());
}

/*!
 * @brief Copy the latest state in engineering units.
 *
 * Telemetry uses this instead of the packed word, so the wire layout stays
 * inside src/navigation/obstacle/. The numeric codes are not fixed by this header.
 *
 * @param[out] p_status Destination.
 *
 * @return E_OK, E_PAR for a null pointer, or E_OBJ before init.
 */
ER obstacle_snapshot (obstacle_status_t * p_status)
{
    uint32_t word = 0U;
    uint32_t aim  = 0U;
    ER       result = E_PAR;

    if (NULL != p_status)
    {
        result = E_OBJ;

        if (g_obstacle.b_ready)
        {
            word = obstacle_get_status();

            p_status->ready = true;
            p_status->state = (uint8_t)obstacle_pack_get(
                word, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK);
            p_status->cause = (uint8_t)obstacle_pack_get(
                word, OBSTACLE_ST_CAUSE_SH, OBSTACLE_ST_CAUSE_MK);
            p_status->faults = obstacle_get_faults();
            p_status->echo_class = (uint8_t)obstacle_pack_get(
                word, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK);
            p_status->distance_mm = (uint16_t)obstacle_pack_get(
                word, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK);
            p_status->req_id = (uint8_t)obstacle_pack_get(
                word, OBSTACLE_ST_ID_SH, OBSTACLE_ST_ID_MK);
            p_status->seq = (uint8_t)obstacle_pack_get(
                word, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);
            p_status->dropped_events = obstacle_dropped_events();
            p_status->flags          = obstacle_comms_flags();
            aim                      = obstacle_comms_aim();
            p_status->servo_deci     = (int16_t)(uint16_t)(aim & AIM_HALF_MASK);
            p_status->body_deci      = (int16_t)(uint16_t)(aim >> AIM_BODY_SHIFT);

            result = E_OK;
        }
    }

    return (result);
}

/*!
 * @brief Events dropped because an inbox or the outbox was full.
 *
 * @return The count, saturating at 255.
 */
uint8_t obstacle_dropped_events (void)
{
    return (obstacle_comms_drops());
}

/*!
 * @brief Copy of the 37-slot profile for telemetry. Never waits.
 *
 * @param[out] p_profile Room for OBSTACLE_SLOT_COUNT packed slots.
 *
 * @return E_OK, or E_BUSY if Buddy 5 was writing it.
 */
ER obstacle_copy_profile (uint16_t * p_profile)
{
    return (obstacle_comms_copy_profile(p_profile));
}

/*!
 * @brief Copy of the plan from the last finished scan. Never waits.
 *
 * @param[out] p_plan Destination.
 *
 * @return E_OK, E_PAR for a null pointer, E_OBJ before init, or E_BUSY if the
 *         obstacle task was writing it.
 */
ER obstacle_copy_plan (obstacle_plan_summary_t * p_plan)
{
    ER result = E_PAR;

    if (NULL != p_plan)
    {
        result = g_obstacle.b_ready ? obstacle_comms_copy_plan(p_plan) : E_OBJ;
    }

    return (result);
}

/*!
 * @brief Pings per echo class since boot.
 *
 * @param[out] p_counts OBSTACLE_ECHO_CLASSES counts.
 */
void obstacle_echo_counts (uint32_t * p_counts)
{
    if ((NULL != p_counts) && g_obstacle.b_ready)
    {
        obstacle_comms_class_counts(p_counts);
    }
}

/*!
 * @brief Buddy 5's event flag, for waiting on the OBSTACLE_FLG_NAV_* bits.
 *
 * @return The flag id, or 0 before init.
 */
ID obstacle_flag_id (void)
{
    return (obstacle_comms_flag());
}

/*!
 * @brief Task body. The composition root creates the task and calls this.
 *
 * Never returns.
 */
void obstacle_run (void)
{
    scan_task_obstacle_task(0, NULL);
}

/*** end of file ***/

