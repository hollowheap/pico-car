/** @file service.c
 *
 * @brief Buddy 5 public API. It runs the start-up order and otherwise
 *        forwards to extras_comms, which owns the queues and published copies.
 *
 * NOTE: The kernel API types ID, ER, TMO and T_CTSK are used where the
 * kernel functions require them (deviation from BARR-C 5.2.a, see
 * testing/README.md).
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "config.h"
#include "comms.h"
#include "service.h"
#include "drv_sonar.h"

/* Processor that owns Buddy 5's ISR and task (RP2040 core 0). */
#define PROCESSOR_1 (1)

/* Start-up state. NOTE: The Boolean global is a b_ member of a g_ struct,
 * which satisfies both BARR-C 7.1.m/7.1.o and the team .clang-tidy. */
typedef struct
{
    bool volatile b_ready;
} obstacle_extras_state_t;

/* Set once start-up has finished; read by every API call. */
static obstacle_extras_state_t g_obstacle = {false};

/* The scan task checks hardware readiness after the platform barrier opens. */
bool obstacle_extras_ready (void)
{
    return g_obstacle.b_ready;
}

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
ER obstacle_extras_setup (obstacle_extras_links_t const * p_links)
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
        result = extras_comms_init(p_links);
    }

    if ((E_OK == result) && (!g_obstacle.b_ready))
    {
        result = extras_sonar_init(extras_comms_flag());
        if (E_OK != result)
        {
            extras_comms_set_init_failed();
        }
    }

    if ((E_OK == result) && (!g_obstacle.b_ready))
    {
        g_obstacle.b_ready = true;
    }

    return (result);
}

/*!
 * @brief Post a command built with extras_pack_cmd(). Never blocks; STOP is
 *        never lost.
 *
 * @param[in] cmd Packed command byte.
 *
 * @return E_OK; E_LIMIT if the 4-slot queue is full; E_OBJ before init.
 */
ER obstacle_extras_post_cmd (uint8_t cmd)
{
    ER result = E_OBJ;

    if (g_obstacle.b_ready)
    {
        result = extras_comms_post_cmd(cmd);
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
ER obstacle_extras_get_event (uint32_t * p_evt, TMO tmout)
{
    ER result = E_OBJ;

    if (g_obstacle.b_ready)
    {
        result = extras_comms_get_event(p_evt, tmout);
    }

    return (result);
}

/*!
 * @brief Latest STEER word; its sequence field changes on every decision.
 *
 * @return The packed STEER event word.
 */
uint32_t obstacle_extras_get_steer (void)
{
    return (extras_comms_steer());
}

/*!
 * @brief Tell Buddy 5 where the servo should point during REACQUIRE
 *        (Navigation decides, as the bypass needs different angles in
 *        each part). Clamped to the servo limit.
 *
 * @param[in] deci Angle in deci-degrees, positive = left.
 */
void obstacle_extras_set_look (int16_t deci)
{
    int16_t clamped = deci;

    if (clamped > (int16_t)EXTRAS_SERVO_LIMIT_DECI)
    {
        clamped = (int16_t)EXTRAS_SERVO_LIMIT_DECI;
    }
    else if (clamped < (int16_t)(-EXTRAS_SERVO_LIMIT_DECI))
    {
        clamped = (int16_t)(-EXTRAS_SERVO_LIMIT_DECI);
    }
    else
    {
        /* In range. */
    }

    extras_comms_set_look(clamped);
}

/*!
 * @brief Tell Buddy 5 that Navigation's chicken-head rotation scan is
 *        running (REACQUIRE only): the servo then follows every new look
 *        angle at once, without the pre-delay and settle wait, so it can
 *        turn smoothly while the car rotates. Navigation turns it off again
 *        on every way out of the scan.
 *
 * @param[in] b_on True while the rotation scan runs.
 */
void obstacle_extras_set_track (bool b_on)
{
    extras_comms_set_track(b_on);
}

/*!
 * @brief Tell Buddy 5 that Navigation's side scan (SWEEP) is running
 *        (REACQUIRE only): the car stands still, so the collision guard
 *        ignores close readings, which only mean "in the way". Navigation
 *        turns it off again on every way out of the sweep.
 *
 * @param[in] b_on True while the side scan runs.
 */
void obstacle_extras_set_sweep (bool b_on)
{
    extras_comms_set_sweep(b_on);
}

/*!
 * @brief Packed status word, read in one 32-bit load.
 *
 * @return The status word.
 */
uint32_t obstacle_extras_get_status (void)
{
    return (extras_comms_status());
}

/*!
 * @brief Which servo angle the newest ping was taken at, read in one
 *        32-bit load: seq 6 | angle + EXTRAS_TAG_LOOK_BIAS 11 (extras_pack.h). The
 *        ping belongs to the status word with the same seq.
 *
 * @return The packed ping tag.
 */
uint32_t obstacle_extras_get_ping_tag (void)
{
    return (extras_comms_ping_tag());
}

/*!
 * @brief Fault byte.
 *
 * @return The EXTRAS_FAULT_* bits.
 */
uint8_t obstacle_extras_get_faults (void)
{
    return (extras_comms_faults());
}

/*!
 * @brief Events dropped because an inbox or the outbox was full.
 *
 * @return The count, saturating at 255.
 */
uint8_t obstacle_extras_dropped_events (void)
{
    return (extras_comms_drops());
}

/*!
 * @brief Copy of the 25-slot profile for telemetry. Never waits.
 *
 * @param[out] p_profile Room for EXTRAS_SLOT_COUNT packed slots.
 *
 * @return E_OK, or E_BUSY if Buddy 5 was writing it.
 */
ER obstacle_extras_copy_profile (uint16_t * p_profile)
{
    return (extras_comms_copy_profile(p_profile));
}

/*!
 * @brief Buddy 5's event flag, for waiting on the EXTRAS_FLG_NAV_* bits.
 *
 * @return The flag id, or 0 before init.
 */
ID obstacle_extras_flag_id (void)
{
    return (extras_comms_flag());
}

/*** end of file ***/
