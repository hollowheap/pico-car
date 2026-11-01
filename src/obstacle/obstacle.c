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
#include "b5_comms.h"
#include "obstacle.h"
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

static ER create_task (void);

/*!
 * @brief Start Buddy 5: links, event flag, hardware, then the task.
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
        result             = create_task();
        g_obstacle.b_ready = (E_OK == result);
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
uint8_t obstacle_get_faults (void)
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
 * @brief Copy of the 25-slot profile for telemetry. Never waits.
 *
 * @param[out] p_profile Room for B5_SLOT_COUNT packed slots.
 *
 * @return E_OK, or E_BUSY if Buddy 5 was writing it.
 */
ER obstacle_copy_profile (uint16_t * p_profile)
{
    return (b5_comms_copy_profile(p_profile));
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

/*!
 * @brief Create and start the Buddy 5 task on processor 1.
 *
 * @return E_OK or a kernel error.
 */
static ER create_task (void)
{
    ER     result = E_OK;
    ID     h_task = 0;
    T_CTSK ctsk   = {
          .exinf   = NULL,
          .itskpri = B5_TASK_PRI,
          .stksz   = B5_TASK_STACK,
          .task    = scan_task_obstacle_task,
#if TK_SUPPORT_SMP
        .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
        .assprc = TP_PRC1,
#else
        .tskatr = TA_HLNG | TA_RNG3,
#endif
    };

    h_task = tk_cre_tsk(&ctsk);
    if (h_task > 0)
    {
        result = tk_sta_tsk(h_task, 0);
    }
    else
    {
        /* NOTE: A negative object id is the kernel's error code. */
        result = (ER)h_task;
    }

    return (result);
}

/*** end of file ***/

