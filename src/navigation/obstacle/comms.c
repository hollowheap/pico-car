/** @file comms.c
 *
 * @brief the obstacle task communications: the command queue, the event outbox, the
 *        published copies that other tasks read without ever blocking
 *        the obstacle task, and the links to the motion service.
 *
 * Sharing rules:
 *   - Status word and steering word: one 32-bit store, one 32-bit load.
 *   - Fault byte and drop counter: the the obstacle task task is the only writer.
 *   - Profile: a sequence lock. The writer never waits; a reader retries
 *     once and then gets E_BUSY.
 *   - Command queue and outbox: many callers, so each is guarded by a
 *     spinlock that also pauses interrupts on the caller's core for a few
 *     instructions.
 *   - STOP never enters the queue. It sets its own flag bit, so it cannot
 *     be lost when the queue is full.
 *
 * NOTE: The kernel API types ID, ER, UINT, INT and TMO are used where the
 * kernel functions require them (deviation from BARR-C 5.2.a, see
 * docs/obstacle.md).
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#include "obstacle_config.h"
#include "comms.h"
#include "statechart.h"
#include "obstacle_pack.h"
#include "obstacle_internal.h"

/* A reader of the profile tries this many times before giving up. */
#define PROFILE_TRIES (2U)

/* Milliseconds in one second, for the odometer estimate. */
#define MSEC_PER_SECOND (1000U)

/* The lock word type. Without SMP the lock is only a placeholder, because
 * pausing interrupts on the one core is enough. */
#if TK_SUPPORT_SMP
typedef T_SPLOCK comms_lock_t;
#else
typedef uint32_t volatile comms_lock_t;
#endif

/* Handles and links. NOTE: Globals that hold a handle are grouped in a
 * g_ struct with h_ members, which satisfies both BARR-C 7.1.n/7.1.o and
 * the team .clang-tidy (it accepts only g_ and gp_ for globals). */
typedef struct
{
    ID volatile h_flag;
    ID volatile h_nav_mbf;
    obstacle_stop_fn_t volatile p_stop;
    obstacle_distance_fn_t volatile p_get_distance_mm;
} comms_links_t;

/* ------------------------------------------------------------------ *
 * Static data. Every item is shared between tasks or with the ISR side,
 * so each one is volatile (BARR-C 1.8.c).
 * ------------------------------------------------------------------ */

static comms_links_t g_links = {0, 0, NULL, NULL};

static uint32_t volatile g_status = 0U;
static uint32_t volatile g_steer  = 0U;
static uint8_t volatile g_faults  = 0U;
static uint8_t volatile g_drops   = 0U;

/* The sequence number is odd while the profile is being written. */
static uint16_t volatile g_profile[OBSTACLE_SLOT_COUNT];
static uint8_t volatile g_profile_seq = 0U;

static uint8_t volatile g_cmd_ring[OBSTACLE_CMD_RING_LEN];
static uint8_t volatile g_cmd_head  = 0U;
static uint8_t volatile g_cmd_count = 0U;
static uint32_t volatile g_evt_ring[OBSTACLE_EVT_RING_LEN];
static uint8_t volatile g_evt_head  = 0U;
static uint8_t volatile g_evt_count = 0U;

static comms_lock_t g_cmd_lock = 0U;
static comms_lock_t g_evt_lock = 0U;

/* ------------------------------------------------------------------ *
 * Private function prototypes.
 * ------------------------------------------------------------------ */

static UINT lock_take (comms_lock_t * p_lock);
static void lock_give (comms_lock_t * p_lock, UINT intsts);
static void memory_barrier (void);
static bool evt_push (uint32_t evt);
static bool evt_pop (uint32_t * p_evt);
static void count_drop (void);
static void deliver (uint32_t evt, UINT flag_bit);
static void publish_profile (obstacle_statechart_t const * p_fsm);

/* ------------------------------------------------------------------ *
 * Public functions.
 * ------------------------------------------------------------------ */

/*!
 * @brief Store the links, set up the locks and create the event flag.
 *
 * @param[in] p_links Links to the motion service and Navigation; NULL means none.
 *
 * @return E_OK, or the error from tk_cre_flg().
 */
ER obstacle_comms_init (obstacle_links_t const * p_links)
{
    ER     result = E_OK;
    ID     h_new  = 0;
    T_CFLG cflg   = {
          .exinf   = NULL,
          .flgatr  = TA_TFIFO | TA_WMUL,
          .iflgptn = 0U,
    };

    if (NULL != p_links)
    {
        g_links.h_nav_mbf         = p_links->h_nav_mbf;
        g_links.p_stop            = p_links->p_stop;
        g_links.p_get_distance_mm = p_links->p_get_distance_mm;
    }

#if TK_SUPPORT_SMP
    (void)InitSpinLock(&g_cmd_lock);
    (void)InitSpinLock(&g_evt_lock);
#endif

    h_new = tk_cre_flg(&cflg);
    if (h_new > 0)
    {
        g_links.h_flag = h_new;
    }
    else
    {
        /* NOTE: A negative object id is the kernel's error code. */
        result = (ER)h_new;
    }

    return (result);
}

/*!
 * @brief Show STOPPED with cause INIT_FAIL to telemetry.
 */
void obstacle_comms_set_init_failed (void)
{
    uint32_t word = 0U;

    word = obstacle_pack_put(word, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK, OBSTACLE_STATE_STOPPED);
    word =
        obstacle_pack_put(word, OBSTACLE_ST_CAUSE_SH, OBSTACLE_ST_CAUSE_MK, OBSTACLE_CAUSE_INIT_FAIL);
    g_status = word;
}

/*!
 * @brief the obstacle task's event flag.
 *
 * @return The flag id, or 0 before obstacle_comms_init().
 */
ID obstacle_comms_flag (void)
{
    return (g_links.h_flag);
}

/*!
 * @brief Post one command. Never blocks.
 *
 * @param[in] cmd Packed command byte (obstacle_pack_cmd()).
 *
 * @return E_OK, E_LIMIT if the queue is full, or a flag error.
 */
ER obstacle_comms_post_cmd (uint8_t cmd)
{
    ER             result = E_OK;
    uint32_t const type   = obstacle_pack_get(cmd, OBSTACLE_CMD_TYPE_SH, OBSTACLE_CMD_TYPE_MK);

    if (OBSTACLE_CMD_STOP == type)
    {
        result = tk_set_flg(g_links.h_flag, OBSTACLE_FLG_STOP);
    }
    else
    {
        UINT const intsts = lock_take(&g_cmd_lock);
        uint32_t   head   = g_cmd_head;
        uint32_t   count  = g_cmd_count;

        if (count < OBSTACLE_CMD_RING_LEN)
        {
            g_cmd_ring[(head + count) % OBSTACLE_CMD_RING_LEN] = cmd;

            /* Cast is safe: count is below OBSTACLE_CMD_RING_LEN (4). */
            g_cmd_count = (uint8_t)(count + 1U);
        }
        else
        {
            result = E_LIMIT;
        }
        lock_give(&g_cmd_lock, intsts);

        if (E_OK == result)
        {
            result = tk_set_flg(g_links.h_flag, OBSTACLE_FLG_CMD);
        }
    }

    return (result);
}

/*!
 * @brief Take the next queued command.
 *
 * @param[out] p_cmd The command, when one was waiting.
 *
 * @return True if a command was taken.
 */
bool obstacle_comms_pop_cmd (uint8_t * p_cmd)
{
    bool       b_taken = false;
    UINT const intsts  = lock_take(&g_cmd_lock);
    uint32_t   head    = g_cmd_head;
    uint32_t   count   = g_cmd_count;

    if (count > 0U)
    {
        *p_cmd = g_cmd_ring[head];

        /* Casts are safe: both results are below OBSTACLE_CMD_RING_LEN (4). */
        g_cmd_head  = (uint8_t)((head + 1U) % OBSTACLE_CMD_RING_LEN);
        g_cmd_count = (uint8_t)(count - 1U);
        b_taken     = true;
    }
    lock_give(&g_cmd_lock, intsts);

    return (b_taken);
}

/*!
 * @brief Publish one statechart step: emergency stop, status, faults,
 *        profile, events and steering, in that order of urgency.
 *
 * @param[in] p_out Requests from the statechart.
 * @param[in] p_fsm Statechart after the step.
 */
void obstacle_comms_publish (obstacle_statechart_out_t const * p_out, obstacle_statechart_t const * p_fsm)
{
    uint8_t idx = 0U;

    if (p_out->b_call_stop && (NULL != g_links.p_stop))
    {
        (void)g_links.p_stop();
    }

    g_status = obstacle_statechart_status(p_fsm);
    g_faults = p_fsm->faults;

    if (p_out->b_profile_changed)
    {
        publish_profile(p_fsm);
    }

    for (idx = 0U; idx < p_out->evt_count; idx++)
    {
        deliver(p_out->evt[idx], OBSTACLE_FLG_NAV_EVT);
    }

    if (p_out->b_steer_valid)
    {
        deliver(p_out->steer_word, OBSTACLE_FLG_NAV_STEER);
    }
}

/*!
 * @brief Distance travelled, from the motion service if possible.
 *
 * NOTE: Without a reading, the distance is estimated from time at the
 * configured bypass speed, so the distance limits still work (safely
 * early) on the bench.
 *
 * @param[in] now_ms Current time.
 *
 * @return Distance in mm.
 */
uint32_t obstacle_comms_odometer_mm (uint32_t now_ms)
{
    uint32_t                     distance_mm = 0U;
    obstacle_distance_fn_t const p_read      = g_links.p_get_distance_mm;
    bool                         b_have_mm   = false;

    if (NULL != p_read)
    {
        b_have_mm = (E_OK == p_read(&distance_mm));
    }

    if (!b_have_mm)
    {
        distance_mm = ((now_ms / MSEC_PER_SECOND) * OBSTACLE_BYPASS_SPEED_MM_S) +
                      (((now_ms % MSEC_PER_SECOND) * OBSTACLE_BYPASS_SPEED_MM_S) /
                       MSEC_PER_SECOND);
    }

    return (distance_mm);
}

/*!
 * @brief Take the next event from the obstacle task's own outbox.
 *
 * @param[out] p_evt The event.
 * @param[in]  tmout How long to wait (TMO_POL, TMO_FEVR or ms).
 *
 * @return E_OK, E_TMOUT, or a flag error.
 */
ER obstacle_comms_get_event (uint32_t * p_evt, TMO tmout)
{
    ER   result = E_TMOUT;
    UINT ptn    = 0U;
    bool b_done = false;

    while (!b_done)
    {
        if (evt_pop(p_evt))
        {
            result = E_OK;
            b_done = true;
        }
        else if (TMO_POL == tmout)
        {
            b_done = true;
        }
        else
        {
            result = tk_wai_flg(g_links.h_flag, OBSTACLE_FLG_NAV_EVT,
                                TWF_ORW | TWF_BITCLR, &ptn, tmout);
            b_done = (E_OK != result);
        }
    }

    return (result);
}

/*!
 * @brief Latest steering word.
 *
 * @return The packed STEER event word.
 */
uint32_t obstacle_comms_steer (void)
{
    return (g_steer);
}

/*!
 * @brief Latest status word.
 *
 * @return The packed status word.
 */
uint32_t obstacle_comms_status (void)
{
    return (g_status);
}

/*!
 * @brief Latest fault byte.
 *
 * @return The OBSTACLE_FAULT_* bits.
 */
uint8_t obstacle_comms_faults (void)
{
    return (g_faults);
}

/*!
 * @brief Events dropped because an inbox or the outbox was full.
 *
 * @return The count, saturating at 255.
 */
uint8_t obstacle_comms_drops (void)
{
    return (g_drops);
}

/*!
 * @brief Copy the 25-slot profile without waiting.
 *
 * @param[out] p_profile Room for OBSTACLE_SLOT_COUNT packed slots.
 *
 * @return E_OK, or E_BUSY if the writer was busy on both tries.
 */
ER obstacle_comms_copy_profile (uint16_t * p_profile)
{
    ER      result = E_BUSY;
    uint8_t tries  = 0U;

    for (tries = 0U; (tries < PROFILE_TRIES) && (E_OK != result); tries++)
    {
        uint8_t const before = g_profile_seq;
        uint8_t       idx    = 0U;

        if (0U == (before & 1U))
        {
            memory_barrier();
            for (idx = 0U; idx < OBSTACLE_SLOT_COUNT; idx++)
            {
                p_profile[idx] = g_profile[idx];
            }
            memory_barrier();

            if (before == g_profile_seq)
            {
                result = E_OK;
            }
        }
    }

    return (result);
}

/* ------------------------------------------------------------------ *
 * Private functions.
 * ------------------------------------------------------------------ */

/*!
 * @brief Take a lock with interrupts paused on this core.
 *
 * WARNING: Hold the lock for a few statements only.
 *
 * @param[in,out] p_lock The lock.
 *
 * @return The saved interrupt state for lock_give().
 */
static UINT lock_take (comms_lock_t * p_lock)
{
    UINT intsts = 0U;

#if TK_SUPPORT_SMP
    (void)ISpinLock(p_lock, &intsts);
#else
    (void)p_lock;
    DI(intsts);
#endif

    return (intsts);
}

/*!
 * @brief Release a lock and restore interrupts.
 *
 * @param[in,out] p_lock The lock.
 * @param[in]     intsts The value lock_take() returned.
 */
static void lock_give (comms_lock_t * p_lock, UINT intsts)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(p_lock, intsts);
#else
    (void)p_lock;
    EI(intsts);
#endif
}

/*!
 * @brief Make the other core see earlier writes before later ones.
 */
static void memory_barrier (void)
{
#if TK_SUPPORT_SMP
    mp_memory_barrir();
#endif
}

/*!
 * @brief Add one event to the outbox.
 *
 * @param[in] evt Packed event word.
 *
 * @return False if the outbox was full.
 */
static bool evt_push (uint32_t evt)
{
    bool       b_added = false;
    UINT const intsts  = lock_take(&g_evt_lock);
    uint32_t   head    = g_evt_head;
    uint32_t   count   = g_evt_count;

    if (count < OBSTACLE_EVT_RING_LEN)
    {
        g_evt_ring[(head + count) % OBSTACLE_EVT_RING_LEN] = evt;

        /* Cast is safe: count is below OBSTACLE_EVT_RING_LEN (4). */
        g_evt_count = (uint8_t)(count + 1U);
        b_added     = true;
    }
    lock_give(&g_evt_lock, intsts);

    return (b_added);
}

/*!
 * @brief Take one event from the outbox.
 *
 * @param[out] p_evt The event.
 *
 * @return False if the outbox was empty.
 */
static bool evt_pop (uint32_t * p_evt)
{
    bool       b_taken = false;
    UINT const intsts  = lock_take(&g_evt_lock);
    uint32_t   head    = g_evt_head;
    uint32_t   count   = g_evt_count;

    if (count > 0U)
    {
        *p_evt = g_evt_ring[head];

        /* Casts are safe: both results are below OBSTACLE_EVT_RING_LEN (4). */
        g_evt_head  = (uint8_t)((head + 1U) % OBSTACLE_EVT_RING_LEN);
        g_evt_count = (uint8_t)(count - 1U);
        b_taken     = true;
    }
    lock_give(&g_evt_lock, intsts);

    return (b_taken);
}

/*!
 * @brief Count one dropped event, saturating at 255.
 */
static void count_drop (void)
{
    uint8_t const drops = g_drops;

    if (drops < UINT8_MAX)
    {
        /* Cast is safe: drops is below 255. */
        g_drops = (uint8_t)(drops + 1U);
    }
}

/*!
 * @brief Deliver one event to Navigation's inbox or to the outbox. A
 *        steering word replaces the previous one (latest wins).
 *
 * @param[in] evt      Packed event word.
 * @param[in] flag_bit OBSTACLE_FLG_NAV_EVT or OBSTACLE_FLG_NAV_STEER.
 */
static void deliver (uint32_t evt, UINT flag_bit)
{
    ID const h_mbf = g_links.h_nav_mbf;

    if (0 != h_mbf)
    {
        /* Cast is safe: sizeof(evt) is 4, which fits in an INT. */
        if (E_OK != tk_snd_mbf(h_mbf, &evt, (INT)sizeof(evt), TMO_POL))
        {
            count_drop();
        }
    }
    else if (OBSTACLE_FLG_NAV_STEER == flag_bit)
    {
        g_steer = evt;
        (void)tk_set_flg(g_links.h_flag, flag_bit);
    }
    else if (evt_push(evt))
    {
        (void)tk_set_flg(g_links.h_flag, flag_bit);
    }
    else
    {
        count_drop();
    }
}

/*!
 * @brief Copy the profile under the sequence lock.
 *
 * @param[in] p_fsm Statechart holding the new profile.
 */
static void publish_profile (obstacle_statechart_t const * p_fsm)
{
    uint8_t idx = 0U;

    /* Cast is safe: the counter is meant to wrap at 256. */
    g_profile_seq = (uint8_t)(g_profile_seq + 1U);
    memory_barrier();

    for (idx = 0U; idx < OBSTACLE_SLOT_COUNT; idx++)
    {
        g_profile[idx] = p_fsm->scan.profile[idx];
    }

    memory_barrier();

    /* Cast is safe: the counter is meant to wrap at 256. */
    g_profile_seq = (uint8_t)(g_profile_seq + 1U);
}

/*** end of file ***/

