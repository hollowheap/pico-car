/** @file statechart.c
 *
 * @brief Obstacle statechart (S40, D96), pure C99.
 *
 *   STOPPED --RESET (cause cleared), then START--> MONITOR
 *   MONITOR --2 VALID readings <= 250 mm--> WAIT_STOP     (event OBSTACLE)
 *   WAIT_STOP --SCAN_REQ(id)--> SCANNING
 *   SCANNING --profile done--> PLAN                      (event SCAN_RESULT)
 *   PLAN --ACCEPT(id)--> BYPASS, or MONITOR if the result was CLEAR
 *   BYPASS --forward beam clear for 2 pings AND TURNED(id)--> REACQUIRE
 *   REACQUIRE --forward ping < 300 mm--> WAIT_STOP       (event OBSTACLE)
 *   REACQUIRE --LINE_FOUND(id) after grace period--> MONITOR
 *   any state --STOP, fault or time limit--> STOPPED     (event FAULT)
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_config.h"
#include "statechart.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "scan_algo.h"

#define EVT_IDX_PRIMARY   (0U)
#define EVT_IDX_SECONDARY (1U)

static uint32_t cnt_get (obstacle_statechart_t const * p_fsm, uint32_t shift, uint32_t mask);
static void     cnt_set (obstacle_statechart_t * p_fsm, uint32_t shift, uint32_t mask,
                         uint32_t value);
static uint32_t cnt_inc (obstacle_statechart_t * p_fsm, uint32_t shift, uint32_t mask);
static uint32_t event_base (uint32_t type, uint32_t req_id);
static void     emit (obstacle_statechart_out_t * p_out, uint32_t evt);
static void     emit_steer (obstacle_statechart_t * p_fsm,
                            obstacle_statechart_out_t * p_out, uint8_t steer);
static void     enter (obstacle_statechart_t * p_fsm, uint8_t state, uint32_t now_ms);
static uint8_t  fault_bit (uint8_t cause);
static void     enter_stopped (obstacle_statechart_t * p_fsm, uint8_t cause, uint32_t now_ms,
                               obstacle_statechart_out_t * p_out);
static bool     is_cause_cleared (obstacle_statechart_t const * p_fsm);
static void     accept_plan (obstacle_statechart_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                             obstacle_statechart_out_t * p_out);
static bool     check_health (obstacle_statechart_t * p_fsm, uint8_t cls, uint32_t now_ms,
                              obstacle_statechart_out_t * p_out);
static bool     check_collision (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                                 uint32_t now_ms, obstacle_statechart_out_t * p_out);
static void     ping_monitor (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                              uint32_t now_ms, obstacle_statechart_out_t * p_out);
static void     ping_scanning (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                               uint32_t now_ms, obstacle_statechart_out_t * p_out);
static void     ping_bypass (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                             uint32_t now_ms, obstacle_statechart_out_t * p_out);
static void     ping_reacquire (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                                uint32_t now_ms, obstacle_statechart_out_t * p_out);

/*!
 * @brief Clear an output record before a step.
 *
 * @param[out] p_out Output record.
 */
void obstacle_statechart_out_clear (obstacle_statechart_out_t * p_out)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < OBSTACLE_STATECHART_OUT_MAX_EVT; idx++)
    {
        p_out->evt[idx] = 0U;
    }

    p_out->evt_count         = 0U;
    p_out->b_steer_valid     = false;
    p_out->steer_word        = 0U;
    p_out->b_call_stop       = false;
    p_out->b_profile_changed = false;
}

/*!
 * @brief Start the statechart in STOPPED with cause BOOT.
 *
 * @param[out] p_fsm  Statechart.
 * @param[in]  p_cfg  Car and course numbers for the planner.
 * @param[in]  now_ms Current time.
 */
void obstacle_statechart_init (obstacle_statechart_t * p_fsm,
                               obstacle_plan_cfg_t const * p_cfg,
                               uint32_t now_ms)
{
    p_fsm->state            = OBSTACLE_STATE_STOPPED;
    p_fsm->cause            = OBSTACLE_CAUSE_BOOT;
    p_fsm->faults           = 0U;
    p_fsm->req_id           = 0U;
    p_fsm->last_class       = OBSTACLE_CLASS_UNKNOWN;
    p_fsm->last_dist        = 0U;
    p_fsm->seq              = 0U;
    p_fsm->steer            = OBSTACLE_STEER_HOLD;
    p_fsm->steer_seq        = 0U;
    p_fsm->counters         = 0U;
    p_fsm->b_reset_ok       = false;
    p_fsm->since_ms         = now_ms;
    p_fsm->bypass_mm        = 0U;
    p_fsm->scan_progress_ms = now_ms;
    p_fsm->hug_limit_mm     = OBSTACLE_HUG_MIN_MM;
    p_fsm->cfg              = *p_cfg;
    scan_algo_start(&p_fsm->scan);

    p_fsm->plan.status                       = OBSTACLE_RESULT_FAILED;
    p_fsm->plan.side                         = OBSTACLE_SIDE_LEFT;
    p_fsm->plan.width_mm                     = 0U;
    p_fsm->plan.start_deg                    = 0U;
    p_fsm->plan.closest_mm                   = 0U;
    p_fsm->plan.b_wall                       = false;
    p_fsm->b_turned                          = false;
    p_fsm->plan.need_mm[OBSTACLE_SIDE_LEFT]  = OBSTACLE_PLAN_SIDE_BLOCKED;
    p_fsm->plan.need_mm[OBSTACLE_SIDE_RIGHT] = OBSTACLE_PLAN_SIDE_BLOCKED;
}

/*!
 * @brief Start-up failed: latch STOPPED with cause INIT_FAIL. Only a
 *        reboot clears it.
 *
 * @param[in,out] p_fsm Statechart.
 */
void obstacle_statechart_init_failed (obstacle_statechart_t * p_fsm)
{
    p_fsm->state            = OBSTACLE_STATE_STOPPED;
    p_fsm->cause            = OBSTACLE_CAUSE_INIT_FAIL;
    p_fsm->b_reset_ok       = false;
    p_fsm->scan_progress_ms = 0U;
    p_fsm->hug_limit_mm     = OBSTACLE_HUG_MIN_MM;
}

/*!
 * @brief Handle one command from Navigation.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     cmd    Packed command byte (obstacle_pack_cmd()).
 * @param[in]     now_ms Current time.
 * @param[in]     odo_mm Distance travelled so far.
 * @param[out]    p_out  Requests for the task.
 */
void obstacle_statechart_command (obstacle_statechart_t * p_fsm, uint8_t cmd,
                                  uint32_t now_ms, uint32_t odo_mm,
                                  obstacle_statechart_out_t * p_out)
{
    uint32_t const type     = obstacle_pack_get(cmd, OBSTACLE_CMD_TYPE_SH, OBSTACLE_CMD_TYPE_MK);
    uint32_t const req_id   = obstacle_pack_get(cmd, OBSTACLE_CMD_ID_SH, OBSTACLE_CMD_ID_MK);
    uint8_t const  state    = p_fsm->state;
    bool const     b_our_id = (req_id == p_fsm->req_id);

    switch (type)
    {
        case OBSTACLE_CMD_STOP:
            if (OBSTACLE_STATE_STOPPED != state)
            {
                enter_stopped(p_fsm, OBSTACLE_CAUSE_EXTERNAL, now_ms, p_out);
            }
            p_fsm->b_reset_ok = false;
        break;

        case OBSTACLE_CMD_RESET:
            if ((OBSTACLE_STATE_STOPPED == state) && (is_cause_cleared(p_fsm)))
            {
                p_fsm->faults     = 0U;
                p_fsm->cause      = OBSTACLE_CAUSE_NONE;
                p_fsm->b_reset_ok = true;
            }
        break;

        case OBSTACLE_CMD_START:
            if ((OBSTACLE_STATE_STOPPED == state) && (p_fsm->b_reset_ok))
            {
                p_fsm->b_reset_ok = false;
                enter(p_fsm, OBSTACLE_STATE_MONITOR, now_ms);
            }
        break;

        case OBSTACLE_CMD_SCAN_REQ:
            if (OBSTACLE_STATE_WAIT_STOP == state)
            {
                p_fsm->req_id = (uint8_t)req_id;
                scan_algo_start(&p_fsm->scan);
                enter(p_fsm, OBSTACLE_STATE_SCANNING, now_ms);
                p_out->b_profile_changed = true;
            }
        break;

        case OBSTACLE_CMD_ACCEPT:
            if ((OBSTACLE_STATE_PLAN == state) && b_our_id)
            {
                accept_plan(p_fsm, now_ms, odo_mm, p_out);
            }
        break;

        case OBSTACLE_CMD_REJECT:
            if ((OBSTACLE_STATE_PLAN == state) && b_our_id)
            {
                enter_stopped(p_fsm, OBSTACLE_CAUSE_PLAN_REJECTED, now_ms, p_out);
            }
        break;

        case OBSTACLE_CMD_LINE_FOUND:
            if ((OBSTACLE_STATE_REACQUIRE == state) && b_our_id)
            {
                if ((now_ms - p_fsm->since_ms) >= OBSTACLE_LINE_GRACE_MS)
                {
                    enter(p_fsm, OBSTACLE_STATE_MONITOR, now_ms);
                }
            }
            else if ((OBSTACLE_STATE_BYPASS == state) && b_our_id)
            {
                p_fsm->b_turned = true;
            }
            else
            {
                /* Not valid now. */
            }
        break;

        default:
            /* Ignore invalid commands. */
        break;
    }
}

/*!
 * @brief Handle one ping taken where obstacle_statechart_servo_deci() asked.
 *
 * @param[in,out] p_fsm   Statechart.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance.
 * @param[in]     now_ms  Current time.
 * @param[out]    p_out   Requests for the task.
 */
void obstacle_statechart_ping (obstacle_statechart_t * p_fsm, uint8_t cls,
                               uint16_t dist_mm, uint32_t now_ms,
                               obstacle_statechart_out_t * p_out)
{
    uint16_t const dist = (OBSTACLE_CLASS_VALID == cls) ? dist_mm : 0U;

    p_fsm->seq        = (uint8_t)(((uint32_t)p_fsm->seq + 1U) & OBSTACLE_ST_SEQ_MK);
    p_fsm->last_class = cls;
    p_fsm->last_dist  = dist;

    if (check_health(p_fsm, cls, now_ms, p_out))
    {
        /* Fault latched. */
    }
    else if (check_collision(p_fsm, cls, dist_mm, now_ms, p_out))
    {
        /* Collision latched. */
    }
    else
    {
        switch (p_fsm->state)
        {
            case OBSTACLE_STATE_MONITOR:
                ping_monitor(p_fsm, cls, dist_mm, now_ms, p_out);
            break;

            case OBSTACLE_STATE_SCANNING:
                ping_scanning(p_fsm, cls, dist_mm, now_ms, p_out);
            break;

            case OBSTACLE_STATE_BYPASS:
                ping_bypass(p_fsm, cls, dist_mm, now_ms, p_out);
            break;

            case OBSTACLE_STATE_REACQUIRE:
                ping_reacquire(p_fsm, cls, dist_mm, now_ms, p_out);
            break;

            default:
                /* No ping action in other states. */
            break;
        }
    }
}

/*!
 * @brief Check time and distance limits.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     now_ms Current time.
 * @param[in]     odo_mm Odometer reading in mm.
 * @param[out]    p_out  Requests for the task.
 */
void obstacle_statechart_tick (obstacle_statechart_t * p_fsm, uint32_t now_ms,
                               uint32_t odo_mm,
                               obstacle_statechart_out_t * p_out)
{
    uint32_t const elapsed_ms  = now_ms - p_fsm->since_ms;
    uint32_t const scan_age_ms = now_ms - p_fsm->scan_progress_ms;
    uint32_t const hugged_mm   = odo_mm - p_fsm->bypass_mm;
    uint8_t        cause       = OBSTACLE_CAUSE_NONE;

    switch (p_fsm->state)
    {
        case OBSTACLE_STATE_WAIT_STOP:
            if (elapsed_ms > OBSTACLE_WAIT_STOP_MS)
            {
                cause = OBSTACLE_CAUSE_HANDOVER;
            }
        break;

        case OBSTACLE_STATE_SCANNING:
            if (scan_age_ms > OBSTACLE_SCAN_STALL_MS)
            {
                cause = OBSTACLE_CAUSE_SCAN_TIMEOUT;
            }
        break;

        case OBSTACLE_STATE_PLAN:
            if (elapsed_ms > OBSTACLE_PLAN_ACCEPT_MS)
            {
                cause = OBSTACLE_CAUSE_PLAN_TIMEOUT;
            }
        break;

        case OBSTACLE_STATE_BYPASS:
            if (elapsed_ms > OBSTACLE_ESCAPE_MAX_MS)
            {
                cause = OBSTACLE_CAUSE_BYPASS_LIMIT;
            }
        break;

        case OBSTACLE_STATE_REACQUIRE:
            if ((elapsed_ms > OBSTACLE_HUG_MAX_MS) ||
                ((!p_fsm->plan.b_wall) && (hugged_mm > p_fsm->hug_limit_mm)))
            {
                cause = OBSTACLE_CAUSE_LINE_TIMEOUT;
            }
        break;

        default:
            cause = OBSTACLE_CAUSE_NONE;
        break;
    }

    if (OBSTACLE_CAUSE_NONE != cause)
    {
        enter_stopped(p_fsm, cause, now_ms, p_out);
    }
}

/*!
 * @brief Where the servo must point for the next ping (S40, D96).
 *
 * @param[in] p_fsm Statechart.
 *
 * @return Angle in deci-degrees, clamped to +-OBSTACLE_SERVO_LIMIT_DECI (+-30 deg).
 */
int16_t obstacle_statechart_servo_deci (obstacle_statechart_t const * p_fsm)
{
    int16_t deci = 0;

    if (OBSTACLE_STATE_SCANNING == p_fsm->state)
    {
        deci = scan_algo_servo_deci(&p_fsm->scan);
    }
    else
    {
        /* While moving or waiting, the sonar faces forward. */
        deci = 0;
    }

    if (deci > (int16_t)OBSTACLE_SERVO_LIMIT_DECI)
    {
        deci = (int16_t)OBSTACLE_SERVO_LIMIT_DECI;
    }
    else if (deci < -(int16_t)OBSTACLE_SERVO_LIMIT_DECI)
    {
        deci = -(int16_t)OBSTACLE_SERVO_LIMIT_DECI;
    }
    else
    {
        /* Within limits. */
    }

    return (deci);
}

/*!
 * @brief Body pivot angle required by the stationary scan (S40, D96).
 *
 * @param[in] p_fsm Statechart.
 *
 * @return Angle in deci-degrees from original line heading.
 */
int16_t obstacle_statechart_body_deci (obstacle_statechart_t const * p_fsm)
{
    int16_t deci = 0;

    if (OBSTACLE_STATE_SCANNING == p_fsm->state)
    {
        deci = scan_algo_body_deci(&p_fsm->scan);
    }
    else
    {
        deci = 0;
    }

    return (deci);
}

/*!
 * @brief Build the packed 32-bit status word.
 *
 * @param[in] p_fsm Statechart.
 *
 * @return Status word.
 */
uint32_t obstacle_statechart_status (obstacle_statechart_t const * p_fsm)
{
    uint32_t word = 0U;

    word = obstacle_pack_put(word, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK, p_fsm->state);
    word = obstacle_pack_put(word, OBSTACLE_ST_CAUSE_SH, OBSTACLE_ST_CAUSE_MK, p_fsm->cause);
    word = obstacle_pack_put(word, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK, p_fsm->last_class);
    word = obstacle_pack_put(word, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK, p_fsm->last_dist);
    word = obstacle_pack_put(word, OBSTACLE_ST_ID_SH, OBSTACLE_ST_ID_MK, p_fsm->req_id);
    word = obstacle_pack_put(word, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK, p_fsm->seq);

    return (word);
}

static uint32_t cnt_get (obstacle_statechart_t const * p_fsm, uint32_t shift, uint32_t mask)
{
    return (obstacle_pack_get(p_fsm->counters, shift, mask));
}

static void cnt_set (obstacle_statechart_t * p_fsm, uint32_t shift, uint32_t mask,
                     uint32_t value)
{
    p_fsm->counters =
        (uint16_t)obstacle_pack_put(p_fsm->counters, shift, mask, value);
}

static uint32_t cnt_inc (obstacle_statechart_t * p_fsm, uint32_t shift, uint32_t mask)
{
    uint32_t count = cnt_get(p_fsm, shift, mask);

    if (count < mask)
    {
        count++;
        cnt_set(p_fsm, shift, mask, count);
    }

    return (count);
}

static uint32_t event_base (uint32_t type, uint32_t req_id)
{
    uint32_t word = 0U;

    word = obstacle_pack_put(word, OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK, type);
    word = obstacle_pack_put(word, OBSTACLE_EV_ID_SH, OBSTACLE_EV_ID_MK, req_id);

    return (word);
}

static void emit (obstacle_statechart_out_t * p_out, uint32_t evt)
{
    if (p_out->evt_count < OBSTACLE_STATECHART_OUT_MAX_EVT)
    {
        p_out->evt[p_out->evt_count] = evt;
        p_out->evt_count++;
    }
}

static void emit_steer (obstacle_statechart_t * p_fsm,
                        obstacle_statechart_out_t * p_out, uint8_t steer)
{
    uint32_t word = 0U;

    p_fsm->steer     = steer;
    p_fsm->steer_seq = (uint8_t)(((uint32_t)p_fsm->steer_seq + 1U) & OBSTACLE_EV_SSEQ_MK);

    word = event_base(OBSTACLE_EVT_STEER, p_fsm->req_id);
    word = obstacle_pack_put(word, OBSTACLE_EV_STEER_SH, OBSTACLE_EV_STEER_MK, steer);
    word = obstacle_pack_put(word, OBSTACLE_EV_SSEQ_SH, OBSTACLE_EV_SSEQ_MK, p_fsm->steer_seq);

    p_out->steer_word    = word;
    p_out->b_steer_valid = true;
}

static void enter (obstacle_statechart_t * p_fsm, uint8_t state, uint32_t now_ms)
{
    p_fsm->state    = state;
    p_fsm->since_ms = now_ms;
    p_fsm->counters = 0U;
    if (OBSTACLE_STATE_SCANNING == state)
    {
        p_fsm->scan_progress_ms = now_ms;
    }
}

static uint8_t fault_bit (uint8_t cause)
{
    uint8_t bit = 0U;

    switch (cause)
    {
        case OBSTACLE_CAUSE_NO_ECHO:
            bit = (uint8_t)OBSTACLE_FAULT_NO_ECHO;
        break;

        case OBSTACLE_CAUSE_ECHO_STUCK:
            bit = (uint8_t)OBSTACLE_FAULT_ECHO_STUCK;
        break;

        case OBSTACLE_CAUSE_SCAN_TIMEOUT:
            bit = (uint8_t)OBSTACLE_FAULT_SCAN_TIMEOUT;
        break;

        case OBSTACLE_CAUSE_NO_SAFE_PATH:
            bit = (uint8_t)OBSTACLE_FAULT_NO_SAFE_PATH;
        break;

        case OBSTACLE_CAUSE_COLLISION:
            bit = (uint8_t)OBSTACLE_FAULT_COLLISION;
        break;

        case OBSTACLE_CAUSE_HANDOVER:
            bit = (uint8_t)OBSTACLE_FAULT_HANDOVER;
        break;

        case OBSTACLE_CAUSE_BYPASS_LIMIT:
            bit = (uint8_t)OBSTACLE_FAULT_BYPASS_LIMIT;
        break;

        case OBSTACLE_CAUSE_LINE_TIMEOUT:
            bit = (uint8_t)OBSTACLE_FAULT_LINE_TIMEOUT;
        break;

        default:
            bit = 0U;
        break;
    }

    return (bit);
}

static void enter_stopped (obstacle_statechart_t * p_fsm, uint8_t cause,
                           uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    uint8_t const  bit  = fault_bit(cause);
    uint32_t       word = 0U;

    enter(p_fsm, OBSTACLE_STATE_STOPPED, now_ms);
    p_fsm->cause      = cause;
    p_fsm->faults    |= bit;
    p_fsm->b_reset_ok = false;

    word = event_base(OBSTACLE_EVT_FAULT, p_fsm->req_id);
    word = obstacle_pack_put(word, OBSTACLE_EV_CAUSE_SH, OBSTACLE_EV_CAUSE_MK, cause);
    word = obstacle_pack_put(word, OBSTACLE_EV_FAULTS_SH, OBSTACLE_EV_FAULTS_MK, p_fsm->faults);
    emit(p_out, word);

    p_out->b_call_stop = (0U != bit);
}

static bool is_cause_cleared (obstacle_statechart_t const * p_fsm)
{
    return (OBSTACLE_CAUSE_INIT_FAIL != p_fsm->cause);
}

static void accept_plan (obstacle_statechart_t * p_fsm, uint32_t now_ms,
                         uint32_t odo_mm, obstacle_statechart_out_t * p_out)
{
    if (OBSTACLE_RESULT_CLEAR == p_fsm->plan.status)
    {
        enter(p_fsm, OBSTACLE_STATE_MONITOR, now_ms);
    }
    else
    {
        p_fsm->b_turned     = false;
        p_fsm->bypass_mm    = odo_mm;
        p_fsm->hug_limit_mm = obstacle_plan_hug_limit_mm(&p_fsm->plan, &p_fsm->cfg);
        enter(p_fsm, OBSTACLE_STATE_BYPASS, now_ms);
        emit_steer(p_fsm, p_out, obstacle_plan_steer_away(p_fsm->plan.side));
    }
}

static bool check_health (obstacle_statechart_t * p_fsm, uint8_t cls,
                          uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    bool b_fault = false;

    if (OBSTACLE_CLASS_UNKNOWN == cls)
    {
        uint32_t const unk = cnt_inc(p_fsm, OBSTACLE_CNT_UNK_SH, OBSTACLE_CNT_UNK_MK);

        if (unk >= OBSTACLE_NO_ECHO_COUNT)
        {
            enter_stopped(p_fsm, OBSTACLE_CAUSE_NO_ECHO, now_ms, p_out);
            b_fault = true;
        }
    }
    else
    {
        cnt_set(p_fsm, OBSTACLE_CNT_UNK_SH, OBSTACLE_CNT_UNK_MK, 0U);
    }

    if ((!b_fault) && (OBSTACLE_CLASS_STUCK == cls))
    {
        uint32_t const stk = cnt_inc(p_fsm, OBSTACLE_CNT_STUCK_SH, OBSTACLE_CNT_STUCK_MK);

        if (stk >= OBSTACLE_STUCK_COUNT)
        {
            enter_stopped(p_fsm, OBSTACLE_CAUSE_ECHO_STUCK, now_ms, p_out);
            b_fault = true;
        }
    }
    else if (!b_fault)
    {
        cnt_set(p_fsm, OBSTACLE_CNT_STUCK_SH, OBSTACLE_CNT_STUCK_MK, 0U);
    }
    else
    {
        /* Fault already entered. */
    }

    return (b_fault);
}

static bool check_collision (obstacle_statechart_t * p_fsm, uint8_t cls,
                             uint16_t dist_mm, uint32_t now_ms,
                             obstacle_statechart_out_t * p_out)
{
    bool b_fault = false;

    if ((OBSTACLE_CLASS_VALID == cls) && (dist_mm <= OBSTACLE_COLLISION_MM))
    {
        uint32_t const hits = cnt_inc(p_fsm, OBSTACLE_CNT_COLL_SH, OBSTACLE_CNT_COLL_MK);

        if (hits >= OBSTACLE_COLLISION_HITS)
        {
            enter_stopped(p_fsm, OBSTACLE_CAUSE_COLLISION, now_ms, p_out);
            b_fault = true;
        }
    }
    else
    {
        cnt_set(p_fsm, OBSTACLE_CNT_COLL_SH, OBSTACLE_CNT_COLL_MK, 0U);
    }

    return (b_fault);
}

static void ping_monitor (obstacle_statechart_t * p_fsm, uint8_t cls,
                          uint16_t dist_mm, uint32_t now_ms,
                          obstacle_statechart_out_t * p_out)
{
    if ((OBSTACLE_CLASS_VALID == cls) && (dist_mm <= OBSTACLE_DETECT_MM))
    {
        uint32_t const hits = cnt_inc(p_fsm, OBSTACLE_CNT_HIT_SH, OBSTACLE_CNT_HIT_MK);

        if (hits >= OBSTACLE_DETECT_HITS)
        {
            uint32_t word = 0U;

            enter(p_fsm, OBSTACLE_STATE_WAIT_STOP, now_ms);
            word = event_base(OBSTACLE_EVT_OBSTACLE, p_fsm->req_id);
            word = obstacle_pack_put(word, OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK, dist_mm);
            emit(p_out, word);
        }
    }
    else
    {
        cnt_set(p_fsm, OBSTACLE_CNT_HIT_SH, OBSTACLE_CNT_HIT_MK, 0U);
    }
}

static void ping_scanning (obstacle_statechart_t * p_fsm, uint8_t cls,
                           uint16_t dist_mm, uint32_t now_ms,
                           obstacle_statechart_out_t * p_out)
{
    bool const b_done = scan_algo_feed(&p_fsm->scan, cls, dist_mm);

    p_fsm->scan_progress_ms = now_ms;

    if (b_done)
    {
        uint32_t word = 0U;

        obstacle_plan_build(&p_fsm->scan, &p_fsm->cfg, &p_fsm->plan);
        enter(p_fsm, OBSTACLE_STATE_PLAN, now_ms);

        word = event_base(OBSTACLE_EVT_SCAN_RESULT, p_fsm->req_id);
        word = obstacle_pack_put(word, OBSTACLE_EV_STATUS_SH, OBSTACLE_EV_STATUS_MK,
                                 p_fsm->plan.status);
        word = obstacle_pack_put(word, OBSTACLE_EV_SIDE_SH, OBSTACLE_EV_SIDE_MK,
                                 p_fsm->plan.side);
        word = obstacle_pack_put(word, OBSTACLE_EV_WIDTH_SH, OBSTACLE_EV_WIDTH_MK,
                                 p_fsm->plan.width_mm);
        word = obstacle_pack_put(word, OBSTACLE_EV_ANGLE_SH, OBSTACLE_EV_ANGLE_MK,
                                 p_fsm->plan.start_deg);
        word = obstacle_pack_put(word, OBSTACLE_EV_WALL_SH, OBSTACLE_EV_WALL_MK,
                                 p_fsm->plan.b_wall ? 1U : 0U);
        emit(p_out, word);
    }
}

static void ping_bypass (obstacle_statechart_t * p_fsm, uint8_t cls,
                         uint16_t dist_mm, uint32_t now_ms,
                         obstacle_statechart_out_t * p_out)
{
    if (obstacle_plan_is_escape_clear(cls, dist_mm, p_fsm->plan.closest_mm))
    {
        uint32_t const clears = cnt_inc(p_fsm, OBSTACLE_CNT_CLEAR_SH, OBSTACLE_CNT_CLEAR_MK);

        if ((clears >= OBSTACLE_ESCAPE_CLEARS) && (p_fsm->b_turned))
        {
            enter(p_fsm, OBSTACLE_STATE_REACQUIRE, now_ms);
            emit_steer(p_fsm, p_out, OBSTACLE_STEER_STRAIGHT);
        }
        else
        {
            emit_steer(p_fsm, p_out, obstacle_plan_steer_away(p_fsm->plan.side));
        }
    }
    else
    {
        cnt_set(p_fsm, OBSTACLE_CNT_CLEAR_SH, OBSTACLE_CNT_CLEAR_MK, 0U);
        emit_steer(p_fsm, p_out, obstacle_plan_steer_away(p_fsm->plan.side));
    }
}

static void ping_reacquire (obstacle_statechart_t * p_fsm, uint8_t cls,
                            uint16_t dist_mm, uint32_t now_ms,
                            obstacle_statechart_out_t * p_out)
{
    /* S40, D96: While moving, the sonar faces forward (0 deg). Any VALID
     * reading under OBSTACLE_FWD_GUARD_MM (300 mm) stops the car and rescans. */
    if ((OBSTACLE_CLASS_VALID == cls) && (dist_mm < OBSTACLE_FWD_GUARD_MM))
    {
        uint32_t word = 0U;

        enter(p_fsm, OBSTACLE_STATE_WAIT_STOP, now_ms);
        word = event_base(OBSTACLE_EVT_OBSTACLE, p_fsm->req_id);
        word = obstacle_pack_put(word, OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK, dist_mm);
        emit(p_out, word);
        emit_steer(p_fsm, p_out, OBSTACLE_STEER_HOLD);
    }
}

/*** end of file ***/
