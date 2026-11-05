/** @file statechart.c
 *
 * @brief Buddy 5 statechart (design review section 6.2), pure C99.
 *
 *   STOPPED --RESET (cause cleared), then START--> MONITOR
 *   MONITOR --2 VALID readings <= 250 mm--> WAIT_STOP     (event OBSTACLE)
 *   WAIT_STOP --SCAN_REQ(id)--> SCANNING
 *   SCANNING --profile done--> PLAN                      (event SCAN_RESULT)
 *   PLAN --ACCEPT(id)--> BYPASS, or MONITOR if the result was CLEAR
 *   BYPASS --obstacle out of the forward beam + 3 pings--> REACQUIRE
 *   REACQUIRE --LINE_FOUND(id) after 200 mm--> MONITOR
 *   any state --STOP, fault or time limit--> STOPPED     (event FAULT)
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_config.h"
#include "statechart.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "scan_algo.h"

static uint32_t cnt_get (obstacle_statechart_t const * p_fsm, uint32_t shift, uint32_t mask);
static void     cnt_set (obstacle_statechart_t * p_fsm, uint32_t shift, uint32_t mask,
                         uint32_t value);
static uint32_t cnt_inc (obstacle_statechart_t * p_fsm, uint32_t shift, uint32_t mask);
static uint32_t event_base (uint32_t type, uint32_t req_id);
static void     emit (obstacle_statechart_out_t * p_out, uint32_t evt);
static void emit_steer (obstacle_statechart_t * p_fsm, obstacle_statechart_out_t * p_out, uint8_t steer);
static void enter (obstacle_statechart_t * p_fsm, uint8_t state, uint32_t now_ms);
static uint8_t fault_bit (uint8_t cause);
static void    enter_stopped (obstacle_statechart_t * p_fsm, uint8_t cause, uint32_t now_ms,
                              obstacle_statechart_out_t * p_out);
static bool    is_cause_cleared (obstacle_statechart_t const * p_fsm);
static void    accept_plan (obstacle_statechart_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                            obstacle_statechart_out_t * p_out);
static bool    check_health (obstacle_statechart_t * p_fsm, uint8_t cls, uint32_t now_ms,
                             obstacle_statechart_out_t * p_out);
static bool    check_collision (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                                uint32_t now_ms, obstacle_statechart_out_t * p_out);
static void    ping_monitor (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                             uint32_t now_ms, obstacle_statechart_out_t * p_out);
static void    ping_scanning (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                              uint32_t now_ms, obstacle_statechart_out_t * p_out);
static void    ping_bypass (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
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
void obstacle_statechart_init (obstacle_statechart_t * p_fsm, obstacle_plan_cfg_t const * p_cfg,
                  uint32_t now_ms)
{
    p_fsm->state      = OBSTACLE_STATE_STOPPED;
    p_fsm->cause      = OBSTACLE_CAUSE_BOOT;
    p_fsm->faults     = 0U;
    p_fsm->req_id     = 0U;
    p_fsm->last_class = OBSTACLE_CLASS_UNKNOWN;
    p_fsm->last_dist  = 0U;
    p_fsm->seq        = 0U;
    p_fsm->steer      = OBSTACLE_STEER_HOLD;
    p_fsm->steer_seq  = 0U;
    p_fsm->counters   = 0U;
    p_fsm->b_reset_ok = false;
    p_fsm->since_ms   = now_ms;
    p_fsm->bypass_mm  = 0U;
    p_fsm->scan_progress_ms = now_ms;
    p_fsm->hug_limit_mm     = OBSTACLE_HUG_MIN_MM;
    p_fsm->cfg        = *p_cfg;
    scan_algo_start(&p_fsm->scan);

    p_fsm->plan.status                 = OBSTACLE_RESULT_FAILED;
    p_fsm->plan.side                   = OBSTACLE_SIDE_LEFT;
    p_fsm->plan.width_mm               = 0U;
    p_fsm->plan.start_deg              = 0U;
    p_fsm->plan.closest_mm             = 0U;
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
    p_fsm->state      = OBSTACLE_STATE_STOPPED;
    p_fsm->cause      = OBSTACLE_CAUSE_INIT_FAIL;
    p_fsm->b_reset_ok = false;
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
void obstacle_statechart_command (obstacle_statechart_t * p_fsm, uint8_t cmd, uint32_t now_ms,
                     uint32_t odo_mm, obstacle_statechart_out_t * p_out)
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
                /* Cast is safe: the id field is 5 bits wide. */
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
            /* Navigation sends it only after the obstacle has been passed
             * (obstacle_nav.c), so it is accepted at once. */
            if ((OBSTACLE_STATE_REACQUIRE == state) && b_our_id)
            {
                enter(p_fsm, OBSTACLE_STATE_MONITOR, now_ms);
            }
        break;

        default:
            /* NONE, or a command that is not valid now: ignore it. */
        break;
    }
}

/*!
 * @brief Handle one ping taken where obstacle_statechart_servo_deci() asked.
 *
 * @param[in,out] p_fsm   Statechart.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance (used for VALID only).
 * @param[in]     now_ms  Current time.
 * @param[out]    p_out   Requests for the task.
 */
void obstacle_statechart_ping (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                  uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    uint16_t const dist = (OBSTACLE_CLASS_VALID == cls) ? dist_mm : 0U;

    /* Cast is safe: the value is masked to 6 bits. */
    p_fsm->seq =
        (uint8_t)obstacle_pack_get((uint32_t)p_fsm->seq + 1U, 0U, OBSTACLE_ST_SEQ_MK);
    p_fsm->last_class = cls;
    p_fsm->last_dist  = dist;

    if ((!check_health(p_fsm, cls, now_ms, p_out)) &&
        (!check_collision(p_fsm, cls, dist, now_ms, p_out)))
    {
        switch (p_fsm->state)
        {
            case OBSTACLE_STATE_MONITOR:
                ping_monitor(p_fsm, cls, dist, now_ms, p_out);
            break;

            case OBSTACLE_STATE_SCANNING:
                ping_scanning(p_fsm, cls, dist, now_ms, p_out);
            break;

            case OBSTACLE_STATE_BYPASS:
                ping_bypass(p_fsm, cls, dist, now_ms, p_out);
            break;

            case OBSTACLE_STATE_REACQUIRE:
                emit_steer(p_fsm, p_out,
                           obstacle_plan_hug_steer(p_fsm->plan.side, cls, dist,
                                             p_fsm->steer));
            break;

            default:
                /* STOPPED, WAIT_STOP and PLAN do nothing per ping. */
            break;
        }
    }
}

/*!
 * @brief Check the time limits of the current state.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     now_ms Current time.
 * @param[in]     odo_mm Distance travelled so far.
 * @param[out]    p_out  Requests for the task.
 */
void obstacle_statechart_tick (obstacle_statechart_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                  obstacle_statechart_out_t * p_out)
{
    /* Unsigned subtraction stays correct when the clock wraps. */
    uint32_t const elapsed_ms = now_ms - p_fsm->since_ms;
    uint32_t const scan_age_ms = now_ms - p_fsm->scan_progress_ms;
    uint32_t const hugged_mm  = odo_mm - p_fsm->bypass_mm;
    uint8_t        cause      = OBSTACLE_CAUSE_NONE;

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
                (hugged_mm > p_fsm->hug_limit_mm))
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
 * @brief Where the servo must point for the next ping.
 *
 * @param[in] p_fsm Statechart.
 *
 * @return Angle in deci-degrees, positive = left.
 */
int16_t obstacle_statechart_servo_deci (obstacle_statechart_t const * p_fsm)
{
    int16_t deci = 0;

    if (OBSTACLE_STATE_SCANNING == p_fsm->state)
    {
        uint8_t const slot = scan_algo_target(&p_fsm->scan);

        if (SCAN_ALGO_NO_SLOT != slot)
        {
            deci = scan_algo_slot_deci(slot);
        }
    }
    else if (OBSTACLE_STATE_REACQUIRE == p_fsm->state)
    {
        /* Passing on the left puts the obstacle on our right (negative).
         * Casts are safe: the hug angle is 450. */
        deci = (OBSTACLE_SIDE_LEFT == p_fsm->plan.side)
                   ? (int16_t)(-OBSTACLE_HUG_SERVO_DECI)
                   : (int16_t)OBSTACLE_HUG_SERVO_DECI;
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
 * @return state 3 | cause 4 | class 2 | dist 12 | id 5 | seq 6.
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

/*!
 * @brief Read one packed counter.
 *
 * @param[in] p_fsm Statechart.
 * @param[in] shift Counter position.
 * @param[in] mask  Counter mask.
 *
 * @return Counter value.
 */
static uint32_t cnt_get (obstacle_statechart_t const * p_fsm, uint32_t shift, uint32_t mask)
{
    return (obstacle_pack_get(p_fsm->counters, shift, mask));
}

/*!
 * @brief Write one packed counter.
 *
 * @param[in,out] p_fsm Statechart.
 * @param[in]     shift Counter position.
 * @param[in]     mask  Counter mask.
 * @param[in]     value New value.
 */
static void cnt_set (obstacle_statechart_t * p_fsm, uint32_t shift, uint32_t mask,
                     uint32_t value)
{
    /* Cast is safe: all counters live in bits 0-12. */
    p_fsm->counters =
        (uint16_t)obstacle_pack_put(p_fsm->counters, shift, mask, value);
}

/*!
 * @brief Count up, saturating at the field's maximum.
 *
 * @param[in,out] p_fsm Statechart.
 * @param[in]     shift Counter position.
 * @param[in]     mask  Counter mask.
 *
 * @return The new counter value.
 */
static uint32_t cnt_inc (obstacle_statechart_t * p_fsm, uint32_t shift, uint32_t mask)
{
    uint32_t value = cnt_get(p_fsm, shift, mask);

    if (value < mask)
    {
        value++;
    }

    cnt_set(p_fsm, shift, mask, value);

    return (value);
}

/*!
 * @brief Start an event word.
 *
 * @param[in] type   Event type (OBSTACLE_EVT_*).
 * @param[in] req_id Request id.
 *
 * @return Event word with type and id filled in.
 */
static uint32_t event_base (uint32_t type, uint32_t req_id)
{
    uint32_t word = 0U;

    word = obstacle_pack_put(word, OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK, type);
    word = obstacle_pack_put(word, OBSTACLE_EV_ID_SH, OBSTACLE_EV_ID_MK, req_id);

    return (word);
}

/*!
 * @brief Add an event to the output record.
 *
 * @param[in,out] p_out Output record.
 * @param[in]     evt   Event word.
 */
static void emit (obstacle_statechart_out_t * p_out, uint32_t evt)
{
    if (p_out->evt_count < OBSTACLE_STATECHART_OUT_MAX_EVT)
    {
        p_out->evt[p_out->evt_count] = evt;
        p_out->evt_count++;
    }
}

/*!
 * @brief Publish a steering decision.
 *
 * @param[in,out] p_fsm Statechart.
 * @param[out]    p_out Output record.
 * @param[in]     steer Steering (OBSTACLE_STEER_*).
 */
static void emit_steer (obstacle_statechart_t * p_fsm, obstacle_statechart_out_t * p_out, uint8_t steer)
{
    uint32_t word = event_base(OBSTACLE_EVT_STEER, p_fsm->req_id);

    p_fsm->steer = steer;

    /* Cast is safe: the value is masked to 6 bits. */
    p_fsm->steer_seq = (uint8_t)obstacle_pack_get((uint32_t)p_fsm->steer_seq + 1U, 0U,
                                            OBSTACLE_EV_SSEQ_MK);

    word = obstacle_pack_put(word, OBSTACLE_EV_STEER_SH, OBSTACLE_EV_STEER_MK, steer);
    word = obstacle_pack_put(word, OBSTACLE_EV_SSEQ_SH, OBSTACLE_EV_SSEQ_MK, p_fsm->steer_seq);
    p_out->steer_word    = word;
    p_out->b_steer_valid = true;
}

/*!
 * @brief Enter a state and reset its per-state counters.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     state  New state (OBSTACLE_STATE_*).
 * @param[in]     now_ms Current time.
 */
static void enter (obstacle_statechart_t * p_fsm, uint8_t state, uint32_t now_ms)
{
    p_fsm->state    = state;
    p_fsm->since_ms = now_ms;

    if (OBSTACLE_STATE_SCANNING == state)
    {
        /* The stall timer starts when the scan starts. */
        p_fsm->scan_progress_ms = now_ms;
    }

    cnt_set(p_fsm, OBSTACLE_CNT_HIT_SH, OBSTACLE_CNT_HIT_MK, 0U);
    cnt_set(p_fsm, OBSTACLE_CNT_COLL_SH, OBSTACLE_CNT_COLL_MK, 0U);
    cnt_set(p_fsm, OBSTACLE_CNT_CLEAR_SH, OBSTACLE_CNT_CLEAR_MK, 0U);
}

/*!
 * @brief Fault byte bit for a stop cause.
 *
 * @param[in] cause Stop cause (OBSTACLE_CAUSE_*).
 *
 * @return The fault bit, or 0 when the cause is not a fault.
 */
static uint8_t fault_bit (uint8_t cause)
{
    uint8_t bit = 0U;

    switch (cause)
    {
        case OBSTACLE_CAUSE_NO_ECHO:
            bit = OBSTACLE_FAULT_NO_ECHO;
        break;

        case OBSTACLE_CAUSE_ECHO_STUCK:
            bit = OBSTACLE_FAULT_ECHO_STUCK;
        break;

        case OBSTACLE_CAUSE_SCAN_TIMEOUT:
            bit = OBSTACLE_FAULT_SCAN_TIMEOUT;
        break;

        case OBSTACLE_CAUSE_NO_SAFE_PATH:
            bit = OBSTACLE_FAULT_NO_SAFE_PATH;
        break;

        case OBSTACLE_CAUSE_COLLISION:
            bit = OBSTACLE_FAULT_COLLISION;
        break;

        case OBSTACLE_CAUSE_HANDOVER:
            /* Both handover time limits share one fault bit, so this case
             * deliberately falls through to the next one. */
        case OBSTACLE_CAUSE_PLAN_TIMEOUT:
            bit = OBSTACLE_FAULT_HANDOVER;
        break;

        case OBSTACLE_CAUSE_BYPASS_LIMIT:
            bit = OBSTACLE_FAULT_BYPASS_LIMIT;
        break;

        case OBSTACLE_CAUSE_LINE_TIMEOUT:
            bit = OBSTACLE_FAULT_LINE_TIMEOUT;
        break;

        default:
            /* BOOT, EXTERNAL, PLAN_REJECTED and INIT_FAIL are not faults. */
            bit = 0U;
        break;
    }

    return (bit);
}

/*!
 * @brief Latch STOPPED, tell Navigation why, and stop the car for a fault.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     cause  Stop cause (OBSTACLE_CAUSE_*).
 * @param[in]     now_ms Current time.
 * @param[out]    p_out  Requests for the task.
 */
static void enter_stopped (obstacle_statechart_t * p_fsm, uint8_t cause, uint32_t now_ms,
                           obstacle_statechart_out_t * p_out)
{
    uint8_t const bit = fault_bit(cause);
    uint32_t      evt = event_base(OBSTACLE_EVT_FAULT, p_fsm->req_id);

    if ((OBSTACLE_STATE_BYPASS == p_fsm->state) ||
        (OBSTACLE_STATE_REACQUIRE == p_fsm->state))
    {
        emit_steer(p_fsm, p_out, OBSTACLE_STEER_HOLD);
    }

    p_fsm->cause = cause;

    /* Cast is safe: both operands are 8-bit fault bytes. */
    p_fsm->faults     = (uint8_t)(p_fsm->faults | bit);
    p_fsm->b_reset_ok = false;
    enter(p_fsm, OBSTACLE_STATE_STOPPED, now_ms);

    evt = obstacle_pack_put(evt, OBSTACLE_EV_CAUSE_SH, OBSTACLE_EV_CAUSE_MK, cause);
    evt = obstacle_pack_put(evt, OBSTACLE_EV_FAULTS_SH, OBSTACLE_EV_FAULTS_MK, p_fsm->faults);
    emit(p_out, evt);

    /* Design review 4.1: fault sources raise the direct stop signal. */
    if (0U != bit)
    {
        p_out->b_call_stop = true;
    }
}

/*!
 * @brief Tell whether RESET may clear the current stop cause. A sensor
 *        fault clears only once the sensor reads normally again.
 *
 * @param[in] p_fsm Statechart.
 *
 * @return True if the cause is cleared.
 */
static bool is_cause_cleared (obstacle_statechart_t const * p_fsm)
{
    bool b_cleared = true;

    if ((OBSTACLE_CAUSE_NO_ECHO == p_fsm->cause) ||
        (OBSTACLE_CAUSE_ECHO_STUCK == p_fsm->cause))
    {
        b_cleared = ((OBSTACLE_CLASS_VALID == p_fsm->last_class) ||
                     (OBSTACLE_CLASS_OPEN == p_fsm->last_class)) &&
                    (0U == cnt_get(p_fsm, OBSTACLE_CNT_UNK_SH, OBSTACLE_CNT_UNK_MK)) &&
                    (0U == cnt_get(p_fsm, OBSTACLE_CNT_STUCK_SH, OBSTACLE_CNT_STUCK_MK));
    }
    else if (OBSTACLE_CAUSE_INIT_FAIL == p_fsm->cause)
    {
        /* Only a reboot clears a start-up failure. */
        b_cleared = false;
    }
    else
    {
        b_cleared = true;
    }

    return (b_cleared);
}

/*!
 * @brief Navigation accepted the plan: go back to MONITOR for a CLEAR
 *        result, otherwise start the bypass.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     now_ms Current time.
 * @param[in]     odo_mm Distance travelled so far.
 * @param[out]    p_out  Requests for the task.
 */
static void accept_plan (obstacle_statechart_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                         obstacle_statechart_out_t * p_out)
{
    if (OBSTACLE_RESULT_CLEAR == p_fsm->plan.status)
    {
        enter(p_fsm, OBSTACLE_STATE_MONITOR, now_ms);
    }
    else
    {
        p_fsm->bypass_mm    = odo_mm;
        p_fsm->hug_limit_mm =
            obstacle_plan_hug_limit_mm(&p_fsm->plan, &p_fsm->cfg);
        enter(p_fsm, OBSTACLE_STATE_BYPASS, now_ms);
        emit_steer(p_fsm, p_out, obstacle_plan_steer_away(p_fsm->plan.side));
    }
}

/*!
 * @brief Update the health counters, shared by every state.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     cls    Ping class.
 * @param[in]     now_ms Current time.
 * @param[out]    p_out  Requests for the task.
 *
 * @return True if this ping stopped Buddy 5.
 */
static bool check_health (obstacle_statechart_t * p_fsm, uint8_t cls, uint32_t now_ms,
                          obstacle_statechart_out_t * p_out)
{
    bool     b_stopped = false;
    uint32_t unknown   = 0U;
    uint32_t stuck     = 0U;
    uint8_t  cause     = OBSTACLE_CAUSE_NONE;

    if (OBSTACLE_CLASS_UNKNOWN == cls)
    {
        unknown = cnt_inc(p_fsm, OBSTACLE_CNT_UNK_SH, OBSTACLE_CNT_UNK_MK);
    }
    else
    {
        cnt_set(p_fsm, OBSTACLE_CNT_UNK_SH, OBSTACLE_CNT_UNK_MK, 0U);
    }

    if (OBSTACLE_CLASS_STUCK == cls)
    {
        stuck = cnt_inc(p_fsm, OBSTACLE_CNT_STUCK_SH, OBSTACLE_CNT_STUCK_MK);
    }
    else
    {
        cnt_set(p_fsm, OBSTACLE_CNT_STUCK_SH, OBSTACLE_CNT_STUCK_MK, 0U);
    }

    if (unknown >= OBSTACLE_NO_ECHO_COUNT)
    {
        cause = OBSTACLE_CAUSE_NO_ECHO;
    }
    else if (stuck >= OBSTACLE_STUCK_COUNT)
    {
        cause = OBSTACLE_CAUSE_ECHO_STUCK;
    }
    else
    {
        cause = OBSTACLE_CAUSE_NONE;
    }

    if (OBSTACLE_CAUSE_NONE == cause)
    {
        /* The sensor is healthy. */
    }
    else if (OBSTACLE_STATE_STOPPED != p_fsm->state)
    {
        enter_stopped(p_fsm, cause, now_ms, p_out);
        b_stopped = true;
    }
    else if (p_fsm->b_reset_ok)
    {
        /* The sensor failed again after RESET but before START. */
        p_fsm->b_reset_ok = false;
        p_fsm->cause      = cause;
    }
    else
    {
        /* Already stopped and not reset: nothing more to do. */
    }

    return (b_stopped);
}

/*!
 * @brief Collision guard while the car may be moving.
 *
 * @param[in,out] p_fsm   Statechart.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance.
 * @param[in]     now_ms  Current time.
 * @param[out]    p_out   Requests for the task.
 *
 * @return True if this ping stopped Buddy 5.
 */
static bool check_collision (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                             uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    uint8_t const state     = p_fsm->state;
    bool const    b_guarded = (OBSTACLE_STATE_WAIT_STOP == state) ||
                           (OBSTACLE_STATE_BYPASS == state) ||
                           (OBSTACLE_STATE_REACQUIRE == state);
    bool const b_close = (OBSTACLE_CLASS_VALID == cls) && (dist_mm < OBSTACLE_COLLISION_MM);
    bool       b_stopped = false;

    if (!b_guarded)
    {
        /* The car is stopped or following the line normally. */
    }
    else if (!b_close)
    {
        cnt_set(p_fsm, OBSTACLE_CNT_COLL_SH, OBSTACLE_CNT_COLL_MK, 0U);
    }
    else if (cnt_inc(p_fsm, OBSTACLE_CNT_COLL_SH, OBSTACLE_CNT_COLL_MK) >=
             OBSTACLE_COLLISION_HITS)
    {
        enter_stopped(p_fsm, OBSTACLE_CAUSE_COLLISION, now_ms, p_out);
        b_stopped = true;
    }
    else
    {
        /* One close reading so far: wait for the next one. */
    }

    return (b_stopped);
}

/*!
 * @brief MONITOR: report an obstacle after enough close hits.
 *
 * @param[in,out] p_fsm   Statechart.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance.
 * @param[in]     now_ms  Current time.
 * @param[out]    p_out   Requests for the task.
 */
static void ping_monitor (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                          uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    bool const b_hit = (OBSTACLE_CLASS_VALID == cls) && (dist_mm <= OBSTACLE_DETECT_MM);

    if (!b_hit)
    {
        cnt_set(p_fsm, OBSTACLE_CNT_HIT_SH, OBSTACLE_CNT_HIT_MK, 0U);
    }
    else if (cnt_inc(p_fsm, OBSTACLE_CNT_HIT_SH, OBSTACLE_CNT_HIT_MK) >= OBSTACLE_DETECT_HITS)
    {
        uint32_t evt = event_base(OBSTACLE_EVT_OBSTACLE, p_fsm->req_id);

        evt = obstacle_pack_put(evt, OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK, dist_mm);
        emit(p_out, evt);
        enter(p_fsm, OBSTACLE_STATE_WAIT_STOP, now_ms);
    }
    else
    {
        /* Not enough hits in a row yet. */
    }
}

/*!
 * @brief SCANNING: feed the scan engine, then report the result.
 *
 * @param[in,out] p_fsm   Statechart.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance.
 * @param[in]     now_ms  Current time.
 * @param[out]    p_out   Requests for the task.
 */
static void ping_scanning (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                           uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    uint8_t const target_before = scan_algo_target(&p_fsm->scan);
    bool const    b_done        = scan_algo_feed(&p_fsm->scan, cls, dist_mm);

    p_out->b_profile_changed = true;

    if ((b_done) || (scan_algo_target(&p_fsm->scan) != target_before))
    {
        /* A slot settled: the scan is still making progress. */
        p_fsm->scan_progress_ms = now_ms;
    }

    if (b_done)
    {
        uint32_t evt = event_base(OBSTACLE_EVT_SCAN_RESULT, p_fsm->req_id);

        obstacle_plan_build(&p_fsm->scan, &p_fsm->cfg, &p_fsm->plan);
        evt = obstacle_pack_put(evt, OBSTACLE_EV_STATUS_SH, OBSTACLE_EV_STATUS_MK,
                          p_fsm->plan.status);
        evt = obstacle_pack_put(evt, OBSTACLE_EV_SIDE_SH, OBSTACLE_EV_SIDE_MK, p_fsm->plan.side);
        evt = obstacle_pack_put(evt, OBSTACLE_EV_WIDTH_SH, OBSTACLE_EV_WIDTH_MK,
                          p_fsm->plan.width_mm);
        evt = obstacle_pack_put(evt, OBSTACLE_EV_ANGLE_SH, OBSTACLE_EV_ANGLE_MK,
                          p_fsm->plan.start_deg);
        emit(p_out, evt);

        if (OBSTACLE_RESULT_NO_SAFE_PATH == p_fsm->plan.status)
        {
            enter_stopped(p_fsm, OBSTACLE_CAUSE_NO_SAFE_PATH, now_ms, p_out);
        }
        else
        {
            enter(p_fsm, OBSTACLE_STATE_PLAN, now_ms);
        }
    }
}

/*!
 * @brief BYPASS: bend away until the forward beam has been clear for
 *        OBSTACLE_ESCAPE_CLEAR_PINGS pings in a row, then start the zig-zag
 *        (driving straight until its first ping).
 *
 * @param[in,out] p_fsm   Statechart.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance.
 * @param[in]     now_ms  Current time.
 * @param[out]    p_out   Requests for the task.
 */
static void ping_bypass (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                         uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    uint32_t clear_count = 0U;

    if (obstacle_plan_is_escape_clear(cls, dist_mm, p_fsm->plan.closest_mm))
    {
        clear_count = cnt_inc(p_fsm, OBSTACLE_CNT_CLEAR_SH, OBSTACLE_CNT_CLEAR_MK);
    }
    else
    {
        cnt_set(p_fsm, OBSTACLE_CNT_CLEAR_SH, OBSTACLE_CNT_CLEAR_MK, 0U);
    }

    if (clear_count >= OBSTACLE_ESCAPE_CLEAR_PINGS)
    {
        /* Drive straight while the servo swings to the hug angle (about
         * 155 ms), instead of pivoting on blind; the first hug ping then
         * picks the steering. */
        enter(p_fsm, OBSTACLE_STATE_REACQUIRE, now_ms);
        emit_steer(p_fsm, p_out, OBSTACLE_STEER_STRAIGHT);
    }
    else
    {
        emit_steer(p_fsm, p_out, obstacle_plan_steer_away(p_fsm->plan.side));
    }
}

/*** end of file ***/

