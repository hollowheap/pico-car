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

#include "config.h"
#include "statechart.h"
#include "packed.h"
#include "planner.h"
#include "scan.h"

static uint32_t cnt_get (extras_fsm_t const * p_fsm, uint32_t shift, uint32_t mask);
static void     cnt_set (extras_fsm_t * p_fsm, uint32_t shift, uint32_t mask,
                         uint32_t value);
static uint32_t cnt_inc (extras_fsm_t * p_fsm, uint32_t shift, uint32_t mask);
static uint32_t event_base (uint32_t type, uint32_t req_id);
static void     emit (extras_fsm_out_t * p_out, uint32_t evt);
static void emit_steer (extras_fsm_t * p_fsm, extras_fsm_out_t * p_out, uint8_t steer);
static void enter (extras_fsm_t * p_fsm, uint8_t state, uint32_t now_ms);
static uint8_t fault_bit (uint8_t cause);
static void    enter_stopped (extras_fsm_t * p_fsm, uint8_t cause, uint32_t now_ms,
                              extras_fsm_out_t * p_out);
static bool    is_cause_cleared (extras_fsm_t const * p_fsm);
static void    accept_plan (extras_fsm_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                            extras_fsm_out_t * p_out);
static bool    check_health (extras_fsm_t * p_fsm, uint8_t cls, uint32_t now_ms,
                             extras_fsm_out_t * p_out);
static void    ping_monitor (extras_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                             uint32_t now_ms, extras_fsm_out_t * p_out);
static void    ping_scanning (extras_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                              uint32_t now_ms, extras_fsm_out_t * p_out);
static void    ping_bypass (extras_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                            uint32_t now_ms, extras_fsm_out_t * p_out);

/*!
 * @brief Clear an output record before a step.
 *
 * @param[out] p_out Output record.
 */
void extras_fsm_out_clear (extras_fsm_out_t * p_out)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < EXTRAS_FSM_OUT_MAX_EVT; idx++)
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
void extras_fsm_init (extras_fsm_t * p_fsm, extras_plan_cfg_t const * p_cfg,
                  uint32_t now_ms)
{
    p_fsm->state       = EXTRAS_STATE_STOPPED;
    p_fsm->cause       = EXTRAS_CAUSE_BOOT;
    p_fsm->faults      = 0U;
    p_fsm->req_id      = 0U;
    p_fsm->last_class  = EXTRAS_CLASS_UNKNOWN;
    p_fsm->last_dist   = 0U;
    p_fsm->seq         = 0U;
    p_fsm->steer       = EXTRAS_STEER_HOLD;
    p_fsm->steer_seq   = 0U;
    p_fsm->counters    = 0U;
    p_fsm->b_reset_ok  = false;
    p_fsm->b_guard_off = false;
    p_fsm->since_ms    = now_ms;
    p_fsm->bypass_mm   = 0U;
    p_fsm->cfg         = *p_cfg;
    extras_scan_start(&p_fsm->scan);

    p_fsm->plan.status                 = EXTRAS_RESULT_FAILED;
    p_fsm->plan.side                   = EXTRAS_SIDE_LEFT;
    p_fsm->plan.width_mm               = 0U;
    p_fsm->plan.start_deg              = 0U;
    p_fsm->plan.closest_mm             = 0U;
    p_fsm->plan.need_mm[EXTRAS_SIDE_LEFT]  = EXTRAS_PLAN_SIDE_BLOCKED;
    p_fsm->plan.need_mm[EXTRAS_SIDE_RIGHT] = EXTRAS_PLAN_SIDE_BLOCKED;
}

/*!
 * @brief Start-up failed: latch STOPPED with cause INIT_FAIL. Only a
 *        reboot clears it.
 *
 * @param[in,out] p_fsm Statechart.
 */
void extras_fsm_init_failed (extras_fsm_t * p_fsm)
{
    p_fsm->state      = EXTRAS_STATE_STOPPED;
    p_fsm->cause      = EXTRAS_CAUSE_INIT_FAIL;
    p_fsm->b_reset_ok = false;
}

/*!
 * @brief Handle one command from Navigation.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     cmd    Packed command byte (extras_pack_cmd()).
 * @param[in]     now_ms Current time.
 * @param[in]     odo_mm Distance travelled so far.
 * @param[out]    p_out  Requests for the task.
 */
void extras_fsm_command (extras_fsm_t * p_fsm, uint8_t cmd, uint32_t now_ms,
                     uint32_t odo_mm, extras_fsm_out_t * p_out)
{
    uint32_t const type     = extras_pack_get(cmd, EXTRAS_CMD_TYPE_SH, EXTRAS_CMD_TYPE_MK);
    uint32_t const req_id   = extras_pack_get(cmd, EXTRAS_CMD_ID_SH, EXTRAS_CMD_ID_MK);
    uint8_t const  state    = p_fsm->state;
    bool const     b_our_id = (req_id == p_fsm->req_id);

    switch (type)
    {
        case EXTRAS_CMD_STOP:
            if (EXTRAS_STATE_STOPPED != state)
            {
                enter_stopped(p_fsm, EXTRAS_CAUSE_EXTERNAL, now_ms, p_out);
            }

            p_fsm->b_reset_ok = false;
        break;

        case EXTRAS_CMD_RESET:
            if ((EXTRAS_STATE_STOPPED == state) && (is_cause_cleared(p_fsm)))
            {
                p_fsm->faults     = 0U;
                p_fsm->cause      = EXTRAS_CAUSE_NONE;
                p_fsm->b_reset_ok = true;
            }
        break;

        case EXTRAS_CMD_START:
            if ((EXTRAS_STATE_STOPPED == state) && (p_fsm->b_reset_ok))
            {
                p_fsm->b_reset_ok = false;
                enter(p_fsm, EXTRAS_STATE_MONITOR, now_ms);
            }
        break;

        case EXTRAS_CMD_SCAN_REQ:
            if (EXTRAS_STATE_WAIT_STOP == state)
            {
                /* Cast is safe: the id field is 5 bits wide. */
                p_fsm->req_id = (uint8_t)req_id;
                extras_scan_start(&p_fsm->scan);
                enter(p_fsm, EXTRAS_STATE_SCANNING, now_ms);
                p_out->b_profile_changed = true;
            }
        break;

        case EXTRAS_CMD_ACCEPT:
            if ((EXTRAS_STATE_PLAN == state) && b_our_id)
            {
                accept_plan(p_fsm, now_ms, odo_mm, p_out);
            }
        break;

        case EXTRAS_CMD_REJECT:
            if ((EXTRAS_STATE_PLAN == state) && b_our_id)
            {
                enter_stopped(p_fsm, EXTRAS_CAUSE_PLAN_REJECTED, now_ms, p_out);
            }
        break;

        case EXTRAS_CMD_LINE_FOUND:
            /* Navigation sends it only after the obstacle has been passed
             * (extras_nav.c), so it is accepted at once. */
            if ((EXTRAS_STATE_REACQUIRE == state) && b_our_id)
            {
                enter(p_fsm, EXTRAS_STATE_MONITOR, now_ms);
            }
        break;

        default:
            /* NONE, or a command that is not valid now: ignore it. */
        break;
    }
}

/*!
 * @brief Handle one ping taken where extras_fsm_servo_deci() asked.
 *
 * @param[in,out] p_fsm   Statechart.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance (used for VALID only).
 * @param[in]     now_ms  Current time.
 * @param[out]    p_out   Requests for the task.
 */
void extras_fsm_ping (extras_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                  uint32_t now_ms, extras_fsm_out_t * p_out)
{
    uint16_t const dist = (EXTRAS_CLASS_VALID == cls) ? dist_mm : 0U;

    /* Cast is safe: the value is masked to 6 bits. */
    p_fsm->seq =
        (uint8_t)extras_pack_get((uint32_t)p_fsm->seq + 1U, 0U, EXTRAS_ST_SEQ_MK);
    p_fsm->last_class = cls;
    p_fsm->last_dist  = dist;

    if (!check_health(p_fsm, cls, now_ms, p_out))
    {
        switch (p_fsm->state)
        {
            case EXTRAS_STATE_MONITOR:
                ping_monitor(p_fsm, cls, dist, now_ms, p_out);
            break;

            case EXTRAS_STATE_SCANNING:
                ping_scanning(p_fsm, cls, dist, now_ms, p_out);
            break;

            case EXTRAS_STATE_BYPASS:
                ping_bypass(p_fsm, cls, dist, now_ms, p_out);
            break;

            case EXTRAS_STATE_REACQUIRE:
                /* Navigation steers the bypass from the status word; the
                 * STEER word only says "straight on". */
                emit_steer(p_fsm, p_out, EXTRAS_STEER_STRAIGHT);
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
void extras_fsm_tick (extras_fsm_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                  extras_fsm_out_t * p_out)
{
    /* Unsigned subtraction stays correct when the clock wraps. The car may
     * back up before its first pivot, so the odometer can be below its
     * value at ACCEPT: that counts as no distance yet. */
    uint32_t const elapsed_ms = now_ms - p_fsm->since_ms;
    uint32_t const bypass_mm =
        (odo_mm >= p_fsm->bypass_mm) ? (odo_mm - p_fsm->bypass_mm) : 0U;
    uint8_t cause = EXTRAS_CAUSE_NONE;

    switch (p_fsm->state)
    {
        case EXTRAS_STATE_WAIT_STOP:
            if (elapsed_ms > EXTRAS_WAIT_STOP_MS)
            {
                cause = EXTRAS_CAUSE_HANDOVER;
            }
        break;

        case EXTRAS_STATE_SCANNING:
            if (elapsed_ms > EXTRAS_SCAN_TIMEOUT_MS)
            {
                cause = EXTRAS_CAUSE_SCAN_TIMEOUT;
            }
        break;

        case EXTRAS_STATE_PLAN:
            if (elapsed_ms > EXTRAS_PLAN_ACCEPT_MS)
            {
                cause = EXTRAS_CAUSE_PLAN_TIMEOUT;
            }
        break;

        case EXTRAS_STATE_BYPASS:
            if (elapsed_ms > EXTRAS_ESCAPE_MAX_MS)
            {
                cause = EXTRAS_CAUSE_BYPASS_LIMIT;
            }
        break;

        case EXTRAS_STATE_REACQUIRE:
            if ((elapsed_ms > EXTRAS_BYPASS_MAX_MS) ||
                (bypass_mm > EXTRAS_BYPASS_MAX_MM))
            {
                cause = EXTRAS_CAUSE_LINE_TIMEOUT;
            }
        break;

        default:
            cause = EXTRAS_CAUSE_NONE;
        break;
    }

    if (EXTRAS_CAUSE_NONE != cause)
    {
        enter_stopped(p_fsm, cause, now_ms, p_out);
    }
}

/*!
 * @brief Where the servo must point for the next ping.
 *
 * @param[in] p_fsm     Statechart.
 * @param[in] look_deci Angle Navigation asked for (obstacle_extras_set_look()),
 *                      used in REACQUIRE.
 *
 * @return Angle in deci-degrees, positive = left.
 */
int16_t extras_fsm_servo_deci (extras_fsm_t const * p_fsm, int16_t look_deci)
{
    int16_t deci = 0;

    if (EXTRAS_STATE_SCANNING == p_fsm->state)
    {
        uint8_t const slot = extras_scan_target(&p_fsm->scan);

        if (EXTRAS_SCAN_NO_SLOT != slot)
        {
            deci = extras_scan_slot_deci(slot);
        }
    }
    else if (EXTRAS_STATE_REACQUIRE == p_fsm->state)
    {
        /* The bypass needs a different angle in each part (straight ahead
         * while driving, a full sweep at each side scan), so Navigation
         * chooses. */
        deci = look_deci;
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
uint32_t extras_fsm_status (extras_fsm_t const * p_fsm)
{
    uint32_t word = 0U;

    word = extras_pack_put(word, EXTRAS_ST_STATE_SH, EXTRAS_ST_STATE_MK, p_fsm->state);
    word = extras_pack_put(word, EXTRAS_ST_CAUSE_SH, EXTRAS_ST_CAUSE_MK, p_fsm->cause);
    word = extras_pack_put(word, EXTRAS_ST_CLASS_SH, EXTRAS_ST_CLASS_MK, p_fsm->last_class);
    word = extras_pack_put(word, EXTRAS_ST_DIST_SH, EXTRAS_ST_DIST_MK, p_fsm->last_dist);
    word = extras_pack_put(word, EXTRAS_ST_ID_SH, EXTRAS_ST_ID_MK, p_fsm->req_id);
    word = extras_pack_put(word, EXTRAS_ST_SEQ_SH, EXTRAS_ST_SEQ_MK, p_fsm->seq);

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
static uint32_t cnt_get (extras_fsm_t const * p_fsm, uint32_t shift, uint32_t mask)
{
    return (extras_pack_get(p_fsm->counters, shift, mask));
}

/*!
 * @brief Write one packed counter.
 *
 * @param[in,out] p_fsm Statechart.
 * @param[in]     shift Counter position.
 * @param[in]     mask  Counter mask.
 * @param[in]     value New value.
 */
static void cnt_set (extras_fsm_t * p_fsm, uint32_t shift, uint32_t mask,
                     uint32_t value)
{
    /* Cast is safe: all counters live in bits 0-12. */
    p_fsm->counters =
        (uint16_t)extras_pack_put(p_fsm->counters, shift, mask, value);
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
static uint32_t cnt_inc (extras_fsm_t * p_fsm, uint32_t shift, uint32_t mask)
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
 * @param[in] type   Event type (EXTRAS_EVT_*).
 * @param[in] req_id Request id.
 *
 * @return Event word with type and id filled in.
 */
static uint32_t event_base (uint32_t type, uint32_t req_id)
{
    uint32_t word = 0U;

    word = extras_pack_put(word, EXTRAS_EV_TYPE_SH, EXTRAS_EV_TYPE_MK, type);
    word = extras_pack_put(word, EXTRAS_EV_ID_SH, EXTRAS_EV_ID_MK, req_id);

    return (word);
}

/*!
 * @brief Add an event to the output record.
 *
 * @param[in,out] p_out Output record.
 * @param[in]     evt   Event word.
 */
static void emit (extras_fsm_out_t * p_out, uint32_t evt)
{
    if (p_out->evt_count < EXTRAS_FSM_OUT_MAX_EVT)
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
 * @param[in]     steer Steering (EXTRAS_STEER_*).
 */
static void emit_steer (extras_fsm_t * p_fsm, extras_fsm_out_t * p_out, uint8_t steer)
{
    uint32_t word = event_base(EXTRAS_EVT_STEER, p_fsm->req_id);

    p_fsm->steer = steer;

    /* Cast is safe: the value is masked to 6 bits. */
    p_fsm->steer_seq = (uint8_t)extras_pack_get((uint32_t)p_fsm->steer_seq + 1U, 0U,
                                            EXTRAS_EV_SSEQ_MK);

    word = extras_pack_put(word, EXTRAS_EV_STEER_SH, EXTRAS_EV_STEER_MK, steer);
    word = extras_pack_put(word, EXTRAS_EV_SSEQ_SH, EXTRAS_EV_SSEQ_MK, p_fsm->steer_seq);
    p_out->steer_word    = word;
    p_out->b_steer_valid = true;
}

/*!
 * @brief Enter a state and reset its per-state counters.
 *
 * @param[in,out] p_fsm  Statechart.
 * @param[in]     state  New state (EXTRAS_STATE_*).
 * @param[in]     now_ms Current time.
 */
static void enter (extras_fsm_t * p_fsm, uint8_t state, uint32_t now_ms)
{
    p_fsm->state    = state;
    p_fsm->since_ms = now_ms;
    cnt_set(p_fsm, EXTRAS_CNT_HIT_SH, EXTRAS_CNT_HIT_MK, 0U);
    cnt_set(p_fsm, EXTRAS_CNT_COLL_SH, EXTRAS_CNT_COLL_MK, 0U);
    cnt_set(p_fsm, EXTRAS_CNT_CLEAR_SH, EXTRAS_CNT_CLEAR_MK, 0U);
}

/*!
 * @brief Fault byte bit for a stop cause.
 *
 * @param[in] cause Stop cause (EXTRAS_CAUSE_*).
 *
 * @return The fault bit, or 0 when the cause is not a fault.
 */
static uint8_t fault_bit (uint8_t cause)
{
    uint8_t bit = 0U;

    switch (cause)
    {
        case EXTRAS_CAUSE_NO_ECHO:
            bit = EXTRAS_FAULT_NO_ECHO;
        break;

        case EXTRAS_CAUSE_ECHO_STUCK:
            bit = EXTRAS_FAULT_ECHO_STUCK;
        break;

        case EXTRAS_CAUSE_SCAN_TIMEOUT:
            bit = EXTRAS_FAULT_SCAN_TIMEOUT;
        break;

        case EXTRAS_CAUSE_NO_SAFE_PATH:
            bit = EXTRAS_FAULT_NO_SAFE_PATH;
        break;

        case EXTRAS_CAUSE_COLLISION:
            bit = EXTRAS_FAULT_COLLISION;
        break;

        case EXTRAS_CAUSE_HANDOVER:
            /* Both handover time limits share one fault bit, so this case
             * deliberately falls through to the next one. */
        case EXTRAS_CAUSE_PLAN_TIMEOUT:
            bit = EXTRAS_FAULT_HANDOVER;
        break;

        case EXTRAS_CAUSE_BYPASS_LIMIT:
            bit = EXTRAS_FAULT_BYPASS_LIMIT;
        break;

        case EXTRAS_CAUSE_LINE_TIMEOUT:
            bit = EXTRAS_FAULT_LINE_TIMEOUT;
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
 * @param[in]     cause  Stop cause (EXTRAS_CAUSE_*).
 * @param[in]     now_ms Current time.
 * @param[out]    p_out  Requests for the task.
 */
static void enter_stopped (extras_fsm_t * p_fsm, uint8_t cause, uint32_t now_ms,
                           extras_fsm_out_t * p_out)
{
    uint8_t const bit = fault_bit(cause);
    uint32_t      evt = event_base(EXTRAS_EVT_FAULT, p_fsm->req_id);

    if ((EXTRAS_STATE_BYPASS == p_fsm->state) ||
        (EXTRAS_STATE_REACQUIRE == p_fsm->state))
    {
        emit_steer(p_fsm, p_out, EXTRAS_STEER_HOLD);
    }

    p_fsm->cause = cause;

    /* Cast is safe: both operands are 8-bit fault bytes. */
    p_fsm->faults     = (uint8_t)(p_fsm->faults | bit);
    p_fsm->b_reset_ok = false;
    enter(p_fsm, EXTRAS_STATE_STOPPED, now_ms);

    evt = extras_pack_put(evt, EXTRAS_EV_CAUSE_SH, EXTRAS_EV_CAUSE_MK, cause);
    evt = extras_pack_put(evt, EXTRAS_EV_FAULTS_SH, EXTRAS_EV_FAULTS_MK, p_fsm->faults);
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
static bool is_cause_cleared (extras_fsm_t const * p_fsm)
{
    bool b_cleared = true;

    if ((EXTRAS_CAUSE_NO_ECHO == p_fsm->cause) ||
        (EXTRAS_CAUSE_ECHO_STUCK == p_fsm->cause))
    {
        b_cleared = ((EXTRAS_CLASS_VALID == p_fsm->last_class) ||
                     (EXTRAS_CLASS_OPEN == p_fsm->last_class)) &&
                    (0U == cnt_get(p_fsm, EXTRAS_CNT_UNK_SH, EXTRAS_CNT_UNK_MK)) &&
                    (0U == cnt_get(p_fsm, EXTRAS_CNT_STUCK_SH, EXTRAS_CNT_STUCK_MK));
    }
    else if (EXTRAS_CAUSE_INIT_FAIL == p_fsm->cause)
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
static void accept_plan (extras_fsm_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                         extras_fsm_out_t * p_out)
{
    if (EXTRAS_RESULT_CLEAR == p_fsm->plan.status)
    {
        enter(p_fsm, EXTRAS_STATE_MONITOR, now_ms);
    }
    else
    {
        p_fsm->bypass_mm = odo_mm;
        enter(p_fsm, EXTRAS_STATE_BYPASS, now_ms);
        emit_steer(p_fsm, p_out, extras_plan_steer_away(p_fsm->plan.side));
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
static bool check_health (extras_fsm_t * p_fsm, uint8_t cls, uint32_t now_ms,
                          extras_fsm_out_t * p_out)
{
    bool     b_stopped = false;
    uint32_t unknown   = 0U;
    uint32_t stuck     = 0U;
    uint8_t  cause     = EXTRAS_CAUSE_NONE;

    if (EXTRAS_CLASS_UNKNOWN == cls)
    {
        unknown = cnt_inc(p_fsm, EXTRAS_CNT_UNK_SH, EXTRAS_CNT_UNK_MK);
    }
    else
    {
        cnt_set(p_fsm, EXTRAS_CNT_UNK_SH, EXTRAS_CNT_UNK_MK, 0U);
    }

    if (EXTRAS_CLASS_STUCK == cls)
    {
        stuck = cnt_inc(p_fsm, EXTRAS_CNT_STUCK_SH, EXTRAS_CNT_STUCK_MK);
    }
    else
    {
        cnt_set(p_fsm, EXTRAS_CNT_STUCK_SH, EXTRAS_CNT_STUCK_MK, 0U);
    }

    if (unknown >= EXTRAS_NO_ECHO_COUNT)
    {
        cause = EXTRAS_CAUSE_NO_ECHO;
    }
    else if (stuck >= EXTRAS_STUCK_COUNT)
    {
        cause = EXTRAS_CAUSE_ECHO_STUCK;
    }
    else
    {
        cause = EXTRAS_CAUSE_NONE;
    }

    if ((EXTRAS_STATE_MONITOR == p_fsm->state) ||
        (EXTRAS_STATE_STOPPED == p_fsm->state))
    {
        /* Echo faults are diagnostic until a measured obstacle owns the
         * wheels. Keep reporting them while parked or following tape. A
         * fault after RESET therefore no longer blocks START. */
        uint32_t const echo_faults = (uint32_t)EXTRAS_FAULT_NO_ECHO | (uint32_t)EXTRAS_FAULT_ECHO_STUCK;
        p_fsm->faults = (uint8_t)(((uint32_t)p_fsm->faults & ~echo_faults) | (uint32_t)fault_bit(cause));
    }
    else if (EXTRAS_CAUSE_NONE != cause)
    {
        enter_stopped(p_fsm, cause, now_ms, p_out);
        b_stopped = true;
    }
    else
    {
        /* Continue the measured-obstacle sequence. */
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
static void ping_monitor (extras_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                          uint32_t now_ms, extras_fsm_out_t * p_out)
{
    bool const b_hit = (EXTRAS_CLASS_VALID == cls) && (dist_mm <= EXTRAS_DETECT_MM);

    if (!b_hit)
    {
        cnt_set(p_fsm, EXTRAS_CNT_HIT_SH, EXTRAS_CNT_HIT_MK, 0U);
    }
    else if (cnt_inc(p_fsm, EXTRAS_CNT_HIT_SH, EXTRAS_CNT_HIT_MK) >= EXTRAS_DETECT_HITS)
    {
        uint32_t evt = event_base(EXTRAS_EVT_OBSTACLE, p_fsm->req_id);

        evt = extras_pack_put(evt, EXTRAS_EV_DIST_SH, EXTRAS_EV_DIST_MK, dist_mm);
        emit(p_out, evt);
        enter(p_fsm, EXTRAS_STATE_WAIT_STOP, now_ms);
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
static void ping_scanning (extras_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                           uint32_t now_ms, extras_fsm_out_t * p_out)
{
    p_out->b_profile_changed = true;

    if (extras_scan_feed(&p_fsm->scan, cls, dist_mm))
    {
        uint32_t evt = event_base(EXTRAS_EVT_SCAN_RESULT, p_fsm->req_id);

        extras_plan_build(&p_fsm->scan, &p_fsm->cfg, &p_fsm->plan);
        evt = extras_pack_put(evt, EXTRAS_EV_STATUS_SH, EXTRAS_EV_STATUS_MK,
                          p_fsm->plan.status);
        evt = extras_pack_put(evt, EXTRAS_EV_SIDE_SH, EXTRAS_EV_SIDE_MK, p_fsm->plan.side);
        evt = extras_pack_put(evt, EXTRAS_EV_WIDTH_SH, EXTRAS_EV_WIDTH_MK,
                          p_fsm->plan.width_mm);
        evt = extras_pack_put(evt, EXTRAS_EV_ANGLE_SH, EXTRAS_EV_ANGLE_MK,
                          p_fsm->plan.start_deg);
        emit(p_out, evt);

        if (EXTRAS_RESULT_NO_SAFE_PATH == p_fsm->plan.status)
        {
            enter_stopped(p_fsm, EXTRAS_CAUSE_NO_SAFE_PATH, now_ms, p_out);
        }
        else
        {
            enter(p_fsm, EXTRAS_STATE_PLAN, now_ms);
        }
    }
}

/*!
 * @brief BYPASS: bend away until the forward beam has been clear for
 *        EXTRAS_ESCAPE_CLEAR_PINGS pings in a row, then start the zig-zag
 *        (driving straight until its first ping).
 *
 * @param[in,out] p_fsm   Statechart.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance.
 * @param[in]     now_ms  Current time.
 * @param[out]    p_out   Requests for the task.
 */
static void ping_bypass (extras_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                         uint32_t now_ms, extras_fsm_out_t * p_out)
{
    uint32_t clear_count = 0U;

    if (extras_plan_is_escape_clear(cls, dist_mm, p_fsm->plan.closest_mm))
    {
        clear_count = cnt_inc(p_fsm, EXTRAS_CNT_CLEAR_SH, EXTRAS_CNT_CLEAR_MK);
    }
    else
    {
        cnt_set(p_fsm, EXTRAS_CNT_CLEAR_SH, EXTRAS_CNT_CLEAR_MK, 0U);
    }

    if (clear_count >= EXTRAS_ESCAPE_CLEAR_PINGS)
    {
        /* The way ahead is clear: Navigation takes over (it keeps pivoting
         * until the car is 90 degrees off the line, then moves out). */
        enter(p_fsm, EXTRAS_STATE_REACQUIRE, now_ms);
        emit_steer(p_fsm, p_out, EXTRAS_STEER_STRAIGHT);
    }
    else
    {
        emit_steer(p_fsm, p_out, extras_plan_steer_away(p_fsm->plan.side));
    }
}

/*!
 * @brief Switch the collision guard off (or back on) while Navigation's
 *        side scan runs (extras_comms_sweep()).
 *
 * @param[in,out] p_fsm Statechart.
 * @param[in]     b_off True while the car stands still for the sweep.
 */
void extras_fsm_set_guard_off (extras_fsm_t * p_fsm, bool b_off)
{
    p_fsm->b_guard_off = b_off;
}

/*** end of file ***/
