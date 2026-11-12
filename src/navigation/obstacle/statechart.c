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

static uint8_t  run_count (uint8_t count, bool b_match);
static uint32_t window_push (uint8_t * p_hist, bool b_hit);
static uint16_t ranged_dist (uint8_t cls, uint16_t dist_mm);
static void     check_plausibility (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm);
static uint32_t event_base (uint32_t type, uint32_t req_id);
static void     emit (obstacle_statechart_out_t * p_out, uint32_t evt);
static void     emit_steer (obstacle_statechart_t * p_fsm,
                            obstacle_statechart_out_t * p_out, uint8_t steer);
static void     enter (obstacle_statechart_t * p_fsm, uint8_t state, uint32_t now_ms);
static uint16_t fault_bit (uint8_t cause);
static void     enter_stopped (obstacle_statechart_t * p_fsm, uint8_t cause, uint32_t now_ms,
                               obstacle_statechart_out_t * p_out);
static bool     is_cause_cleared (obstacle_statechart_t const * p_fsm);
static void     accept_plan (obstacle_statechart_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                             obstacle_statechart_out_t * p_out);
static bool     check_health (obstacle_statechart_t * p_fsm, uint8_t cls, uint32_t now_ms,
                              obstacle_statechart_out_t * p_out);
static bool     check_collision (obstacle_statechart_t * p_fsm, uint16_t dist_mm,
                                 uint32_t now_ms, obstacle_statechart_out_t * p_out);
static void     ping_monitor (obstacle_statechart_t * p_fsm, uint16_t dist_mm,
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
    p_fsm->last_class       = OBSTACLE_CLASS_NO_ECHO;
    p_fsm->last_dist        = 0U;
    p_fsm->seq              = 0U;
    p_fsm->steer            = OBSTACLE_STEER_HOLD;
    p_fsm->steer_seq        = 0U;
    p_fsm->counts           = (obstacle_counters_t){0};
    p_fsm->b_reset_ok       = false;
    p_fsm->since_ms         = now_ms;
    p_fsm->bypass_mm        = 0U;
    p_fsm->scan_progress_ms = now_ms;
    p_fsm->hug_limit_mm     = OBSTACLE_HUG_MIN_MM;
    p_fsm->odo_mm           = 0U;
    p_fsm->anchor_odo_mm    = 0U;
    p_fsm->anchor_dist      = 0U;
    p_fsm->b_anchor         = false;
    p_fsm->b_inconsistent   = false;
    p_fsm->boot_pings       = 0U;
    p_fsm->boot_hits        = 0U;
    p_fsm->b_beam_blocked   = false;
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
    uint16_t const dist = ranged_dist(cls, dist_mm);

    p_fsm->seq        = (uint8_t)(((uint32_t)p_fsm->seq + 1U) & OBSTACLE_ST_SEQ_MK);
    p_fsm->last_class = cls;
    p_fsm->last_dist  = dist;
    check_plausibility(p_fsm, cls, dist);

    if (check_health(p_fsm, cls, now_ms, p_out))
    {
        /* Fault latched. */
    }
    else if (check_collision(p_fsm, dist, now_ms, p_out))
    {
        /* Collision latched. */
    }
    else
    {
        switch (p_fsm->state)
        {
            case OBSTACLE_STATE_MONITOR:
                ping_monitor(p_fsm, dist, now_ms, p_out);
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

/*!
 * @brief Warning flags for the snapshot: wall plan, noisy echo, noisy scan.
 *
 * @param[in] p_fsm Statechart.
 *
 * @return OBSTACLE_FLAG_* bits.
 */
uint8_t obstacle_statechart_flags (obstacle_statechart_t const * p_fsm)
{
    uint8_t flags = 0U;
    bool const b_planned = (OBSTACLE_STATE_PLAN == p_fsm->state) ||
                           (OBSTACLE_STATE_BYPASS == p_fsm->state) ||
                           (OBSTACLE_STATE_REACQUIRE == p_fsm->state);

    if (b_planned && p_fsm->plan.b_wall)
    {
        flags |= (uint8_t)OBSTACLE_FLAG_WALL;
    }
    if (scan_algo_echo_noisy(&p_fsm->scan))
    {
        flags |= (uint8_t)OBSTACLE_FLAG_ECHO_NOISY;
    }
    if (p_fsm->b_inconsistent)
    {
        flags |= (uint8_t)OBSTACLE_FLAG_ECHO_INCONSISTENT;
    }
    if (p_fsm->b_beam_blocked)
    {
        flags |= (uint8_t)OBSTACLE_FLAG_BEAM_BLOCKED;
    }
    if (p_fsm->scan.b_scan_noisy)
    {
        flags |= (uint8_t)OBSTACLE_FLAG_SCAN_NOISY;
    }

    return (flags);
}

void obstacle_statechart_set_odometer (obstacle_statechart_t * p_fsm, uint32_t odo_mm)
{
    p_fsm->odo_mm = odo_mm;
}

/* Plausibility checks on a ping. They raise flags and never stop the car.
 * P3: of the first pings after boot, mostly hits means the beam is blocked.
 * P2: the range change between two VALID readings must follow the wheel travel. */
static void check_plausibility (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm)
{
    uint16_t const dist = ranged_dist(cls, dist_mm);

    if (p_fsm->boot_pings < OBSTACLE_BEAM_CHECK_PINGS)
    {
        p_fsm->boot_pings++;
        if ((0U != dist) && (dist <= OBSTACLE_DETECT_MM))
        {
            p_fsm->boot_hits++;
        }
        if (OBSTACLE_BEAM_CHECK_PINGS == p_fsm->boot_pings)
        {
            p_fsm->b_beam_blocked = (p_fsm->boot_hits >= OBSTACLE_BEAM_CHECK_HITS);
        }
    }

    if ((OBSTACLE_CLASS_VALID == cls) && (dist_mm < OBSTACLE_CONSIST_RANGE_MM))
    {
        int32_t const travel = (int32_t)(p_fsm->odo_mm - p_fsm->anchor_odo_mm);

        if (!p_fsm->b_anchor || (travel < 0))
        {
            p_fsm->b_anchor = true;
        }
        else if (travel >= (int32_t)OBSTACLE_CONSIST_TRAVEL_MM)
        {
            int32_t const expected = (int32_t)p_fsm->anchor_dist - travel;
            int32_t const diff     = (int32_t)dist_mm - expected;

            p_fsm->b_inconsistent = (diff > (int32_t)OBSTACLE_CONSIST_TOL_MM) ||
                                    (-diff > (int32_t)OBSTACLE_CONSIST_TOL_MM);
        }
        else
        {
            /* Not far enough yet: keep the anchor. */
            return;
        }
        p_fsm->anchor_dist   = dist_mm;
        p_fsm->anchor_odo_mm = p_fsm->odo_mm;
    }
}

/* Length of the current run of matching pings, saturated at 255. */
static uint8_t run_count (uint8_t count, bool b_match)
{
    uint8_t result = 0U;

    if (b_match)
    {
        result = (count < UINT8_MAX) ? (uint8_t)(count + 1U) : count;
    }

    return (result);
}

/* Shift one ping into a hit history of OBSTACLE_HIT_WINDOW bits (newest in
 * bit 0) and return how many of the pings in the window are hits. */
static uint32_t window_push (uint8_t * p_hist, bool b_hit)
{
    uint32_t bits = (((uint32_t)*p_hist << 1U) | (b_hit ? 1U : 0U)) &
                    ((1UL << OBSTACLE_HIT_WINDOW) - 1UL);
    uint32_t hits = 0U;

    *p_hist = (uint8_t)bits;
    while (0U != bits)
    {
        hits += bits & 1U;
        bits >>= 1U;
    }

    return (hits);
}

/* The range of a ping that has one: the reading for VALID, and the minimum
 * range for TOO_CLOSE. Any other class has none (0). */
static uint16_t ranged_dist (uint8_t cls, uint16_t dist_mm)
{
    uint16_t dist = 0U;

    if (OBSTACLE_CLASS_VALID == cls)
    {
        dist = dist_mm;
    }
    else if (OBSTACLE_CLASS_TOO_CLOSE == cls)
    {
        dist = (uint16_t)OBSTACLE_MIN_RANGE_MM;
    }
    else
    {
        dist = 0U;
    }

    return (dist);
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
    p_fsm->counts   = (obstacle_counters_t){0};
    if (OBSTACLE_STATE_SCANNING == state)
    {
        p_fsm->scan_progress_ms = now_ms;
    }
}

static uint16_t fault_bit (uint8_t cause)
{
    uint16_t bit = 0U;

    switch (cause)
    {
        case OBSTACLE_CAUSE_NO_ECHO:
            bit = (uint16_t)OBSTACLE_FAULT_NO_ECHO;
        break;

        case OBSTACLE_CAUSE_ECHO_STUCK:
            bit = (uint16_t)OBSTACLE_FAULT_ECHO_STUCK;
        break;

        case OBSTACLE_CAUSE_SCAN_TIMEOUT:
            bit = (uint16_t)OBSTACLE_FAULT_SCAN_TIMEOUT;
        break;

        case OBSTACLE_CAUSE_NO_SAFE_PATH:
            bit = (uint16_t)OBSTACLE_FAULT_NO_SAFE_PATH;
        break;

        case OBSTACLE_CAUSE_COLLISION:
            bit = (uint16_t)OBSTACLE_FAULT_COLLISION;
        break;

        case OBSTACLE_CAUSE_ECHO_NOISE:
            bit = (uint16_t)OBSTACLE_FAULT_ECHO_NOISE;
        break;

        case OBSTACLE_CAUSE_HANDOVER:
            bit = (uint16_t)OBSTACLE_FAULT_HANDOVER;
        break;

        case OBSTACLE_CAUSE_BYPASS_LIMIT:
            bit = (uint16_t)OBSTACLE_FAULT_BYPASS_LIMIT;
        break;

        case OBSTACLE_CAUSE_LINE_TIMEOUT:
            bit = (uint16_t)OBSTACLE_FAULT_LINE_TIMEOUT;
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
    uint16_t const bit  = fault_bit(cause);
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
    obstacle_counters_t * const p_cnt = &p_fsm->counts;
    uint8_t                     cause = OBSTACLE_CAUSE_NONE;

    p_cnt->no_echo = run_count(p_cnt->no_echo, OBSTACLE_CLASS_NO_ECHO == cls);
    p_cnt->stuck   = run_count(p_cnt->stuck, OBSTACLE_CLASS_STUCK == cls);
    p_cnt->glitch  = run_count(p_cnt->glitch, OBSTACLE_CLASS_GLITCH == cls);
    p_cnt->fail    = run_count(p_cnt->fail, obstacle_pack_class_is_unresolved(cls));

    if (p_cnt->no_echo >= OBSTACLE_NO_ECHO_COUNT)
    {
        cause = OBSTACLE_CAUSE_NO_ECHO;
    }
    else if (p_cnt->stuck >= OBSTACLE_STUCK_COUNT)
    {
        cause = OBSTACLE_CAUSE_ECHO_STUCK;
    }
    else if ((p_cnt->glitch >= OBSTACLE_GLITCH_COUNT) || (p_cnt->fail >= OBSTACLE_FAIL_COUNT))
    {
        cause = OBSTACLE_CAUSE_ECHO_NOISE;
    }
    else
    {
        /* The echo line is healthy enough. */
    }

    if (OBSTACLE_CAUSE_NONE != cause)
    {
        enter_stopped(p_fsm, cause, now_ms, p_out);
    }

    return (OBSTACLE_CAUSE_NONE != cause);
}

static bool check_collision (obstacle_statechart_t * p_fsm, uint16_t dist_mm,
                             uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    bool const     b_close = (0U != dist_mm) && (dist_mm <= OBSTACLE_COLLISION_MM);
    uint32_t const hits    = window_push(&p_fsm->counts.coll_hist, b_close);
    bool           b_fault = false;

    if (hits >= OBSTACLE_COLLISION_HITS)
    {
        enter_stopped(p_fsm, OBSTACLE_CAUSE_COLLISION, now_ms, p_out);
        b_fault = true;
    }

    return (b_fault);
}

static void ping_monitor (obstacle_statechart_t * p_fsm, uint16_t dist_mm,
                          uint32_t now_ms, obstacle_statechart_out_t * p_out)
{
    bool const     b_hit = (0U != dist_mm) && (dist_mm <= OBSTACLE_DETECT_MM);
    uint32_t const hits  = window_push(&p_fsm->counts.hit_hist, b_hit);

    if (hits >= OBSTACLE_DETECT_HITS)
    {
        uint32_t word = 0U;

        enter(p_fsm, OBSTACLE_STATE_WAIT_STOP, now_ms);
        word = event_base(OBSTACLE_EVT_OBSTACLE, p_fsm->req_id);
        word = obstacle_pack_put(word, OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK, dist_mm);
        emit(p_out, word);
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

        /* The finished profile and plan are what the profile telemetry shows. */
        p_out->b_profile_changed = true;
    }
}

static void ping_bypass (obstacle_statechart_t * p_fsm, uint8_t cls,
                         uint16_t dist_mm, uint32_t now_ms,
                         obstacle_statechart_out_t * p_out)
{
    if (obstacle_plan_is_escape_clear(cls, dist_mm, p_fsm->plan.closest_mm))
    {
        p_fsm->counts.clear = run_count(p_fsm->counts.clear, true);

        if ((p_fsm->counts.clear >= OBSTACLE_ESCAPE_CLEARS) && (p_fsm->b_turned))
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
        p_fsm->counts.clear = 0U;
        emit_steer(p_fsm, p_out, obstacle_plan_steer_away(p_fsm->plan.side));
    }
}

static void ping_reacquire (obstacle_statechart_t * p_fsm, uint8_t cls,
                            uint16_t dist_mm, uint32_t now_ms,
                            obstacle_statechart_out_t * p_out)
{
    /* S40, D96: While moving, the sonar faces forward (0 deg). Any VALID or
     * TOO_CLOSE reading under OBSTACLE_FWD_GUARD_MM (300 mm) stops the car and
     * rescans. */
    uint16_t const dist = ranged_dist(cls, dist_mm);

    if ((0U != dist) && (dist < OBSTACLE_FWD_GUARD_MM))
    {
        uint32_t word = 0U;

        enter(p_fsm, OBSTACLE_STATE_WAIT_STOP, now_ms);
        word = event_base(OBSTACLE_EVT_OBSTACLE, p_fsm->req_id);
        word = obstacle_pack_put(word, OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK, dist);
        emit(p_out, word);
        emit_steer(p_fsm, p_out, OBSTACLE_STEER_HOLD);
    }
}

/*** end of file ***/
