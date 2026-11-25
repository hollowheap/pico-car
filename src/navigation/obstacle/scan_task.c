/** @file scan_task.c
 *
 * @brief Buddy 5 task (processor 1, priority 6). Each pass of the loop
 *        does exactly one of: move the servo and let it settle, wait for
 *        the 60 ms ping spacing, or take one ping and feed it to the
 *        statechart. Commands and time limits are handled on every pass.
 *
 * NOTE: The kernel API types INT, UINT, ER, TMO and SYSTIM are used where
 * the kernel functions require them (deviation from BARR-C 5.2.a, see
 * docs/obstacle.md).
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "obstacle_config.h"
#include "comms.h"
#include "statechart.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "obstacle_internal.h"
#include "platform/liveness.h"
#include "supervisor/health_ids.h"
#include "scan_algo.h"
#include "scan_hw.h"
#include "scan_task.h"

/* Microseconds per millisecond. */
#define USEC_PER_MSEC (1000U)

/* Deci-degrees per degree. */
#define DECI_PER_DEG (10U)

/* The servo's start position is unknown at boot, so assume the worst
 * case: a full 60 degree sweep. */
#define FULL_SWEEP_DECI (600U)

/* The ping spacing in microseconds. */
#define PING_PERIOD_US (OBSTACLE_PING_PERIOD_MS * USEC_PER_MSEC)

/* Flag bits that wake the task: a command, STOP, or a ping request. */
#define CMD_BITS (OBSTACLE_FLG_CMD | OBSTACLE_FLG_STOP | OBSTACLE_FLG_PING)

/* Last servo command. NOTE: The Boolean is a b_ member of a g_ struct,
 * which satisfies both BARR-C 7.1.m/7.1.o and the team .clang-tidy. */
typedef struct
{
    int16_t deci;
    bool    b_known;
} servo_state_t;

/* Task-private state. Only this task touches it, so it is not volatile. */
static obstacle_statechart_t      g_fsm;
static servo_state_t g_servo        = {0, false};
static uint32_t      g_last_trig_us = 0U;

static uint32_t now_ms (void);
static void     handle_commands (UINT ptn);
static void     poll_commands (void);
static void     wait_ms (uint32_t delay_ms, UINT stop_bits);
#if !PICO_CAR_PILOT
static void     tick (void);
#endif
static uint32_t settle_ms (int16_t target);
#if !PICO_CAR_PILOT
static bool     is_car_driving (void);
#endif
static bool     move_servo (void);
static void     ping (uint8_t * p_cls, uint16_t * p_dist_mm);
static void     ping_and_step (void);
static bool     serve_ping_request (void);

/*!
 * @brief Buddy 5 task body. Never returns.
 *
 * @param[in] stacd Start code (unused).
 * @param[in] exinf Extended information (unused).
 */
void scan_task_obstacle_task (INT stacd, void * exinf)
{
    obstacle_plan_cfg_t cfg;
    obstacle_statechart_out_t  out;

    (void)stacd;
    (void)exinf;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&g_fsm, &cfg, now_ms());
    obstacle_statechart_out_clear(&out);
    out.b_profile_changed = true;
    obstacle_comms_publish(&out, &g_fsm);

    /* Allow the first ping straight away. */
    g_last_trig_us = scan_hw_now_us() - PING_PERIOD_US;

    for (;;)
    {
        uint32_t since_us = 0U;

        liveness_kick(LIVE_OBSTACLE);
        poll_commands();
#if !PICO_CAR_PILOT
        /* With the pilot the statechart has no time or distance limits to
         * check: the pilot owns the wheels. */
        tick();
#endif
        since_us = scan_hw_now_us() - g_last_trig_us;

        if (serve_ping_request())
        {
            /* One requested ping was taken; loop to re-check commands. */
        }
        else if (move_servo())
        {
            /* The servo moved and settled; loop to re-check commands. */
        }
        else if (since_us < PING_PERIOD_US)
        {
            /* Round up so the next pass is never early. A ping request ends
             * the wait, so its servo settles while the spacing runs out. */
            wait_ms(((PING_PERIOD_US - since_us) + (USEC_PER_MSEC - 1U)) /
                        USEC_PER_MSEC,
                    OBSTACLE_FLG_PING);
        }
        else
        {
            ping_and_step();
        }
    }
}

/*!
 * @brief Kernel time in milliseconds.
 *
 * @return Low 32 bits of the system time.
 */
static uint32_t now_ms (void)
{
    SYSTIM tim = {0, 0U};

    (void)tk_get_otm(&tim);

    return (tim.lo);
}

/*!
 * @brief Feed STOP (if flagged) and every queued command to the
 *        statechart, publishing after each one.
 *
 * @param[in] ptn Flag bits that woke the task, or 0.
 */
static void handle_commands (UINT ptn)
{
    obstacle_statechart_out_t   out;
    obstacle_travel_t travel;
    uint8_t           cmd    = 0U;
    uint32_t const    now    = now_ms();
    uint32_t          odo_mm = 0U;

    (void)obstacle_comms_travel(&travel);
    /* The bypass budget counts all travel, forward and backward. */
    odo_mm = travel.fwd_mm + travel.back_mm;

    if (0U != (ptn & OBSTACLE_FLG_STOP))
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_command(&g_fsm, obstacle_pack_cmd(OBSTACLE_CMD_STOP, 0U), now, odo_mm, &out);
        obstacle_comms_publish(&out, &g_fsm);
    }

    while (obstacle_comms_pop_cmd(&cmd))
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_command(&g_fsm, cmd, now, odo_mm, &out);
        obstacle_comms_publish(&out, &g_fsm);
    }
}

/*!
 * @brief Check the command bits without waiting.
 */
static void poll_commands (void)
{
    UINT     ptn    = 0U;
    ER const result = tk_wai_flg(obstacle_comms_flag(), CMD_BITS,
                                 TWF_ORW | TWF_BITCLR, &ptn, TMO_POL);

    if (E_OK != result)
    {
        /* NOTE: The queue may still hold commands whose flag bit was
         * already cleared by an earlier wake-up. */
        ptn = 0U;
    }

    handle_commands(ptn);
}

/*!
 * @brief Sleep, but wake up for commands.
 *
 * @param[in] delay_ms  How long to sleep.
 * @param[in] stop_bits Flag bits that end the sleep early. A settle wait
 *                      passes 0: a ping request must not cut it short.
 */
static void wait_ms (uint32_t delay_ms, UINT stop_bits)
{
    uint32_t const start_ms = now_ms();
    uint32_t       spent_ms = 0U;
    bool           b_stop   = false;

    while ((spent_ms < delay_ms) && (!b_stop))
    {
        UINT ptn = 0U;

        /* Cast is safe: the remaining time is below 1000 ms. */
        ER const result =
            tk_wai_flg(obstacle_comms_flag(), CMD_BITS, TWF_ORW | TWF_BITCLR, &ptn,
                       (TMO)(delay_ms - spent_ms));

        if (E_OK == result)
        {
            handle_commands(ptn);
            b_stop = (0U != (ptn & stop_bits));
        }

        spent_ms = now_ms() - start_ms;
    }
}

#if !PICO_CAR_PILOT
/*!
 * @brief Let the statechart check its time and distance limits.
 */
static void tick (void)
{
    obstacle_statechart_out_t out;
    obstacle_travel_t         travel;
    uint32_t const            now = now_ms();

    (void)obstacle_comms_travel(&travel);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&g_fsm, now, travel.fwd_mm + travel.back_mm, &out);
    obstacle_comms_publish(&out, &g_fsm);
}
#endif

/*!
 * @brief Settle time for a servo move: 3 ms per degree plus 20 ms [Spec],
 *        sized for LiPo sag.
 *
 * @param[in] target New angle in deci-degrees.
 *
 * @return Settle time in ms.
 */
static uint32_t settle_ms (int16_t target)
{
    uint32_t delta_deci = FULL_SWEEP_DECI;

    if (g_servo.b_known)
    {
        int32_t delta = target - g_servo.deci;

        if (delta < 0)
        {
            delta = -delta;
        }

        /* Cast is safe: delta is 0..1200 after taking the magnitude. */
        delta_deci = (uint32_t)delta;
    }

    return (((delta_deci * OBSTACLE_SETTLE_MS_PER_DEG) / DECI_PER_DEG) +
            OBSTACLE_SETTLE_BASE_MS);
}

#if !PICO_CAR_PILOT
/*!
 * @brief Tell whether the car may be driving in the current state (line
 *        following in MONITOR, the body turns of a scan, or the bypass).
 *
 * @return True in MONITOR, SCANNING, PLAN, BYPASS and REACQUIRE.
 */
static bool is_car_driving (void)
{
    return ((OBSTACLE_STATE_MONITOR == g_fsm.state) ||
            (OBSTACLE_STATE_SCANNING == g_fsm.state) ||
            (OBSTACLE_STATE_PLAN == g_fsm.state) ||
            (OBSTACLE_STATE_BYPASS == g_fsm.state) ||
            (OBSTACLE_STATE_REACQUIRE == g_fsm.state));
}

/*!
 * @brief The servo angle for the next ping: the face follow's look in
 *        REACQUIRE, the statechart's angle in every other state.
 *
 * @return Angle in deci-degrees, inside +-OBSTACLE_SERVO_LIMIT_DECI.
 */
static int16_t servo_target (void)
{
    int16_t target = obstacle_statechart_servo_deci(&g_fsm);

    if (OBSTACLE_STATE_REACQUIRE == g_fsm.state)
    {
        target = obstacle_comms_look();
        if (target > (int16_t)OBSTACLE_SERVO_LIMIT_DECI)
        {
            target = (int16_t)OBSTACLE_SERVO_LIMIT_DECI;
        }
        else if (target < -(int16_t)OBSTACLE_SERVO_LIMIT_DECI)
        {
            target = -(int16_t)OBSTACLE_SERVO_LIMIT_DECI;
        }
        else
        {
            /* Within the limit. */
        }
    }
    return (target);
}
#endif

/*!
 * @brief Move the servo if the target angle changed.
 *
 * @return True if the servo moved (and has now settled).
 */
static bool move_servo (void)
{
    bool    b_moved = false;
#if PICO_CAR_PILOT
    /* A pilot look leaves the servo at its last ray. Free-running pings point
     * ahead, so return to 0 once the next such ping is due. Not before: the
     * next ray of one look arrives within the ping spacing. */
    int16_t const target = 0;

    if ((!g_servo.b_known) ||
        ((target != g_servo.deci) && ((scan_hw_now_us() - g_last_trig_us) >= PING_PERIOD_US)))
    {
        scan_hw_servo_set(target);
        g_servo.deci    = target;
        g_servo.b_known = true;
        wait_ms(settle_ms(target), 0U);
        b_moved = true;
    }

    return (b_moved);
#else
    int16_t target  = servo_target();

    if (g_servo.b_known && (target != g_servo.deci) && (is_car_driving()))
    {
        /* Let Navigation stop the wheels before the servo swings: a servo
         * swing while a motor drives disturbs the wheel encoders. The
         * target may change meanwhile, so read it again. */
        wait_ms(OBSTACLE_SERVO_PREDELAY_MS, 0U);
        target = servo_target();
    }

    if ((!g_servo.b_known) || (target != g_servo.deci))
    {
        uint32_t const delay_ms = settle_ms(target);

        scan_hw_servo_set(target);
        g_servo.deci    = target;
        g_servo.b_known = true;
        wait_ms(delay_ms, 0U);
        b_moved = true;
    }

    return (b_moved);
#endif
}

/*!
 * @brief One ping: re-arm, trigger, wait up to 40 ms, classify.
 *
 * @param[out] p_cls     Echo class (OBSTACLE_CLASS_*).
 * @param[out] p_dist_mm Distance; 0 unless the class is VALID.
 */
static void ping (uint8_t * p_cls, uint16_t * p_dist_mm)
{
    bool b_stuck = false;

    /* Empty the result first, then drop a stale ECHO bit: a late edge in
     * between then leaves no set bit behind an empty result. */
    b_stuck        = scan_hw_rearm();
    (void)tk_clr_flg(obstacle_comms_flag(), ~(UINT)OBSTACLE_FLG_ECHO);
    g_last_trig_us = scan_hw_now_us();
    *p_dist_mm     = 0U;

    if (b_stuck)
    {
        *p_cls = OBSTACLE_CLASS_STUCK;
    }
    else
    {
        UINT     ptn    = 0U;
        ER       result = E_OK;
        uint32_t echo_result    = 0U;

        bool const b_pulse = scan_hw_trigger();

        if (!b_pulse)
        {
            /* P4: the TRIG pad did not read high during the pulse, so the sensor
             * got no trigger. Skip the wait; this ping proves nothing. */
            *p_cls = OBSTACLE_CLASS_NO_ECHO;
        }
        else
        {
            /* Cast is safe: OBSTACLE_ECHO_WAIT_MS is 40. */
            result = tk_wai_flg(obstacle_comms_flag(), OBSTACLE_FLG_ECHO, TWF_ORW | TWF_BITCLR,
                                &ptn, (TMO)OBSTACLE_ECHO_WAIT_MS);
            echo_result    = scan_hw_echo_result();
            if ((E_OK == result) && (0U == (echo_result & OBSTACLE_ECHO_DONE)))
            {
                /* Woken without a finished echo: this ping proves nothing. */
                *p_cls = OBSTACLE_CLASS_NO_ECHO;
            }
            else
            {
                *p_cls = scan_algo_classify(echo_result, (E_OK != result),
                                            scan_hw_is_echo_high(), false);
            }

            if (OBSTACLE_CLASS_VALID == *p_cls)
            {
                *p_dist_mm = scan_algo_dist_mm(echo_result & OBSTACLE_ECHO_WIDTH_MASK);
            }
        }
    }
}

/*!
 * @brief Serve a pending ping request: move the servo to the clamped angle,
 *        let it settle, keep the ping spacing, ping and publish the reply.
 *
 * The ping also goes to the statechart's health and collision checks, so a
 * stuck echo or a dead sonar sets the same fault bit as a free-running ping.
 *
 * @return True if a request was served.
 */
static bool serve_ping_request (void)
{
    uint32_t const req = obstacle_comms_take_ping_request();
    bool           b_served = false;

    if (obstacle_pack_ping_req_is_pending(req))
    {
        obstacle_statechart_out_t out;
        uint8_t                   cls     = OBSTACLE_CLASS_NO_ECHO;
        uint16_t                  dist_mm = 0U;
        /* Cast is safe: the clamp keeps the angle inside +-300. */
        int16_t const target =
            (int16_t)obstacle_pack_servo_clamp(obstacle_pack_ping_req_servo(req));
        uint32_t since_us = 0U;

        if ((!g_servo.b_known) || (target != g_servo.deci))
        {
            uint32_t const delay_ms = settle_ms(target);

            scan_hw_servo_set(target);
            g_servo.deci    = target;
            g_servo.b_known = true;
            wait_ms(delay_ms, 0U);
        }

        since_us = scan_hw_now_us() - g_last_trig_us;
        if (since_us < PING_PERIOD_US)
        {
            wait_ms(((PING_PERIOD_US - since_us) + (USEC_PER_MSEC - 1U)) / USEC_PER_MSEC, 0U);
        }

        ping(&cls, &dist_mm);
        obstacle_comms_count_class(cls);
        obstacle_statechart_out_clear(&out);
#if PICO_CAR_PILOT
        obstacle_statechart_ping_health(&g_fsm, cls, dist_mm, false, now_ms(), &out);
#else
        obstacle_statechart_ping_guard(&g_fsm, cls, dist_mm, now_ms(), &out);
#endif
        obstacle_comms_publish(&out, &g_fsm);
        obstacle_comms_publish_ping_reply(
            obstacle_pack_ping_reply(obstacle_pack_ping_req_seq(req), cls, dist_mm, target));
        b_served = true;
    }

    return (b_served);
}

/*!
 * @brief Take one ping and feed it to the statechart.
 */
static void ping_and_step (void)
{
    obstacle_statechart_out_t out;
    uint8_t                   cls     = OBSTACLE_CLASS_NO_ECHO;
    uint16_t                  dist_mm = 0U;
#if !PICO_CAR_PILOT
    obstacle_travel_t         travel;
    /* The body may start or stop turning during the 40 ms echo wait. */
    bool const                b_turned = obstacle_comms_is_pivoting();
    bool const                b_fresh  = obstacle_comms_travel(&travel);

    /* The range check compares the net forward travel. */
    obstacle_statechart_set_odometer(&g_fsm, travel.fwd_mm - travel.back_mm, b_fresh);
#endif
    obstacle_statechart_set_look(&g_fsm, g_servo.deci);
    ping(&cls, &dist_mm);
    obstacle_comms_count_class(cls);
    obstacle_statechart_out_clear(&out);
#if PICO_CAR_PILOT
    /* Free-running pings feed only the echo health checks. */
    obstacle_statechart_ping_health(&g_fsm, cls, dist_mm, true, now_ms(), &out);
#else
    if (b_turned || obstacle_comms_is_pivoting())
    {
        obstacle_statechart_ping_guard(&g_fsm, cls, dist_mm, now_ms(), &out);
    }
    else
    {
        obstacle_statechart_ping(&g_fsm, cls, dist_mm, now_ms(), &out);
    }
#endif
    obstacle_comms_publish(&out, &g_fsm);
}

/*** end of file ***/

