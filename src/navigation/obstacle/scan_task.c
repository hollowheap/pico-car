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

/* Flag bits that mean "a command is waiting". */
#define CMD_BITS (OBSTACLE_FLG_CMD | OBSTACLE_FLG_STOP)

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
static void     wait_ms (uint32_t delay_ms);
static void     tick (void);
static uint32_t settle_ms (int16_t target);
static bool     is_car_driving (void);
static bool     move_servo (void);
static void     ping (uint8_t * p_cls, uint16_t * p_dist_mm);
static void     ping_and_step (void);

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
        tick();
        since_us = scan_hw_now_us() - g_last_trig_us;

        if (move_servo())
        {
            /* The servo moved and settled; loop to re-check commands. */
        }
        else if (since_us < PING_PERIOD_US)
        {
            /* Round up so the next pass is never early. */
            wait_ms(((PING_PERIOD_US - since_us) + (USEC_PER_MSEC - 1U)) /
                    USEC_PER_MSEC);
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
    uint8_t        cmd    = 0U;
    uint32_t const now    = now_ms();
    uint32_t const odo_mm = obstacle_comms_odometer_mm(now);

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
 * @param[in] delay_ms How long to sleep.
 */
static void wait_ms (uint32_t delay_ms)
{
    uint32_t const start_ms = now_ms();
    uint32_t       spent_ms = 0U;

    while (spent_ms < delay_ms)
    {
        UINT ptn = 0U;

        /* Cast is safe: the remaining time is below 1000 ms. */
        ER const result =
            tk_wai_flg(obstacle_comms_flag(), CMD_BITS, TWF_ORW | TWF_BITCLR, &ptn,
                       (TMO)(delay_ms - spent_ms));

        if (E_OK == result)
        {
            handle_commands(ptn);
        }

        spent_ms = now_ms() - start_ms;
    }
}

/*!
 * @brief Let the statechart check its time and distance limits.
 */
static void tick (void)
{
    obstacle_statechart_out_t   out;
    uint32_t const now = now_ms();

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&g_fsm, now, obstacle_comms_odometer_mm(now), &out);
    obstacle_comms_publish(&out, &g_fsm);
}

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

/*!
 * @brief Tell whether the car may be driving in the current state (line
 *        following in MONITOR, or the bypass).
 *
 * @return True in MONITOR, BYPASS and REACQUIRE.
 */
static bool is_car_driving (void)
{
    return ((OBSTACLE_STATE_MONITOR == g_fsm.state) ||
            (OBSTACLE_STATE_BYPASS == g_fsm.state) ||
            (OBSTACLE_STATE_REACQUIRE == g_fsm.state));
}

/*!
 * @brief Move the servo if the statechart wants a new angle.
 *
 * @return True if the servo moved (and has now settled).
 */
static bool move_servo (void)
{
    bool    b_moved = false;
    int16_t target  = obstacle_statechart_servo_deci(&g_fsm);

    if (g_servo.b_known && (target != g_servo.deci) && (is_car_driving()))
    {
        /* Let Navigation stop the wheels before the servo swings: a servo
         * swing while a motor drives disturbs the wheel encoders. The
         * target may change meanwhile, so read it again. */
        wait_ms(OBSTACLE_SERVO_PREDELAY_MS);
        target = obstacle_statechart_servo_deci(&g_fsm);
    }

    if ((!g_servo.b_known) || (target != g_servo.deci))
    {
        uint32_t const delay_ms = settle_ms(target);

        scan_hw_servo_set(target);
        g_servo.deci    = target;
        g_servo.b_known = true;
        wait_ms(delay_ms);
        b_moved = true;
    }

    return (b_moved);
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

    /* Drop a stale ECHO bit left over from a late edge. */
    (void)tk_clr_flg(obstacle_comms_flag(), ~(UINT)OBSTACLE_FLG_ECHO);
    b_stuck        = scan_hw_rearm();
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
            *p_cls = scan_algo_classify(echo_result, (E_OK != result),
                                        scan_hw_is_echo_high(), false);

            if (OBSTACLE_CLASS_VALID == *p_cls)
            {
                *p_dist_mm = scan_algo_dist_mm(echo_result & OBSTACLE_ECHO_WIDTH_MASK);
            }
        }
    }
}

/*!
 * @brief Take one ping and feed it to the statechart.
 */
static void ping_and_step (void)
{
    obstacle_statechart_out_t out;
    uint8_t      cls     = OBSTACLE_CLASS_NO_ECHO;
    uint16_t     dist_mm = 0U;

    obstacle_statechart_set_odometer(&g_fsm, obstacle_comms_odometer_mm(now_ms()));
    ping(&cls, &dist_mm);
    obstacle_comms_count_class(cls);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&g_fsm, cls, dist_mm, now_ms(), &out);
    obstacle_comms_publish(&out, &g_fsm);
}

/*** end of file ***/

