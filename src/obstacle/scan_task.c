/** @file scan_task.c
 *
 * @brief Buddy 5 task (processor 1, priority 6). Each pass of the loop
 *        does exactly one of: move the servo and let it settle, wait for
 *        the 60 ms ping spacing, or take one ping and feed it to the
 *        statechart. Commands and time limits are handled on every pass.
 *        While Navigation's rotation scan has tracking on, the servo is
 *        re-aimed without settling and the wait is cut to
 *        B5_TRACK_PERIOD_MS, so it follows the car smoothly between pings.
 *        While Navigation's side scan (sweep) runs, the wait is cut the
 *        same way, so a new angle is picked up at once, and every move
 *        after the first skips B5_SERVO_PREDELAY_MS (the car stands
 *        still). Every ping is tagged with its servo angle
 *        (b5_comms_set_ping_tag()).
 *
 * NOTE: The kernel API types INT, UINT, ER, TMO and SYSTIM are used where
 * the kernel functions require them (deviation from BARR-C 5.2.a, see
 * docs/buddy5_obstacle.md).
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "b5_cfg.h"
#include "b5_comms.h"
#include "b5_fsm.h"
#include "b5_pack.h"
#include "b5_plan.h"
#include "obstacle.h"
#include "scan_algo.h"
#include "scan_hw.h"
#include "scan_task.h"

/* Microseconds per millisecond. */
#define USEC_PER_MSEC (1000U)

/* Deci-degrees per degree. */
#define DECI_PER_DEG (10U)

/* The servo's start position is unknown at boot, so assume the worst
 * case: a full 120 degree sweep. */
#define FULL_SWEEP_DECI (1200U)

/* The ping spacing in microseconds. */
#define PING_PERIOD_US (B5_PING_PERIOD_MS * USEC_PER_MSEC)

/* Flag bits that mean "a command is waiting". */
#define CMD_BITS (B5_FLG_CMD | B5_FLG_STOP)

/* Last servo command, and whether it already moved during the current
 * sweep. NOTE: The Booleans are b_ members of a g_ struct, which satisfies
 * both BARR-C 7.1.m/7.1.o and the team .clang-tidy. */
typedef struct
{
    int16_t deci;
    bool    b_known;
    bool    b_swept;
} servo_state_t;

/* Task-private state. Only this task touches it, so it is not volatile. */
static b5_fsm_t      g_fsm;
static servo_state_t g_servo        = {0, false, false};
static uint32_t      g_last_trig_us = 0U;

static uint32_t now_ms (void);
static void     handle_commands (UINT ptn);
static void     poll_commands (void);
static void     wait_ms (uint32_t delay_ms);
static void     tick (void);
static uint32_t settle_ms (int16_t target);
static bool     is_car_driving (void);
static bool     is_tracking (void);
static bool     is_sweeping (void);
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
    b5_plan_cfg_t cfg;
    b5_fsm_out_t  out;

    (void)stacd;
    (void)exinf;

    b5_plan_cfg_default(&cfg);
    b5_fsm_init(&g_fsm, &cfg, now_ms());
    b5_fsm_out_clear(&out);
    out.b_profile_changed = true;
    b5_comms_publish(&out, &g_fsm);

    /* Allow the first ping straight away. */
    g_last_trig_us = scan_hw_now_us() - PING_PERIOD_US;

    for (;;)
    {
        uint32_t since_us = 0U;

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
            uint32_t wait_for_ms =
                ((PING_PERIOD_US - since_us) + (USEC_PER_MSEC - 1U)) /
                USEC_PER_MSEC;

            /* While tracking, come back at least every B5_TRACK_PERIOD_MS
             * to re-aim the servo between pings. */
            if (((is_tracking()) || (is_sweeping())) &&
                (wait_for_ms > B5_TRACK_PERIOD_MS))
            {
                wait_for_ms = B5_TRACK_PERIOD_MS;
            }

            wait_ms(wait_for_ms);
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
    b5_fsm_out_t   out;
    uint8_t        cmd    = 0U;
    uint32_t const now    = now_ms();
    uint32_t const odo_mm = b5_comms_odometer_mm(now);

    if (0U != (ptn & B5_FLG_STOP))
    {
        b5_fsm_out_clear(&out);
        b5_fsm_command(&g_fsm, b5_pack_cmd(B5_CMD_STOP, 0U), now, odo_mm, &out);
        b5_comms_publish(&out, &g_fsm);
    }

    while (b5_comms_pop_cmd(&cmd))
    {
        b5_fsm_out_clear(&out);
        b5_fsm_command(&g_fsm, cmd, now, odo_mm, &out);
        b5_comms_publish(&out, &g_fsm);
    }
}

/*!
 * @brief Check the command bits without waiting.
 */
static void poll_commands (void)
{
    UINT     ptn    = 0U;
    ER const result = tk_wai_flg(b5_comms_flag(), CMD_BITS,
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
            tk_wai_flg(b5_comms_flag(), CMD_BITS, TWF_ORW | TWF_BITCLR, &ptn,
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
    b5_fsm_out_t   out;
    uint32_t const now = now_ms();

    b5_fsm_out_clear(&out);
    b5_fsm_tick(&g_fsm, now, b5_comms_odometer_mm(now), &out);
    b5_comms_publish(&out, &g_fsm);
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

    return (((delta_deci * B5_SETTLE_MS_PER_DEG) / DECI_PER_DEG) +
            B5_SETTLE_BASE_MS);
}

/*!
 * @brief Tell whether the car may be driving in the current state (line
 *        following in MONITOR, or the bypass).
 *
 * @return True in MONITOR, BYPASS and REACQUIRE.
 */
static bool is_car_driving (void)
{
    return ((B5_STATE_MONITOR == g_fsm.state) ||
            (B5_STATE_BYPASS == g_fsm.state) ||
            (B5_STATE_REACQUIRE == g_fsm.state));
}

/*!
 * @brief Tell whether Navigation's rotation scan has servo tracking on.
 *        Only honoured in REACQUIRE, so a flag left on by mistake cannot
 *        change the servo handling anywhere else.
 *
 * @return True while tracking.
 */
static bool is_tracking (void)
{
    return ((b5_comms_track()) && (B5_STATE_REACQUIRE == g_fsm.state));
}

/*!
 * @brief Tell whether Navigation's side scan (sweep) is running. Only
 *        honoured in REACQUIRE, like is_tracking().
 *
 * @return True while sweeping.
 */
static bool is_sweeping (void)
{
    return ((b5_comms_sweep()) && (B5_STATE_REACQUIRE == g_fsm.state));
}

/*!
 * @brief Move the servo if the statechart wants a new angle.
 *
 * NOTE: While tracking (is_tracking()) the servo is set at once, with no
 * B5_SERVO_PREDELAY_MS and no settle wait, and the function reports "not
 * moved" so the ping stays on time. This deliberately skips the "stop the
 * wheels before the servo swings" rule below, because the car is rotating
 * while the servo turns the other way; the encoder-disturbance test in
 * docs/buddy5_obstacle.md (rotation scan checks) checks that this is safe.
 * While sweeping (is_sweeping()) only the first move keeps
 * B5_SERVO_PREDELAY_MS; the car stands still for the rest of the sweep.
 *
 * @return True if the servo moved (and has now settled).
 */
static bool move_servo (void)
{
    bool    b_moved = false;
    int16_t target  = b5_fsm_servo_deci(&g_fsm, b5_comms_look());

    if (is_tracking())
    {
        if ((!g_servo.b_known) || (target != g_servo.deci))
        {
            scan_hw_servo_set(target);
            g_servo.deci    = target;
            g_servo.b_known = true;
        }
    }
    else if (g_servo.b_known && (target != g_servo.deci) &&
             (is_car_driving()) && (!g_servo.b_swept))
    {
        /* Let Navigation stop the wheels before the servo swings: a servo
         * swing while a motor drives disturbs the wheel encoders. The
         * target may change meanwhile, so read it again. */
        wait_ms(B5_SERVO_PREDELAY_MS);
        target = b5_fsm_servo_deci(&g_fsm, b5_comms_look());
    }

    /* After the tracking branch the servo already matches the target, so
     * this settle step only runs outside tracking. */
    if ((!g_servo.b_known) || (target != g_servo.deci))
    {
        uint32_t const delay_ms = settle_ms(target);

        scan_hw_servo_set(target);
        g_servo.deci    = target;
        g_servo.b_known = true;
        wait_ms(delay_ms);
        b_moved = true;
    }

    /* The first move of a sweep keeps the pre-delay (the car may still be
     * braking from its last rotation); later ones skip it. */
    g_servo.b_swept = is_sweeping() && (g_servo.b_swept || b_moved);

    return (b_moved);
}

/*!
 * @brief One ping: re-arm, trigger, wait up to 40 ms, classify.
 *
 * @param[out] p_cls     Echo class (B5_CLASS_*).
 * @param[out] p_dist_mm Distance; 0 unless the class is VALID.
 */
static void ping (uint8_t * p_cls, uint16_t * p_dist_mm)
{
    bool b_stuck = false;

    /* Drop a stale ECHO bit left over from a late edge. */
    (void)tk_clr_flg(b5_comms_flag(), ~(UINT)B5_FLG_ECHO);
    b_stuck        = scan_hw_rearm();
    g_last_trig_us = scan_hw_now_us();
    *p_dist_mm     = 0U;

    if (b_stuck)
    {
        *p_cls = B5_CLASS_STUCK;
    }
    else
    {
        UINT     ptn    = 0U;
        ER       result = E_OK;
        uint32_t mbx    = 0U;

        scan_hw_trigger();

        /* Cast is safe: B5_ECHO_WAIT_MS is 40. */
        result = tk_wai_flg(b5_comms_flag(), B5_FLG_ECHO, TWF_ORW | TWF_BITCLR,
                            &ptn, (TMO)B5_ECHO_WAIT_MS);
        mbx    = scan_hw_mailbox();
        *p_cls = scan_algo_classify(mbx, (E_OK != result),
                                    scan_hw_is_echo_high(), false);

        if (B5_CLASS_VALID == *p_cls)
        {
            *p_dist_mm = scan_algo_dist_mm(mbx & B5_MBX_WIDTH_MASK);
        }
    }
}

/*!
 * @brief Take one ping and feed it to the statechart.
 */
static void ping_and_step (void)
{
    b5_fsm_out_t out;
    uint8_t      cls     = B5_CLASS_UNKNOWN;
    uint16_t     dist_mm = 0U;

    ping(&cls, &dist_mm);
    b5_fsm_out_clear(&out);
    b5_fsm_set_guard_off(&g_fsm, b5_comms_sweep());
    b5_fsm_ping(&g_fsm, cls, dist_mm, now_ms(), &out);
    b5_comms_set_ping_tag(g_fsm.seq, g_servo.deci);
    b5_comms_publish(&out, &g_fsm);
}

/*** end of file ***/

