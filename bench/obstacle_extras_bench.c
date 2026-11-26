/** @file obstacle_extras_bench.c
 * @brief Exercise the obstacle scanner over USB with simulated navigation.
 *
 * GP20 selects a command and GP21 sends it. AUTO-RUN simulates braking,
 * accepting a scan, reacquiring tape and recovering from a fault.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "drv_bench_gpio.h"
#include "obstacle_extras_bench.h"
#include "obstacle/extras/extras.h"
#include "platform/drv_gpio_irq.h"
#include "platform/log.h"
#include "platform/platform.h"
#include "platform/usb_console_compat.h"

/* Bench buttons, active low with pull-ups. */
#define BTN_NEXT_PIN (20U)
#define BTN_SEND_PIN (21U)

/* Loop period, and one status line every 50 loops (once a second). */
#define LOOP_MS      (20U)
#define STATUS_EVERY (50U)

/* Wait up to 20 s for USB enumeration before printing. */
#define USB_WAIT_LOOPS (200U)
#define USB_WAIT_MS    (100U)
#define USB_UP         (2U)

/* AUTO-RUN delays. */
#define BRAKE_MS        (300U)
#define AUTO_LINE_MS    (4000U)
#define AUTO_RECOVER_MS (3000U)

/* Micrometres per millimetre, deci-degrees per degree. */
#define UM_PER_MM    (1000U)
#define DECI_PER_DEG (10)

/* Menu items. */
#define ITEM_AUTO       (0U)
#define ITEM_RESET      (1U)
#define ITEM_START      (2U)
#define ITEM_SCAN_REQ   (3U)
#define ITEM_ACCEPT     (4U)
#define ITEM_REJECT     (5U)
#define ITEM_LINE_FOUND (6U)
#define ITEM_STOP       (7U)
#define ITEM_PROFILE    (8U)
#define ITEM_COUNT      (9U)
#define ITEM_NONE       (0xFFU)

/* Name table sizes: 3-bit command, 2-bit class/steer/result fields. */
#define CMD_NAMES    (8U)
#define FIELD2_NAMES (4U)

/* Forces the first state change to be printed. */
#define STATUS_UNSEEN (0xFFFFFFFFU)

/* Pretend car. The obstacle-extras task calls sim_stop() and sim_distance(), so
 * both members are shared between tasks and are volatile. */
typedef struct
{
    uint32_t volatile odo_um;
    bool volatile b_estopped;
} sim_car_t;

/* Pretend-Navigation state; only the bench task touches it. */
typedef struct
{
    bool     b_auto;
    uint8_t  item;
    uint8_t  next_id;
    uint8_t  req_id;
    uint32_t state_ms;
    uint32_t wait_ms;
    uint8_t  wait_item;
} sim_nav_t;

static char const * const g_item_name[ITEM_COUNT] = {
    "AUTO-RUN on/off", "RESET",      "START", "SCAN_REQ",     "ACCEPT",
    "REJECT",          "LINE_FOUND", "STOP",  "PRINT PROFILE"};
static char const * const g_cmd_name[CMD_NAMES] = {
    "NONE",       "SCAN_REQ", "ACCEPT", "REJECT",
    "LINE_FOUND", "STOP",     "RESET",  "START"};
static char const * const g_state_name[EXTRAS_STATE_COUNT] = {
    "STOPPED", "MONITOR", "WAIT_STOP", "SCANNING",
    "PLAN",    "BYPASS",  "REACQUIRE"};
static char const * const g_cause_name[EXTRAS_CAUSE_COUNT] = {
    "NONE",          "BOOT",         "EXTERNAL",     "NO_ECHO",  "ECHO_STUCK",
    "SCAN_TIMEOUT",  "NO_SAFE_PATH", "COLLISION",    "HANDOVER", "PLAN_TIMEOUT",
    "PLAN_REJECTED", "BYPASS_LIMIT", "LINE_TIMEOUT", "INIT_FAIL"};
static char const * const g_class_name[FIELD2_NAMES]  = {"VALID", "OPEN",
                                                         "UNKNOWN", "STUCK"};
static char const * const g_steer_name[FIELD2_NAMES]  = {"STRAIGHT", "LEFT",
                                                         "RIGHT", "HOLD"};
static char const * const g_result_name[FIELD2_NAMES] = {
    "OK", "CLEAR", "NO_SAFE_PATH", "FAILED"};

/* NOTE: Boolean globals are b_ members of g_ structs, which satisfies both
 * BARR-C 7.1.m/7.1.o and the team .clang-tidy. */
static sim_car_t g_car = {0U, false};
static sim_nav_t g_nav = {true, 0U, 0U, 0U, 0U, 0U, ITEM_NONE};

static ER   sim_stop (void);
static ER   sim_distance (uint32_t * p_distance_mm);
static void print_profile (void);
static void print_event (uint32_t evt);
static void print_status (uint32_t status);
static void send_cmd (uint32_t type, uint8_t req_id);
static void send_item (uint8_t item);
static void auto_on_event (uint32_t evt);
static void auto_fire (void);
static void auto_tick (uint8_t state);
static bool is_pressed (UINT pin, bool * p_b_was_down);
static void sim_move (uint8_t state);
static void sim_nav_loop (void);


/*!
 * @brief Pretend Buddy 2 emergency stop.
 *
 * @return E_OK.
 */
static ER sim_stop (void)
{
    g_car.b_estopped = true;
    LOG_CONT("[sim-move] !!! EMERGENCY STOP called by obstacle-extras\n");

    return (E_OK);
}

/*!
 * @brief Pretend Buddy 2 odometer.
 *
 * @param[out] p_distance_mm Distance travelled.
 *
 * @return E_OK.
 */
static ER sim_distance (uint32_t * p_distance_mm)
{
    *p_distance_mm = g_car.odo_um / UM_PER_MM;

    return (E_OK);
}

/*!
 * @brief Print the 25-slot profile, one line per slot.
 */
static void print_profile (void)
{
    uint16_t profile[EXTRAS_SLOT_COUNT];
    uint8_t  slot = 0U;

    if (E_OK != obstacle_extras_copy_profile(profile))
    {
        LOG_CONT("[b5] profile busy, try again\n");
    }
    else
    {
        LOG_CONT("[b5] profile  slot  angle  class     dist\n");
        for (slot = 0U; slot < EXTRAS_SLOT_COUNT; slot++)
        {
            INT const deci =
                (INT)EXTRAS_SLOT_FIRST_DECI + ((INT)slot * (INT)EXTRAS_SLOT_STEP_DECI);

            if (extras_pack_slot_is_measured(profile[slot]))
            {
                LOG_CONT("[b5] profile  %2d  %+4d   %-8s %4d mm\n",
                          (INT)slot, deci / DECI_PER_DEG,
                          g_class_name[extras_pack_slot_class(profile[slot])],
                          (INT)extras_pack_slot_dist(profile[slot]));
            }
            else
            {
                LOG_CONT("[b5] profile  %2d  %+4d   --\n",
                          (INT)slot, deci / DECI_PER_DEG);
            }
        }
    }
}

/*!
 * @brief Print one event in words.
 *
 * @param[in] evt Packed event word.
 */
static void print_event (uint32_t evt)
{
    uint32_t const type   = extras_pack_get(evt, EXTRAS_EV_TYPE_SH, EXTRAS_EV_TYPE_MK);
    uint32_t const req_id = extras_pack_get(evt, EXTRAS_EV_ID_SH, EXTRAS_EV_ID_MK);
    uint32_t const side   = extras_pack_get(evt, EXTRAS_EV_SIDE_SH, EXTRAS_EV_SIDE_MK);

    switch (type)
    {
        case EXTRAS_EVT_OBSTACLE:
            LOG_CONT("[b5] EVENT OBSTACLE at %d mm\n",
                      (INT)extras_pack_get(evt, EXTRAS_EV_DIST_SH, EXTRAS_EV_DIST_MK));
        break;

        case EXTRAS_EVT_SCAN_RESULT:
            LOG_CONT("[b5] EVENT SCAN_RESULT id=%d %s side=%s"
                                  " width=%d mm start=%d deg\n",
                      (INT)req_id,
                      g_result_name[extras_pack_get(evt, EXTRAS_EV_STATUS_SH,
                                                EXTRAS_EV_STATUS_MK)],
                      (EXTRAS_SIDE_LEFT == side) ? "LEFT" : "RIGHT",
                      (INT)extras_pack_get(evt, EXTRAS_EV_WIDTH_SH, EXTRAS_EV_WIDTH_MK),
                      (INT)extras_pack_get(evt, EXTRAS_EV_ANGLE_SH, EXTRAS_EV_ANGLE_MK));
        break;

        case EXTRAS_EVT_FAULT:
            LOG_CONT(
                "[b5] EVENT FAULT cause=%s faults=0x%02x\n",
                g_cause_name[extras_pack_get(evt, EXTRAS_EV_CAUSE_SH, EXTRAS_EV_CAUSE_MK) %
                             EXTRAS_CAUSE_COUNT],
                (UINT)extras_pack_get(evt, EXTRAS_EV_FAULTS_SH, EXTRAS_EV_FAULTS_MK));
        break;

        default:
            LOG_CONT("[b5] EVENT 0x%08x\n", (UINT)evt);
        break;
    }
}

/*!
 * @brief Print one status line.
 *
 * @param[in] status Packed status word.
 */
static void print_status (uint32_t status)
{
    LOG_CONT("[b5] %-9s cause=%-13s last=%-7s %4d mm  seq=%2d"
                          "  faults=0x%02x drops=%d odo=%d mm\n",
              g_state_name[extras_pack_get(status, EXTRAS_ST_STATE_SH, EXTRAS_ST_STATE_MK) %
                           EXTRAS_STATE_COUNT],
              g_cause_name[extras_pack_get(status, EXTRAS_ST_CAUSE_SH, EXTRAS_ST_CAUSE_MK) %
                           EXTRAS_CAUSE_COUNT],
              g_class_name[extras_pack_get(status, EXTRAS_ST_CLASS_SH, EXTRAS_ST_CLASS_MK)],
              (INT)extras_pack_get(status, EXTRAS_ST_DIST_SH, EXTRAS_ST_DIST_MK),
              (INT)extras_pack_get(status, EXTRAS_ST_SEQ_SH, EXTRAS_ST_SEQ_MK),
              (UINT)obstacle_extras_get_faults(), (INT)obstacle_extras_dropped_events(),
              (INT)(g_car.odo_um / UM_PER_MM));
}

/*!
 * @brief Post one command to obstacle-extras and print the result.
 *
 * @param[in] type   Command type (EXTRAS_CMD_*).
 * @param[in] req_id Request id.
 */
static void send_cmd (uint32_t type, uint8_t req_id)
{
    ER const result = obstacle_extras_post_cmd(extras_pack_cmd(type, req_id));

    LOG_CONT("[sim-nav] -> %s id=%d (er=%d)\n",
              g_cmd_name[type & EXTRAS_CMD_TYPE_MK], (INT)req_id, (INT)result);
}

/*!
 * @brief Carry out one menu item.
 *
 * @param[in] item Menu item (ITEM_*).
 */
static void send_item (uint8_t item)
{
    switch (item)
    {
        case ITEM_AUTO:
            g_nav.b_auto = !g_nav.b_auto;
            LOG_CONT("[sim-nav] AUTO-RUN %s\n",
                      g_nav.b_auto ? "ON" : "OFF");
        break;

        case ITEM_RESET:
            send_cmd(EXTRAS_CMD_RESET, 0U);
        break;

        case ITEM_START:
            send_cmd(EXTRAS_CMD_START, 0U);
        break;

        case ITEM_SCAN_REQ:
            g_nav.req_id  = g_nav.next_id;
            g_nav.next_id = (uint8_t)((g_nav.next_id + 1U) & EXTRAS_ID_MAX);
            send_cmd(EXTRAS_CMD_SCAN_REQ, g_nav.req_id);
        break;

        case ITEM_ACCEPT:
            send_cmd(EXTRAS_CMD_ACCEPT, g_nav.req_id);
        break;

        case ITEM_REJECT:
            send_cmd(EXTRAS_CMD_REJECT, g_nav.req_id);
        break;

        case ITEM_LINE_FOUND:
            send_cmd(EXTRAS_CMD_LINE_FOUND, g_nav.req_id);
        break;

        case ITEM_STOP:
            send_cmd(EXTRAS_CMD_STOP, 0U);
        break;

        case ITEM_PROFILE:
            print_profile();
        break;

        default:
            /* Unknown item: nothing to do. */
        break;
    }
}

/*!
 * @brief AUTO-RUN reaction to one event.
 *
 * @param[in] evt Packed event word.
 */
static void auto_on_event (uint32_t evt)
{
    uint32_t const type = extras_pack_get(evt, EXTRAS_EV_TYPE_SH, EXTRAS_EV_TYPE_MK);

    if (EXTRAS_EVT_OBSTACLE == type)
    {
        g_nav.wait_item = ITEM_SCAN_REQ;
        g_nav.wait_ms   = BRAKE_MS;
    }
    else if (EXTRAS_EVT_FAULT == type)
    {
        g_nav.wait_item = ITEM_RESET;
        g_nav.wait_ms   = AUTO_RECOVER_MS;
    }
    else if (EXTRAS_EVT_SCAN_RESULT == type)
    {
        uint32_t const status =
            extras_pack_get(evt, EXTRAS_EV_STATUS_SH, EXTRAS_EV_STATUS_MK);

        print_profile();
        if ((EXTRAS_RESULT_OK == status) || (EXTRAS_RESULT_CLEAR == status))
        {
            send_item(ITEM_ACCEPT);
        }
    }
    else
    {
        /* STEER needs no answer. */
    }
}

/*!
 * @brief Send the pending AUTO-RUN item (RESET is followed by START).
 */
static void auto_fire (void)
{
    send_item(g_nav.wait_item);
    if (ITEM_RESET == g_nav.wait_item)
    {
        send_item(ITEM_START);
    }

    g_nav.wait_item = ITEM_NONE;
}

/*!
 * @brief AUTO-RUN timers: delayed SCAN_REQ, RESET + START, LINE_FOUND.
 *
 * @param[in] state Current obstacle-extras state.
 */
static void auto_tick (uint8_t state)
{
    if (ITEM_NONE == g_nav.wait_item)
    {
        /* Nothing is pending. */
    }
    else if (g_nav.wait_ms > LOOP_MS)
    {
        g_nav.wait_ms -= LOOP_MS;
    }
    else
    {
        auto_fire();
    }

    if ((EXTRAS_STATE_REACQUIRE == state) && (g_nav.state_ms >= AUTO_LINE_MS))
    {
        LOG_CONT("[sim-line] line sensors see the line\n");
        send_item(ITEM_LINE_FOUND);
        g_nav.state_ms = 0U;
    }
}

/*!
 * @brief Active-low button edge detector.
 *
 * @param[in]     pin          GPIO number.
 * @param[in,out] p_b_was_down Whether the button was down last time.
 *
 * @return True once per press.
 */
static bool is_pressed (UINT pin, bool * p_b_was_down)
{
    bool const b_down    = drv_bench_gpio_pressed(pin);
    bool const b_pressed = b_down && (!*p_b_was_down);

    *p_b_was_down = b_down;

    return (b_pressed);
}

/*!
 * @brief Advance the pretend odometer while obstacle-extras lets the car move.
 *
 * @param[in] state Current obstacle-extras state.
 */
static void sim_move (uint8_t state)
{
    bool const b_moving =
        (!g_car.b_estopped) &&
        ((EXTRAS_STATE_MONITOR == state) || (EXTRAS_STATE_BYPASS == state) ||
         (EXTRAS_STATE_REACQUIRE == state));

    if (b_moving)
    {
        /* mm/s multiplied by ms gives micrometres. */
        g_car.odo_um += EXTRAS_BYPASS_SPEED_MM_S * LOOP_MS;
    }
}

/*!
 * @brief Pretend-Navigation loop: print changes, drain events, read the
 *        buttons, run AUTO-RUN. Never returns.
 */
static void sim_nav_loop (void)
{
    uint32_t evt         = 0U;
    uint32_t last_steer  = 0U;
    uint32_t last_state  = STATUS_UNSEEN;
    uint32_t loops       = 0U;
    bool     b_next_down = drv_bench_gpio_pressed(BTN_NEXT_PIN);
    bool     b_send_down = drv_bench_gpio_pressed(BTN_SEND_PIN);

    for (;;)
    {
        uint32_t const status = obstacle_extras_get_status();
        uint32_t const steer  = obstacle_extras_get_steer();
        uint8_t const  state =
            (uint8_t)extras_pack_get(status, EXTRAS_ST_STATE_SH, EXTRAS_ST_STATE_MK);
        uint32_t const steer_dir =
            extras_pack_get(steer, EXTRAS_EV_STEER_SH, EXTRAS_EV_STEER_MK);

        if (last_state != state)
        {
            LOG_CONT("[b5] STATE -> %s\n",
                      g_state_name[state % EXTRAS_STATE_COUNT]);
            g_nav.state_ms = 0U;
            if (EXTRAS_STATE_MONITOR == state)
            {
                g_car.b_estopped = false;
            }
        }

        last_state = state;
        g_nav.state_ms += LOOP_MS;

        while (E_OK == obstacle_extras_get_event(&evt, TMO_POL))
        {
            print_event(evt);
            if (g_nav.b_auto)
            {
                auto_on_event(evt);
            }
        }

        if (steer_dir !=
            extras_pack_get(last_steer, EXTRAS_EV_STEER_SH, EXTRAS_EV_STEER_MK))
        {
            LOG_CONT("[b5] STEER %s\n", g_steer_name[steer_dir]);
        }

        last_steer = steer;

        if (is_pressed(BTN_NEXT_PIN, &b_next_down))
        {
            g_nav.item = (uint8_t)((g_nav.item + 1U) % ITEM_COUNT);
            LOG_CONT("[menu] > %s   (GP21 to send)\n",
                      g_item_name[g_nav.item]);
        }

        if (is_pressed(BTN_SEND_PIN, &b_send_down))
        {
            send_item(g_nav.item);
        }

        if (g_nav.b_auto)
        {
            auto_tick(state);
        }

        sim_move(state);
        loops++;
        if (0U == (loops % STATUS_EVERY))
        {
            print_status(status);
        }

        (void)tk_slp_tsk(LOOP_MS);
    }
}

/* Both buttons must be held at boot to select this motor-free mode. */
bool extras_bench_requested (void)
{
    drv_bench_gpio_input(BTN_NEXT_PIN);
    drv_bench_gpio_input(BTN_SEND_PIN);
    return drv_bench_gpio_pressed(BTN_NEXT_PIN) &&
           drv_bench_gpio_pressed(BTN_SEND_PIN);
}

/*!
 * @brief Wait for USB and run the bench simulation.
 *
 * @param[in] stacd Start code (unused).
 * @param[in] exinf Extended information (unused).
 */
void extras_bench_run (INT stacd, void * exinf)
{
    obstacle_extras_links_t const links  = {0, sim_stop, sim_distance};
    ER                     result = E_OK;
    uint32_t               tries  = 0U;

    (void)stacd;
    (void)exinf;

    for (tries = 0U; (tries < USB_WAIT_LOOPS) && (tm_usb_state() < USB_UP);
         tries++)
    {
        (void)tk_slp_tsk(USB_WAIT_MS);
    }

    (void)tk_slp_tsk(USB_WAIT_MS);
    LOG_CONT("\n=== obstacle-extras bench ===\n");
    LOG_CONT("GP20 = next menu item, GP21 = send."
                          " AUTO-RUN is ON.\n");

    result = drv_gpio_irq_init();
    LOG_CONT("[init] drv_gpio_irq_init -> %d\n", (int)result);
    if (E_OK == result)
    {
        result = obstacle_extras_setup(&links);
        LOG_CONT("[init] obstacle_extras_setup -> %d\n", (int)result);
    }

    if (E_OK == result)
    {
        platform_startup_ready();
        (void)tk_slp_tsk(USB_WAIT_MS);
        send_item(ITEM_RESET);
        send_item(ITEM_START);
        sim_nav_loop();
    }

    platform_startup_failed();
    platform_park();
}

/*** end of file ***/
