/** @file b5_bench_main.c
 *
 * @brief Buddy 5 bench program: its own usermain, Buddy 5 and a pretend
 *        Navigation. No other buddy's code runs and the motors are never
 *        touched. Everything is printed over USB.
 *
 * Buttons (USB input is not implemented in libtm, so there is no serial
 * menu):
 *   GP20 = next menu item (the item is printed)
 *   GP21 = send it
 *
 * AUTO-RUN (on at boot) answers the way Navigation would:
 *   OBSTACLE -> wait 300 ms "braking" -> SCAN_REQ(id)
 *   SCAN_RESULT OK/CLEAR -> ACCEPT(id)
 *   REACQUIRE for 4 s -> LINE_FOUND(id)
 *   FAULT -> wait 3 s -> RESET + START
 *
 * Build (the "Buddy 5" block in the team CMakeLists.txt):
 *   ./configure.sh
 *   ./build.sh            -> build/obstacle_bench.uf2
 *
 * NOTE: Casts in this file fall into three categories, documented here
 * once rather than on every line (BARR-C 1.6.a):
 *   (UB const *) "..."  tm_printf() takes UB; it only reads the string.
 *   (INT) and (UINT)    printf arguments; every value is below 2^16.
 *   (uint8_t)           a field read from a packed word whose mask is
 *                       8 bits or narrower.
 * NOTE: usermain() is the name the kernel calls, so it has no module
 * prefix (BARR-C 6.1.i). This main module has no header (BARR-C 4.2.a).
 * The kernel API types are used where the kernel requires them.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "../sys/gpio_irq.h"
#include "../usb_console_compat.h"
#include "b5_cfg.h"
#include "b5_pack.h"
#include "obstacle.h"

/* Bench buttons, active low with pull-ups. */
#define BTN_NEXT_PIN (20U)
#define BTN_SEND_PIN (21U)

/* GPIO pad pull bits (unsigned copies of the port's GPIO_PUE/GPIO_PDE). */
#define PAD_PULL_UP   (1UL << 3U)
#define PAD_PULL_DOWN (1UL << 2U)

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

/* Pretend-Navigation task. */
#define SIM_PRI   (8)
#define SIM_STACK (4096U)

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

/* Pretend car. The Buddy 5 task calls sim_stop() and sim_distance(), so
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
static char const * const g_state_name[B5_STATE_COUNT] = {
    "STOPPED", "MONITOR", "WAIT_STOP", "SCANNING",
    "PLAN",    "BYPASS",  "REACQUIRE"};
static char const * const g_cause_name[B5_CAUSE_COUNT] = {
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
static void button_init (UINT pin);
static void sim_move (uint8_t state);
static void sim_nav_loop (void);
static void sim_task (INT stacd, void * exinf);

/*!
 * @brief Kernel entry point: start the bench task and sleep for ever.
 *
 * @return Never returns in practice; returning shuts the kernel down.
 */
INT usermain (void)
{
    ID     h_task = 0;
    T_CTSK ctsk   = {
          .exinf   = NULL,
          .itskpri = SIM_PRI,
          .stksz   = SIM_STACK,
          .task    = sim_task,
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
        (void)tk_sta_tsk(h_task, 0);
    }

    /* The initial task must not return: that shuts the kernel down. */
    (void)tk_slp_tsk(TMO_FEVR);

    return (0);
}

/*!
 * @brief Pretend Buddy 2 emergency stop.
 *
 * @return E_OK.
 */
static ER sim_stop (void)
{
    g_car.b_estopped = true;
    tm_printf((UB const *)"[sim-move] !!! EMERGENCY STOP called by Buddy 5\n");

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
    uint16_t profile[B5_SLOT_COUNT];
    uint8_t  slot = 0U;

    if (E_OK != obstacle_copy_profile(profile))
    {
        tm_printf((UB const *)"[b5] profile busy, try again\n");
    }
    else
    {
        tm_printf((UB const *)"[b5] profile  slot  angle  class     dist\n");
        for (slot = 0U; slot < B5_SLOT_COUNT; slot++)
        {
            INT const deci =
                (INT)B5_SLOT_FIRST_DECI + ((INT)slot * (INT)B5_SLOT_STEP_DECI);

            if (b5_pack_slot_is_measured(profile[slot]))
            {
                tm_printf((UB const *)"[b5] profile  %2d  %+4d   %-8s %4d mm\n",
                          (INT)slot, deci / DECI_PER_DEG,
                          g_class_name[b5_pack_slot_class(profile[slot])],
                          (INT)b5_pack_slot_dist(profile[slot]));
            }
            else
            {
                tm_printf((UB const *)"[b5] profile  %2d  %+4d   --\n",
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
    uint32_t const type   = b5_pack_get(evt, B5_EV_TYPE_SH, B5_EV_TYPE_MK);
    uint32_t const req_id = b5_pack_get(evt, B5_EV_ID_SH, B5_EV_ID_MK);
    uint32_t const side   = b5_pack_get(evt, B5_EV_SIDE_SH, B5_EV_SIDE_MK);

    switch (type)
    {
        case B5_EVT_OBSTACLE:
            tm_printf((UB const *)"[b5] EVENT OBSTACLE at %d mm\n",
                      (INT)b5_pack_get(evt, B5_EV_DIST_SH, B5_EV_DIST_MK));
        break;

        case B5_EVT_SCAN_RESULT:
            tm_printf((UB const *)"[b5] EVENT SCAN_RESULT id=%d %s side=%s"
                                  " width=%d mm start=%d deg\n",
                      (INT)req_id,
                      g_result_name[b5_pack_get(evt, B5_EV_STATUS_SH,
                                                B5_EV_STATUS_MK)],
                      (B5_SIDE_LEFT == side) ? "LEFT" : "RIGHT",
                      (INT)b5_pack_get(evt, B5_EV_WIDTH_SH, B5_EV_WIDTH_MK),
                      (INT)b5_pack_get(evt, B5_EV_ANGLE_SH, B5_EV_ANGLE_MK));
        break;

        case B5_EVT_FAULT:
            tm_printf(
                (UB const *)"[b5] EVENT FAULT cause=%s faults=0x%02x\n",
                g_cause_name[b5_pack_get(evt, B5_EV_CAUSE_SH, B5_EV_CAUSE_MK) %
                             B5_CAUSE_COUNT],
                (UINT)b5_pack_get(evt, B5_EV_FAULTS_SH, B5_EV_FAULTS_MK));
        break;

        default:
            tm_printf((UB const *)"[b5] EVENT 0x%08x\n", (UINT)evt);
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
    tm_printf((UB const *)"[b5] %-9s cause=%-13s last=%-7s %4d mm  seq=%2d"
                          "  faults=0x%02x drops=%d odo=%d mm\n",
              g_state_name[b5_pack_get(status, B5_ST_STATE_SH, B5_ST_STATE_MK) %
                           B5_STATE_COUNT],
              g_cause_name[b5_pack_get(status, B5_ST_CAUSE_SH, B5_ST_CAUSE_MK) %
                           B5_CAUSE_COUNT],
              g_class_name[b5_pack_get(status, B5_ST_CLASS_SH, B5_ST_CLASS_MK)],
              (INT)b5_pack_get(status, B5_ST_DIST_SH, B5_ST_DIST_MK),
              (INT)b5_pack_get(status, B5_ST_SEQ_SH, B5_ST_SEQ_MK),
              (UINT)obstacle_get_faults(), (INT)obstacle_dropped_events(),
              (INT)(g_car.odo_um / UM_PER_MM));
}

/*!
 * @brief Post one command to Buddy 5 and print the result.
 *
 * @param[in] type   Command type (B5_CMD_*).
 * @param[in] req_id Request id.
 */
static void send_cmd (uint32_t type, uint8_t req_id)
{
    ER const result = obstacle_post_cmd(b5_pack_cmd(type, req_id));

    tm_printf((UB const *)"[sim-nav] -> %s id=%d (er=%d)\n",
              g_cmd_name[type & B5_CMD_TYPE_MK], (INT)req_id, (INT)result);
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
            tm_printf((UB const *)"[sim-nav] AUTO-RUN %s\n",
                      g_nav.b_auto ? "ON" : "OFF");
        break;

        case ITEM_RESET:
            send_cmd(B5_CMD_RESET, 0U);
        break;

        case ITEM_START:
            send_cmd(B5_CMD_START, 0U);
        break;

        case ITEM_SCAN_REQ:
            g_nav.req_id  = g_nav.next_id;
            g_nav.next_id = (uint8_t)((g_nav.next_id + 1U) & B5_ID_MAX);
            send_cmd(B5_CMD_SCAN_REQ, g_nav.req_id);
        break;

        case ITEM_ACCEPT:
            send_cmd(B5_CMD_ACCEPT, g_nav.req_id);
        break;

        case ITEM_REJECT:
            send_cmd(B5_CMD_REJECT, g_nav.req_id);
        break;

        case ITEM_LINE_FOUND:
            send_cmd(B5_CMD_LINE_FOUND, g_nav.req_id);
        break;

        case ITEM_STOP:
            send_cmd(B5_CMD_STOP, 0U);
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
    uint32_t const type = b5_pack_get(evt, B5_EV_TYPE_SH, B5_EV_TYPE_MK);

    if (B5_EVT_OBSTACLE == type)
    {
        g_nav.wait_item = ITEM_SCAN_REQ;
        g_nav.wait_ms   = BRAKE_MS;
    }
    else if (B5_EVT_FAULT == type)
    {
        g_nav.wait_item = ITEM_RESET;
        g_nav.wait_ms   = AUTO_RECOVER_MS;
    }
    else if (B5_EVT_SCAN_RESULT == type)
    {
        uint32_t const status =
            b5_pack_get(evt, B5_EV_STATUS_SH, B5_EV_STATUS_MK);

        print_profile();
        if ((B5_RESULT_OK == status) || (B5_RESULT_CLEAR == status))
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
 * @param[in] state Current Buddy 5 state.
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

    if ((B5_STATE_REACQUIRE == state) && (g_nav.state_ms >= AUTO_LINE_MS))
    {
        tm_printf((UB const *)"[sim-line] line sensors see the line\n");
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
    bool const b_down    = (0U == gpio_get_val(pin));
    bool const b_pressed = b_down && (!*p_b_was_down);

    *p_b_was_down = b_down;

    return (b_pressed);
}

/*!
 * @brief Make a pin a pulled-up input for a button.
 *
 * @param[in] pin GPIO number.
 */
static void button_init (UINT pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), PAD_PULL_DOWN);
    set_w(GPIO(pin), PAD_PULL_UP);
}

/*!
 * @brief Advance the pretend odometer while Buddy 5 lets the car move.
 *
 * @param[in] state Current Buddy 5 state.
 */
static void sim_move (uint8_t state)
{
    bool const b_moving =
        (!g_car.b_estopped) &&
        ((B5_STATE_MONITOR == state) || (B5_STATE_BYPASS == state) ||
         (B5_STATE_REACQUIRE == state));

    if (b_moving)
    {
        /* mm/s multiplied by ms gives micrometres. */
        g_car.odo_um += B5_BYPASS_SPEED_MM_S * LOOP_MS;
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
    bool     b_next_down = false;
    bool     b_send_down = false;

    for (;;)
    {
        uint32_t const status = obstacle_get_status();
        uint32_t const steer  = obstacle_get_steer();
        uint8_t const  state =
            (uint8_t)b5_pack_get(status, B5_ST_STATE_SH, B5_ST_STATE_MK);
        uint32_t const steer_dir =
            b5_pack_get(steer, B5_EV_STEER_SH, B5_EV_STEER_MK);

        if (last_state != state)
        {
            tm_printf((UB const *)"[b5] STATE -> %s\n",
                      g_state_name[state % B5_STATE_COUNT]);
            g_nav.state_ms = 0U;
            if (B5_STATE_MONITOR == state)
            {
                g_car.b_estopped = false;
            }
        }

        last_state = state;
        g_nav.state_ms += LOOP_MS;

        while (E_OK == obstacle_get_event(&evt, TMO_POL))
        {
            print_event(evt);
            if (g_nav.b_auto)
            {
                auto_on_event(evt);
            }
        }

        if (steer_dir !=
            b5_pack_get(last_steer, B5_EV_STEER_SH, B5_EV_STEER_MK))
        {
            tm_printf((UB const *)"[b5] STEER %s\n", g_steer_name[steer_dir]);
        }

        last_steer = steer;

        if (is_pressed(BTN_NEXT_PIN, &b_next_down))
        {
            g_nav.item = (uint8_t)((g_nav.item + 1U) % ITEM_COUNT);
            tm_printf((UB const *)"[menu] > %s   (GP21 to send)\n",
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

        (void)tk_dly_tsk(LOOP_MS);
    }
}

/*!
 * @brief Bench task: wait for USB, start Buddy 5, run pretend Navigation.
 *
 * @param[in] stacd Start code (unused).
 * @param[in] exinf Extended information (unused).
 */
static void sim_task (INT stacd, void * exinf)
{
    obstacle_links_t const links  = {0, sim_stop, sim_distance};
    ER                     result = E_OK;
    uint32_t               tries  = 0U;

    (void)stacd;
    (void)exinf;

    for (tries = 0U; (tries < USB_WAIT_LOOPS) && (tm_usb_state() < USB_UP);
         tries++)
    {
        (void)tk_dly_tsk(USB_WAIT_MS);
    }

    (void)tk_dly_tsk(USB_WAIT_MS);
    tm_printf((UB const *)"\n=== Buddy 5 obstacle bench (Phase 1) ===\n");
    tm_printf((UB const *)"GP20 = next menu item, GP21 = send."
                          " AUTO-RUN is ON.\n");

    button_init(BTN_NEXT_PIN);
    button_init(BTN_SEND_PIN);

    /* Bench only: no other buddy's code is linked, so the bench starts
     * the shared dispatcher itself. */
    result = gpio_irq_init();
    tm_printf((UB const *)"[init] gpio_irq_init -> %d\n", (INT)result);
    result = obstacle_init(&links);
    tm_printf((UB const *)"[init] obstacle_init -> %d%s\n", (INT)result,
              (E_OK == result)
                  ? ""
                  : "  (E_SYS: timer, E_OBJ: PWM slice 6 busy, E_CTX: core)");

    if (E_OK == result)
    {
        (void)tk_dly_tsk(USB_WAIT_MS);
        send_item(ITEM_RESET);
        send_item(ITEM_START);
        sim_nav_loop();
    }

    (void)tk_slp_tsk(TMO_FEVR);
}

/*** end of file ***/

