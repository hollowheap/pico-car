#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>

typedef struct
{
    int32_t hi;
    uint32_t lo;
} SYSTIM;

ER tk_get_otm (SYSTIM * p_time);

/* Inject sonar events, time and wheel commands without running Pico W drivers.
 * The sweep/rejoin cases enter a phase directly and check its port output. */
#include "../../app_program/motion/obstacle_extras/navigation.c"
#include "statechart.h"

static uint32_t g_now_ms;
static uint32_t g_status_word;
static uint32_t g_tag;
static uint32_t g_event;
static bool g_event_pending;
static unsigned int g_commands;
static uint8_t g_last_command;
static unsigned int g_halts;
static bool g_held;
static int16_t g_look = INT16_MIN;
static int32_t g_left_speed;
static int32_t g_right_speed;

ER tk_get_otm (SYSTIM * p_time)
{
    p_time->hi = 0;
    p_time->lo = g_now_ms;
    return E_OK;
}

void tm_printf (const UB * p_format, ...)
{
    (void)p_format;
}

ER obstacle_extras_setup (obstacle_extras_links_t const * p_links)
{
    (void)p_links;
    return E_OK;
}

ER obstacle_extras_post_cmd (uint8_t cmd)
{
    g_last_command = cmd;
    g_commands++;
    return E_OK;
}

ER obstacle_extras_get_event (uint32_t * p_event, TMO tmout)
{
    (void)tmout;
    if (!g_event_pending)
    {
        return E_TMOUT;
    }
    *p_event = g_event;
    g_event_pending = false;
    return E_OK;
}

uint32_t obstacle_extras_get_status (void)
{
    return g_status_word;
}

uint32_t obstacle_extras_get_ping_tag (void)
{
    return g_tag;
}

uint32_t obstacle_extras_get_steer (void)
{
    return 0U;
}

uint8_t obstacle_extras_get_faults (void)
{
    return 0U;
}

ER obstacle_extras_copy_profile (uint16_t * p_profile)
{
    (void)p_profile;
    return E_OBJ;
}

void obstacle_extras_set_look (int16_t deci)
{
    g_look = deci;
}

void obstacle_extras_set_track (bool enabled)
{
    (void)enabled;
}

void obstacle_extras_set_sweep (bool enabled)
{
    (void)enabled;
}

void extras_log_add (extras_log_rec_t const * p_record)
{
    if (EXTRAS_LOG_HALT == p_record->kind)
    {
        g_halts++;
    }
}

uint32_t extras_log_profile (uint8_t id, uint16_t const * p_profile)
{
    (void)id;
    (void)p_profile;
    return 0U;
}

void extras_log_service (void)
{
}

ER motion_service_read_status (motion_service_status_t * p_status)
{
    (void)p_status;
    return E_OBJ;
}

ER motion_service_set_hold (uint32_t reason, bool held)
{
    if (MOTION_HOLD_OBSTACLE == reason)
    {
        g_held = held;
    }
    return E_OK;
}

int motion_set_speed (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    (void)ttl_us;
    g_left_speed = left_um_s;
    g_right_speed = right_um_s;
    return E_OK;
}

#define CHECK(expr) do { if (!(expr)) { \
    (void)fprintf(stderr, "line %d: %s\n", __LINE__, #expr); return false; \
} } while (0)

static bool no_path_twice (void)
{
    CHECK(E_OK == extras_nav_init());
    CHECK(2U == g_commands);

    g_status_word = extras_pack_put(EXTRAS_STATE_STOPPED, EXTRAS_ST_CAUSE_SH,
                                   EXTRAS_ST_CAUSE_MK, EXTRAS_CAUSE_NO_SAFE_PATH);
    g_event = extras_pack_put(EXTRAS_EVT_FAULT, EXTRAS_EV_CAUSE_SH,
                              EXTRAS_EV_CAUSE_MK, EXTRAS_CAUSE_NO_SAFE_PATH);
    g_event_pending = true;
    CHECK(extras_nav_step(false, false));
    CHECK(g_held && (0U == g_halts));

    g_now_ms = 2999U;
    CHECK(extras_nav_step(false, false));
    CHECK(2U == g_commands);
    g_now_ms = 3000U;
    CHECK(extras_nav_step(false, false));
    CHECK(4U == g_commands);
    CHECK(extras_pack_cmd(EXTRAS_CMD_START, 0U) == g_last_command);

    g_now_ms += 100U;
    g_event_pending = true;
    CHECK(extras_nav_step(false, false));
    CHECK(g_held && (1U == g_halts));
    g_now_ms += 10000U;
    CHECK(extras_nav_step(false, false));
    CHECK(g_held && (4U == g_commands) && (1U == g_halts));
    return true;
}

static uint32_t ping_status (uint32_t seq)
{
    uint32_t word = extras_pack_put(0U, EXTRAS_ST_CLASS_SH,
                                    EXTRAS_ST_CLASS_MK, EXTRAS_CLASS_VALID);
    word = extras_pack_put(word, EXTRAS_ST_DIST_SH, EXTRAS_ST_DIST_MK, 200U);
    return extras_pack_put(word, EXTRAS_ST_SEQ_SH, EXTRAS_ST_SEQ_MK, seq);
}

static void tag_ping (uint32_t seq, int32_t look)
{
    g_tag = extras_pack_put(seq, EXTRAS_TAG_LOOK_SH, EXTRAS_TAG_LOOK_MK,
                            (uint32_t)(look + EXTRAS_TAG_LOOK_BIAS));
}

static bool sweep_uses_matching_ping (void)
{
    g_nav.sweep_slot = 4U;
    g_nav.sweep_mm[4] = UINT16_MAX;
    g_nav.look_deci = -370;

    tag_ping(2U, -370);
    run_sweep(ping_status(1U));
    CHECK(INT16_MIN == g_look);

    tag_ping(2U, -330);
    run_sweep(ping_status(2U));
    CHECK(INT16_MIN == g_look);

    tag_ping(3U, -370);
    run_sweep(ping_status(3U));
    CHECK(-330 == g_look);
    CHECK((0 == g_left_speed) && (0 == g_right_speed));
    return true;
}

static bool rejoin_changes_speed (void)
{
    int64_t const turn_um =
        ((int64_t)900 * EXTRAS_WHEEL_TRACK_MM * UM_PER_MM +
         EXTRAS_NAV_DECI_PER_RAD - 1) / EXTRAS_NAV_DECI_PER_RAD;

    g_nav.phase = PHASE_TO_LINE;
    g_nav.side = EXTRAS_SIDE_LEFT;
    g_nav.wheel_um[WHEEL_RIGHT] = -turn_um;
    g_nav.x_um = 300000;

    run_to_line(false);
    CHECK(g_left_speed == (int32_t)EXTRAS_NAV_SPEED_UM_S);
    CHECK(g_right_speed == g_left_speed);
    g_nav.x_um = 200000;
    run_to_line(false);
    CHECK(g_left_speed == (int32_t)EXTRAS_NAV_LINE_UM_S);
    CHECK(g_right_speed == g_left_speed);
    return true;
}

static bool reversal_pause (void)
{
    drive_speeds(EXTRAS_STEER_STRAIGHT, 70000, 70000);
    drive_speeds(EXTRAS_STEER_STRAIGHT, -70000, -70000);
    CHECK((0 == g_left_speed) && (0 == g_right_speed));
    g_now_ms = 149U;
    drive_speeds(EXTRAS_STEER_STRAIGHT, -70000, -70000);
    CHECK((0 == g_left_speed) && (0 == g_right_speed));
    g_now_ms = 150U;
    drive_speeds(EXTRAS_STEER_STRAIGHT, -70000, -70000);
    CHECK((-70000 == g_left_speed) && (-70000 == g_right_speed));
    return true;
}

static bool reacquire_close_echoes (void)
{
    extras_plan_cfg_t config;
    extras_fsm_t fsm;
    extras_fsm_out_t output;

    extras_plan_cfg_default(&config);
    extras_fsm_init(&fsm, &config, 0U);
    fsm.state = EXTRAS_STATE_REACQUIRE;
    extras_fsm_out_clear(&output);
    extras_fsm_ping(&fsm, EXTRAS_CLASS_VALID, 75U, 60U, &output);
    CHECK((EXTRAS_STATE_REACQUIRE == fsm.state) && !output.b_call_stop);
    extras_fsm_out_clear(&output);
    extras_fsm_ping(&fsm, EXTRAS_CLASS_VALID, 75U, 120U, &output);
    CHECK((EXTRAS_STATE_REACQUIRE == fsm.state) && !output.b_call_stop);
    return true;
}

int main (int argc, char ** argv)
{
    static const struct
    {
        const char * name;
        bool (*run)(void);
    } cases[] = {
        {"no_path_twice", no_path_twice},
        {"matching_ping", sweep_uses_matching_ping},
        {"rejoin_speed", rejoin_changes_speed},
        {"reversal_pause", reversal_pause},
        {"close_echoes", reacquire_close_echoes},
    };

    if (2 == argc)
    {
        for (size_t index = 0U; index < sizeof(cases) / sizeof(cases[0]); index++)
        {
            if (0 == strcmp(argv[1], cases[index].name))
            {
                return cases[index].run() ? 0 : 1;
            }
        }
    }
    return 2;
}
