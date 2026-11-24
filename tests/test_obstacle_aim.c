/** Run the scanner task against a scripted clock, servo and sonar. */
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "comms.h"
#include "obstacle_pack.h"
#include "platform/liveness.h"
#include "scan_hw.h"
#include "scan_task.h"

typedef struct
{
    int16_t  angle;
    unsigned writes;
    uint32_t ms;
} aim_t;

typedef enum
{
    STOP_NONE,
    STOP_SETTLE,
    STOP_SPACING,
    STOP_ECHO,
    STOP_LATE_ECHO,
    STOP_PENDING
} stop_case_t;

static jmp_buf  g_finish;
static uint32_t g_ms;
static uint32_t g_request;
static uint32_t g_last_write_ms;
static uint32_t g_first_forward_ms;
static unsigned g_pass;
static unsigned g_pings;
static unsigned g_replies;
static unsigned g_writes;
static unsigned g_aim_count;
static unsigned g_fail;
static int16_t  g_servo;
static int16_t  g_direction = 1;
static aim_t    g_aims[8];
static stop_case_t g_stop_case;
static bool g_stop_pending;
static bool g_stop_delivered;
static unsigned g_cancel_pass;
static unsigned g_cancel_triggers;
static uint8_t g_last_reply;

/* Milliseconds after each trigger. The final rising edge never falls. */
static const struct {
    uint32_t rise_ms;
    uint32_t fall_ms;
    uint8_t cls;
} g_echoes[] = {
    {1U, 47U, OBSTACLE_CLASS_OPEN},
    {38U, 42U, OBSTACLE_CLASS_VALID},
    {UINT32_MAX, UINT32_MAX, OBSTACLE_CLASS_NO_ECHO},
    {1U, UINT32_MAX, OBSTACLE_CLASS_NO_ECHO},
};
static bool g_echo_timing;
static uint32_t g_trigger_ms;
static unsigned g_class_count;
static bool g_late_request;

#define CHECK(cond) do { if (!(cond)) { \
    (void)printf("FAIL line %d: %s\n", __LINE__, #cond); g_fail++; \
} } while (0)

static void observe_aim (int16_t commanded_deci)
{
    CHECK(g_aim_count < sizeof(g_aims) / sizeof(g_aims[0]));
    CHECK(commanded_deci == g_servo);
    if (g_aim_count < sizeof(g_aims) / sizeof(g_aims[0]))
    {
        g_aims[g_aim_count] = (aim_t){commanded_deci, g_writes, g_ms};
    }
    if (g_aim_count != 0U)
    {
        CHECK(g_last_write_ms == g_ms);
    }
    g_aim_count++;
}

void liveness_kick (uint8_t task_bit)
{
    (void)task_bit;
    g_pass++;

    if (g_echo_timing)
    {
        if (g_class_count == 5U) longjmp(g_finish, 1);
        return;
    }
    if (g_stop_case != STOP_NONE)
    {
        if (g_pass == 2U || (g_stop_case == STOP_SPACING && g_pass == 3U))
        {
            g_request = obstacle_pack_ping_req(800, g_pass - 1U);
            g_stop_pending = g_stop_case == STOP_PENDING;
        }
        else if (g_pass == 6U)
        {
            g_request = obstacle_pack_ping_req(-800, 9U);
        }
        else if (g_pass == 10U)
        {
            longjmp(g_finish, 1);
        }
        return;
    }

    switch (g_pass)
    {
        case 2U:
            g_request = obstacle_pack_ping_req((int16_t)(800 * g_direction), 1U);
            break;
        case 3U:
            g_request = obstacle_pack_ping_req((int16_t)(600 * g_direction), 2U);
            break;
        case 4U:
            g_request = obstacle_pack_ping_req((int16_t)(-800 * g_direction), 3U);
            break;
        case 5U:
            g_ms += 100U;
            break;
        case 8U:
            longjmp(g_finish, 1);
            break;
        default:
            break;
    }
}

ER tk_get_otm (SYSTIM * p_time)
{
    p_time->lo = g_ms;
    return E_OK;
}

ER tk_clr_flg (ID flag, UINT pattern)
{
    (void)flag;
    (void)pattern;
    return E_OK;
}

ER tk_wai_flg (ID flag, UINT bits, UINT mode, UINT * p_flags, TMO timeout)
{
    (void)flag;
    *p_flags = 0U;
    if ((bits & OBSTACLE_FLG_ECHO) != 0U)
    {
        if (g_echo_timing)
        {
            uint32_t elapsed = g_ms - g_trigger_ms;
            uint32_t fall = g_echoes[g_pings - 1U].fall_ms;
            if (fall > elapsed + (uint32_t)timeout)
            {
                g_ms += (uint32_t)timeout;
                return E_TMOUT;
            }
            if (fall > elapsed) g_ms += fall - elapsed;
            *p_flags = OBSTACLE_FLG_ECHO;
            return E_OK;
        }
        if (g_late_request)
        {
            uint32_t elapsed = g_ms - g_trigger_ms;
            if (elapsed < 40U)
            {
                g_ms += (uint32_t)timeout;
                return E_TMOUT;
            }
            CHECK((bits & OBSTACLE_FLG_STOP) != 0U);
            g_ms += 3U;
            g_stop_pending = (mode & TWF_BITCLR) == 0U;
            *p_flags = OBSTACLE_FLG_STOP;
            return E_OK;
        }
        if (g_stop_case == STOP_ECHO && g_pass == g_cancel_pass)
        {
            g_stop_pending = true;
        }
        *p_flags = OBSTACLE_FLG_ECHO;
        g_ms++;
        return E_OK;
    }
    if (!g_stop_delivered && (bits & OBSTACLE_FLG_STOP) != 0U &&
        (g_stop_pending ||
         ((g_stop_case == STOP_SETTLE || g_stop_case == STOP_SPACING) &&
          g_pass == g_cancel_pass && timeout > TMO_POL)))
    {
        g_stop_pending   = false;
        g_stop_delivered = true;
        *p_flags = OBSTACLE_FLG_STOP;
        if (timeout > TMO_POL)
        {
            g_ms++;
        }
        return E_OK;
    }
    g_ms += (uint32_t)timeout;
    return E_TMOUT;
}

ID obstacle_comms_flag (void) { return 1; }

bool obstacle_comms_pop_cmd (uint8_t * p_cmd)
{
    (void)p_cmd;
    return false;
}

bool obstacle_comms_travel (obstacle_travel_t * p_travel)
{
    *p_travel = (obstacle_travel_t){0};
    return true;
}

bool obstacle_comms_is_pivoting (void) { return false; }
int16_t obstacle_comms_look (void) { return 0; }

void obstacle_comms_publish (obstacle_statechart_out_t const * p_out,
                             obstacle_statechart_t const * p_fsm)
{
    (void)p_out;
    (void)p_fsm;
}

uint32_t obstacle_comms_take_ping_request (void)
{
    uint32_t const request = g_request;

    g_request = 0U;
    return request;
}

void obstacle_comms_publish_ping_reply (uint32_t reply)
{
    g_replies++;
    g_last_reply = obstacle_pack_ping_seq(reply);
    if (g_stop_case != STOP_NONE)
    {
        CHECK(g_pass != g_cancel_pass);
    }
    else
    {
        CHECK(g_last_reply == g_replies);
    }
    CHECK(obstacle_pack_ping_servo(reply) == g_servo);
    CHECK(obstacle_pack_ping_echo(reply) == OBSTACLE_CLASS_VALID);
}

void obstacle_comms_count_class (uint8_t cls)
{
    if (g_echo_timing)
    {
        if (g_class_count < 4U)
        {
            CHECK(cls == g_echoes[g_class_count].cls);
            CHECK(g_ms - g_trigger_ms <= 60U);
            if (g_class_count == 2U) CHECK(g_ms - g_trigger_ms == 40U);
        }
        else
        {
            CHECK(cls == OBSTACLE_CLASS_STUCK);
        }
        g_class_count++;
        return;
    }
    CHECK(cls == OBSTACLE_CLASS_VALID);
}

void scan_hw_servo_set (int16_t deci)
{
    g_servo         = deci;
    g_last_write_ms = g_ms;
    g_writes++;
}

uint32_t scan_hw_now_us (void) { return g_ms * 1000U; }
bool scan_hw_is_echo_high (void)
{
    if (g_late_request) return (g_ms - g_trigger_ms < 48U);
    if (!g_echo_timing || g_pings == 0U) return false;
    uint32_t elapsed = g_ms - g_trigger_ms;
    return elapsed >= g_echoes[g_pings - 1U].rise_ms &&
           elapsed < g_echoes[g_pings - 1U].fall_ms;
}
bool scan_hw_rearm (void) { return scan_hw_is_echo_high(); }

bool scan_hw_trigger (void)
{
    bool const b_track_forward = g_stop_case == STOP_NONE ? g_pass >= 6U : g_stop_delivered;

    if (g_echo_timing)
    {
        CHECK(g_pings < 4U);
        if (g_pings >= 4U) longjmp(g_finish, 1);
        if (g_pings > 0U) CHECK(g_ms - g_trigger_ms >= 60U);
    }
    g_trigger_ms = g_ms;
    g_late_request = g_stop_case == STOP_LATE_ECHO && g_pass == g_cancel_pass;
    if (g_stop_case != STOP_NONE && g_pass == g_cancel_pass && g_servo != 0)
    {
        g_cancel_triggers++;
    }
    if (b_track_forward && (g_servo == 0) && (g_first_forward_ms == 0U))
    {
        g_first_forward_ms = g_ms;
    }
    g_pings++;
    return true;
}

uint32_t scan_hw_echo_result (uint32_t trigger_us)
{
    if (g_late_request) return OBSTACLE_ECHO_RISE;
    if (g_echo_timing)
    {
        CHECK(trigger_us == g_trigger_ms * 1000U);
        uint32_t elapsed = g_ms - g_trigger_ms;
        uint32_t rise = g_echoes[g_pings - 1U].rise_ms;
        uint32_t fall = g_echoes[g_pings - 1U].fall_ms;
        if (elapsed < rise) return 0U;
        if (elapsed < fall) return OBSTACLE_ECHO_RISE;
        return OBSTACLE_ECHO_RISE | OBSTACLE_ECHO_DONE | ((fall - rise) * 1000U);
    }
    return OBSTACLE_ECHO_RISE | OBSTACLE_ECHO_DONE | 2000U;
}

int main (int argc, char ** argv)
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "echo-timing") == 0) g_echo_timing = true;
        else if (strcmp(argv[1], "stop-settle") == 0) g_stop_case = STOP_SETTLE;
        else if (strcmp(argv[1], "stop-spacing") == 0) g_stop_case = STOP_SPACING;
        else if (strcmp(argv[1], "stop-echo") == 0) g_stop_case = STOP_ECHO;
        else if (strcmp(argv[1], "stop-late-echo") == 0) g_stop_case = STOP_LATE_ECHO;
        else if (strcmp(argv[1], "stop-pending") == 0) g_stop_case = STOP_PENDING;
        else g_direction = -1;
    }
    g_cancel_pass = g_stop_case == STOP_SPACING ? 3U : 2U;
    obstacle_bind_sonar_aim(observe_aim);
    if (setjmp(g_finish) == 0)
    {
        scan_task_obstacle_task(0, NULL);
    }
    if (g_echo_timing)
    {
        CHECK(g_pings == 4U && g_class_count == 5U);
        return g_fail == 0U ? 0 : 1;
    }
    if (g_stop_case != STOP_NONE)
    {
        CHECK(g_stop_delivered);
        CHECK(g_cancel_triggers == ((g_stop_case == STOP_ECHO || g_stop_case == STOP_LATE_ECHO) ? 1U : 0U));
        CHECK(g_replies == (g_stop_case == STOP_SPACING ? 2U : 1U));
        CHECK(g_last_reply == 9U);
        if (g_stop_case == STOP_PENDING)
        {
            CHECK(g_aims[1].angle == -OBSTACLE_SERVO_LIMIT_DECI);
            CHECK(g_first_forward_ms > 0U && g_first_forward_ms < g_aims[1].ms);
        }
        else
        {
            CHECK(g_aims[1].angle == OBSTACLE_SERVO_LIMIT_DECI && g_aims[2].angle == 0);
            CHECK(g_aims[2].ms >= g_aims[1].ms + 80U);
            CHECK(g_first_forward_ms >= g_aims[2].ms + 80U);
        }
        return g_fail == 0U ? 0 : 1;
    }
    CHECK(g_aim_count == 4U);
    CHECK(g_aims[0].angle == 0);
    CHECK(g_aims[0].writes == 0U);
    CHECK(g_aims[1].angle == (OBSTACLE_SERVO_LIMIT_DECI * g_direction));
    CHECK(g_aims[1].writes == 2U);
    CHECK(g_aims[2].angle == (-OBSTACLE_SERVO_LIMIT_DECI * g_direction));
    CHECK(g_aims[2].writes == 3U);
    CHECK(g_aims[3].angle == 0);
    CHECK(g_aims[3].writes == 4U);
    CHECK(g_aims[3].ms > g_aims[2].ms);
    /* 20 ms base + 3 ms per degree from the 20-degree limit. */
    CHECK(g_first_forward_ms >= g_aims[3].ms + 80U);
    CHECK(g_replies == 3U);
    CHECK(g_pings > g_replies);
    return g_fail == 0U ? 0 : 1;
}
