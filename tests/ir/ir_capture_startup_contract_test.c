/* Inject initialization failures at the kernel and GPIO boundaries. */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ir/barcode/action.h"
#include "platform/alarm.h"
#include "platform/log.h"
#include "ir/line/track.h"

#if defined(TEST_CAPTURE_PILOT)
#include "motion/pilot/pilot.h"
#endif

#define BARCODE_PIN_BIT (1U << 26)
#define LINE_LEFT_BIT   (1U << 27)
#define LINE_RIGHT_BIT  (1U << 28)
#define SAMPLE_US      1000U
#define LINE_SAMPLES   5U

static bool g_semaphores[3];
static unsigned g_created;
static bool g_b_fail_second_sem;
static bool g_b_fail_gpio;
static bool g_b_fail_timer_handler;
static bool g_b_timer_armed;
static uint32_t g_timer_enable;
static uint32_t g_alarm_at_us;
static uint32_t g_now_us;
static uint32_t g_arm_delay_us;
static uint32_t g_gpio_in;
static unsigned g_failures;

FP g_shim_inthdr;
int g_shim_int_enabled;
INT g_shim_prc = TP_PRC1;

#define CHECK(cond)                                                       \
    do                                                                    \
    {                                                                     \
        if (!(cond))                                                      \
        {                                                                 \
            (void)fprintf (stderr, "FAIL line %d: %s\n", __LINE__, #cond); \
            g_failures++;                                                 \
        }                                                                 \
    } while (0)

ID tk_cre_sem (const T_CSEM * p_def)
{
    (void)p_def;
    g_created++;
    if (g_b_fail_second_sem && (2U == g_created))
    {
        return E_NOMEM;
    }
    if (g_created >= (sizeof (g_semaphores) / sizeof (g_semaphores[0])))
    {
        return E_LIMIT;
    }
    g_semaphores[g_created] = true;
    return (ID)g_created;
}

ER tk_del_sem (ID sem)
{
    if ((sem <= 0) || (sem > 2) || !g_semaphores[sem])
    {
        return E_ID;
    }
    CHECK (NULL == g_shim_inthdr);
    CHECK (0 == g_shim_int_enabled);
    CHECK (0U == g_timer_enable);
    CHECK (!g_b_timer_armed);
    g_semaphores[sem] = false;
    return E_OK;
}

ER tk_sig_sem (ID sem, INT count)
{
    (void)sem;
    (void)count;
    return E_OK;
}

ER tk_wai_sem (ID sem, INT count, TMO timeout)
{
    (void)sem;
    (void)count;
    (void)timeout;
    return E_TMOUT;
}

ER gpio_set_pin (UINT pin, UINT mode)
{
    (void)pin;
    (void)mode;
    return g_b_fail_gpio ? E_PAR : E_OK;
}

ER ir_capture_test_def_int (UINT vector, const T_DINT * p_definition)
{
    (void)vector;
    if (g_b_fail_timer_handler && (NULL != p_definition))
    {
        return E_PAR;
    }
    g_shim_inthdr = (NULL != p_definition) ? p_definition->inthdr : NULL;
    return E_OK;
}

uint32_t shim_reg_read (uint32_t addr)
{
    switch (addr)
    {
        case TIMER_INTE: return g_timer_enable;
        case TIMER_TIMERAWL: return g_now_us;
        case GPIO_IN: return g_gpio_in;
        default: return 0U;
    }
}

void shim_reg_write (uint32_t addr, uint32_t value)
{
    if (TIMER_INTE == addr)
    {
        g_timer_enable = value;
    }
    else if (TIMER_ALARM0 == addr)
    {
        g_now_us += g_arm_delay_us;
        g_arm_delay_us = 0U;
        g_alarm_at_us = value;
        g_b_timer_armed = true;
    }
    else if ((TIMER_ARMED == addr) && (0U != (value & (1U << LINE_TIMER_ALARM_NUM))))
    {
        g_b_timer_armed = false;
    }
}

void log_write (log_sev_t severity, log_task_t task, log_mod_t module, const char * p_format, ...)
{
    (void)severity;
    (void)task;
    (void)module;
    (void)p_format;
}

static void fire_samples (unsigned count)
{
    for (unsigned i = 0U; i < count; i++)
    {
        g_now_us = g_alarm_at_us;
        ((void (*)(UINT))g_shim_inthdr) (LINE_TIMER_ALARM_IRQ);
    }
}

static void barcode_level (bool b_high, unsigned count)
{
    g_gpio_in = (g_gpio_in & ~BARCODE_PIN_BIT) | (b_high ? BARCODE_PIN_BIT : 0U);
    fire_samples (count);
}

static void test_barcode_noise (bool b_overflow)
{
    uint32_t stamp;
    bool level;

    CHECK (!ir_barcode_recent ());
    barcode_level (true, 2U);
    CHECK (ir_barcode_recent ());
    if (b_overflow)
    {
        /* The 64-slot ring holds 63 edges. */
        for (unsigned i = 1U; i < 64U; i++)
        {
            barcode_level (0U == (i % 2U), 2U);
        }
    }
    else
    {
        g_alarm_at_us += SAMPLE_US;
        fire_samples (1U);
    }
    CHECK (1U == ir_barcode_drops ());
    CHECK (!ir_barcode_recent ());
    CHECK (IR_BARCODE_EDGE == ir_barcode_pop (&stamp, &level));
    barcode_level (b_overflow, 2U);
    for (unsigned i = 0U; i < (b_overflow ? 62U : 0U); i++)
    {
        CHECK (IR_BARCODE_EDGE == ir_barcode_pop (&stamp, &level));
    }
    CHECK (IR_BARCODE_EDGE_AFTER_LOSS == ir_barcode_pop (&stamp, &level));
    CHECK (IR_BARCODE_EMPTY == ir_barcode_pop (&stamp, &level));
    CHECK (!ir_barcode_recent ());

    fire_samples (BARCODE_SUPPRESS_US / SAMPLE_US + 1U);
    barcode_level (!b_overflow, 2U);
    CHECK (ir_barcode_recent ());
    g_now_us += BARCODE_SUPPRESS_US;
    CHECK (!ir_barcode_recent ());
}

static void test_barcode_chatter (void)
{
    uint32_t stamp;
    bool level;

    /* Stable transitions retain their first sample's time across chatter. */
    uint32_t const start = g_alarm_at_us;
    barcode_level (true, 2U);
    for (unsigned i = 0U; i < 3U; i++)
    {
        barcode_level (false, 1U);
        barcode_level (true, 1U);
    }
    fire_samples (22U);
    barcode_level (false, 2U);
    CHECK (IR_BARCODE_EDGE == ir_barcode_pop (&stamp, &level) && stamp == start && level);
    CHECK (IR_BARCODE_EDGE == ir_barcode_pop (&stamp, &level) && stamp == start + 30000U && !level);
    CHECK (IR_BARCODE_EMPTY == ir_barcode_pop (&stamp, &level));
    CHECK (0U == ir_barcode_drops ());
}

static void test_barcode_timer_wrap (void)
{
    uint32_t stamp;
    bool level;

    barcode_level (true, 1U);
    CHECK (0U == g_now_us);
    CHECK (!ir_barcode_recent ());
    barcode_level (true, 1U);
    CHECK (ir_barcode_recent ());
    CHECK (IR_BARCODE_EDGE == ir_barcode_pop (&stamp, &level) && (0U == stamp) && level);
    uint32_t const fall_us = g_alarm_at_us;
    barcode_level (false, 2U);
    CHECK (IR_BARCODE_EDGE == ir_barcode_pop (&stamp, &level) && (fall_us == stamp) && !level);
    fire_samples (BARCODE_SUPPRESS_US / SAMPLE_US + 1U);
    CHECK (!ir_barcode_recent ());
}

static void line_capture (uint32_t gpio, bool b_left, bool b_right, unsigned repeats)
{
    g_gpio_in = gpio;
    for (unsigned i = 0U; i < repeats; i++)
    {
        fire_samples (LINE_SAMPLES);
        line_sample_t const sample = ir_get_line_sample ();
        CHECK (g_now_us == sample.timestamp_us);
        CHECK (b_left == sample.state.left);
        CHECK (b_right == sample.state.right);
    }
}

static void test_route_capture (void)
{
    g_gpio_in = BARCODE_PIN_BIT | LINE_LEFT_BIT;
    fire_samples (3U * LINE_SAMPLES);
    line_sample_t const sample = ir_get_line_sample ();
    CHECK (sample.state.left && sample.state.barcode);
    barcode_level (false, 2U);
    CHECK (!ir_get_state ().barcode);
    CHECK (ir_get_line_sample ().timestamp_us == sample.timestamp_us);
    CHECK (ir_get_line_sample ().state.barcode);
    fire_samples (LINE_SAMPLES - 2U);
    CHECK (!ir_get_line_sample ().state.barcode);
}

static void test_line_capture (void)
{
    line_capture (LINE_LEFT_BIT, false, false, 2U);
    line_capture (LINE_LEFT_BIT, true, false, 1U);
    line_capture (0U, true, false, 2U);
    line_capture (0U, false, false, 1U);
    line_capture (LINE_RIGHT_BIT, false, false, 2U);
    line_capture (LINE_RIGHT_BIT, false, true, 1U);
}

#if defined(TEST_CAPTURE_PILOT)
/* Single-sample barcode glitches leave ordinary tape steering active. */
static void test_pilot_barcode_noise (void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd;
    pilot_meas_t m = {.b_line_left = true, .ping_echo = LOOK_ECHO_OPEN};
    pilot_init (&pilot);
    for (unsigned tick = 1; tick <= 60; tick++)
    {
        m.stamp_ms = m.now_ms = tick * 10U;
        barcode_level (true, 1U);
        barcode_level (false, 9U);
        m.b_barcode = ir_barcode_recent ();
        m.ping_seq = (uint8_t)((tick / 5U) & OBSTACLE_PING_SEQ_MK);
        m.left_um += 500;
        m.right_um += 1000;
        pilot_step (&pilot, &m, &cmd);
        CHECK (!m.b_barcode && cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE);
        CHECK (cmd.speed_mm_s > 0);
        if (tick > 1U)
        {
            CHECK (cmd.turn_deci_s > 0);
        }
    }
    CHECK (ir_barcode_drops () == 0U);
}
#endif

int main (int argc, char ** argv)
{
    ER anticipated = E_OK;
    if (argc == 2 && strcmp(argv[1], "late-arm") == 0)
    {
        g_arm_delay_us = SAMPLE_US + 1U;
        CHECK(E_OK == ir_capture_init());
        CHECK((int32_t)(g_alarm_at_us - g_now_us) > 0);
        g_now_us = g_alarm_at_us + 2U * SAMPLE_US - 1U;
        g_arm_delay_us = 2U;
        ((void (*)(UINT))g_shim_inthdr)(LINE_TIMER_ALARM_IRQ);
        CHECK((int32_t)(g_alarm_at_us - g_now_us) > 0);
        return g_failures == 0U ? 0 : 1;
    }

    if (2 != argc)
    {
        (void)fprintf (stderr, "Usage: %s second-semaphore|pin-setup|timer-handler|"
                       "barcode-pulse-noise|barcode-overflow-noise|barcode-timer-wrap|line-capture\n",
                       argv[0]);
        return 2;
    }
#if defined(TEST_CAPTURE_PILOT)
    if (0 == strcmp (argv[1], "pilot-barcode-noise"))
    {
        if (E_OK != ir_capture_init ())
        {
            return 1;
        }
        test_pilot_barcode_noise ();
        return (0U == g_failures) ? 0 : 1;
    }
#endif
    if (0 == strcmp (argv[1], "barcode-timer-wrap"))
    {
        g_now_us = UINT32_MAX - SAMPLE_US + 1U;
        if (E_OK != ir_capture_init ())
        {
            return 1;
        }
        test_barcode_timer_wrap ();
        return (0U == g_failures) ? 0 : 1;
    }
    if (0 == strcmp (argv[1], "route-capture"))
    {
        if (E_OK != ir_capture_init ())
        {
            return 1;
        }
        test_route_capture ();
        return (0U == g_failures) ? 0 : 1;
    }
    if (0 == strcmp (argv[1], "barcode-chatter"))
    {
        if (E_OK != ir_capture_init ())
        {
            return 1;
        }
        test_barcode_chatter ();
        return (0U == g_failures) ? 0 : 1;
    }
    if ((0 == strcmp (argv[1], "barcode-pulse-noise")) ||
        (0 == strcmp (argv[1], "barcode-overflow-noise")) || (0 == strcmp (argv[1], "line-capture")))
    {
        if (E_OK != ir_capture_init ())
        {
            return 1;
        }
        if (0 == strcmp (argv[1], "line-capture"))
        {
            test_line_capture ();
        }
        else
        {
            test_barcode_noise (0 == strcmp (argv[1], "barcode-overflow-noise"));
        }
        return (0U == g_failures) ? 0 : 1;
    }
    if (0 == strcmp (argv[1], "second-semaphore"))
    {
        g_b_fail_second_sem = true;
        anticipated = E_NOMEM;
    }
    else if (0 == strcmp (argv[1], "pin-setup"))
    {
        g_b_fail_gpio = true;
        anticipated = E_PAR;
    }
    else if (0 == strcmp (argv[1], "timer-handler"))
    {
        g_b_fail_timer_handler = true;
        anticipated = E_PAR;
    }
    else
    {
        return 2;
    }

    CHECK (anticipated == ir_capture_init ());
    CHECK (!g_semaphores[1]);
    CHECK (!g_semaphores[2]);
    CHECK (NULL == g_shim_inthdr);
    CHECK (0 == g_shim_int_enabled);
    CHECK (0U == g_timer_enable);
    CHECK (!g_b_timer_armed);
    return (0U == g_failures) ? 0 : 1;
}
