/* Inject initialization failures at the kernel and GPIO boundaries. */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "barcode/action.h"
#include "platform/alarm.h"
#include "platform/isr.h"
#include "platform/log.h"
#include "track/track.h"

#if defined(TEST_CAPTURE_PILOT)
#include "pilot/pilot.h"
#endif

#define BARCODE_PIN_BIT (1U << 26)
#define LINE_LEFT_BIT   (1U << 27)
#define LINE_RIGHT_BIT  (1U << 28)
#define SAMPLE_US      5000U

static bool g_semaphores[3];
static unsigned g_created;
static bool g_b_fail_second_sem;
static bool g_b_fail_gpio;
static bool g_b_fail_timer_handler;
static bool g_b_fail_barcode_handler;
static bool g_b_barcode_handler;
static bool g_b_timer_armed;
static uint32_t g_timer_enable;
static uint32_t g_alarm_at_us;
static uint32_t g_now_us;
static uint32_t g_arm_delay_us;
static uint32_t g_gpio_in;
static gpio_irq_handler_t g_edge_handler;
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

ER gpio_irq_add_handler (UW proc, uint32_t mask, gpio_irq_handler_t handler)
{
    (void)proc;
    CHECK (g_b_timer_armed);
    CHECK (BARCODE_PIN_BIT == mask);
    g_b_barcode_handler = !g_b_fail_barcode_handler;
    g_edge_handler = g_b_barcode_handler ? handler : NULL;
    return g_b_barcode_handler ? E_OK : E_LIMIT;
}

void log_write (log_sev_t severity, log_task_t task, log_mod_t module, const char * p_format, ...)
{
    (void)severity;
    (void)task;
    (void)module;
    (void)p_format;
}

static void barcode_edges (uint32_t delay_us, bool b_rise, bool b_fall)
{
    g_now_us += delay_us;
    gpio_edge_snapshot_t const snapshot = {.timestamp_us = g_now_us,
        .rise_mask = b_rise ? BARCODE_PIN_BIT : 0U,
        .fall_mask = b_fall ? BARCODE_PIN_BIT : 0U};

    g_edge_handler (&snapshot);
}

static void test_barcode_noise (bool b_overflow)
{
    uint32_t stamp;
    bool level;

    CHECK (!ir_barcode_recent ());
    barcode_edges (SAMPLE_US, true, false);
    CHECK (ir_barcode_recent ());
    if (b_overflow)
    {
        /* A 64-slot ring can hold 63 edges until its decoder drains it. */
        for (unsigned i = 1U; i < 64U; i++)
        {
            barcode_edges (10U, (0U == (i % 2U)), (0U != (i % 2U)));
        }
    }
    else
    {
        barcode_edges (SAMPLE_US, true, true);
    }
    CHECK (1U == ir_barcode_drops ());
    CHECK (!ir_barcode_recent ());
    /* Queue the first post-loss edge behind the surviving prefix. */
    CHECK (IR_BARCODE_EDGE == ir_barcode_pop (&stamp, &level));
    barcode_edges (SAMPLE_US, true, false);
    for (unsigned read = 0U; read < (b_overflow ? 62U : 0U); read++)
    {
        CHECK (IR_BARCODE_EDGE == ir_barcode_pop (&stamp, &level));
    }
    CHECK (IR_BARCODE_EDGE_AFTER_LOSS == ir_barcode_pop (&stamp, &level));
    CHECK (!ir_barcode_pop (&stamp, &level));

    /* Interleaved credible edges in a burst still reach the decoder, but
     * must not hide a line gap until the sensor has gone quiet. */
    for (unsigned i = 0U; i < 3U; i++)
    {
        barcode_edges (BARCODE_SUPPRESS_US - 1U, true, true);
        CHECK (!ir_barcode_recent ());
        barcode_edges (SAMPLE_US, false, true);
        CHECK (!ir_barcode_recent ());
        CHECK (IR_BARCODE_EDGE_AFTER_LOSS == ir_barcode_pop (&stamp, &level));
        CHECK (!level);
    }
    barcode_edges (BARCODE_SUPPRESS_US, true, false);
    CHECK (ir_barcode_recent ());
    g_now_us += BARCODE_SUPPRESS_US;
    CHECK (!ir_barcode_recent ());
}

static void fire_line_alarm (void)
{
    g_now_us = g_alarm_at_us;
    ((void (*)(UINT))g_shim_inthdr) (LINE_TIMER_ALARM_IRQ);
}

static void run_line_alarms_for_timer_cycle (void)
{
    for (uint32_t i = 0U; i <= UINT32_MAX / SAMPLE_US; i++)
    {
        fire_line_alarm ();
    }
}

static void test_barcode_timer_wrap (void)
{
    uint32_t stamp;
    bool level;

    barcode_edges (SAMPLE_US - 2U, true, false);
    fire_line_alarm ();
    CHECK (0U == g_now_us);
    CHECK (ir_barcode_recent ());
    barcode_edges (2U, false, true);
    CHECK (ir_barcode_pop (&stamp, &level) && (UINT32_MAX - 1U == stamp) && level);
    CHECK (ir_barcode_pop (&stamp, &level) && (2U == stamp) && !level);
    CHECK (ir_barcode_recent ());

    run_line_alarms_for_timer_cycle ();
    CHECK (!ir_barcode_recent ());
    barcode_edges (2U, true, false);
    CHECK (ir_barcode_recent ());

    barcode_edges (2U, true, true);
    CHECK (!ir_barcode_recent ());
    run_line_alarms_for_timer_cycle ();
    barcode_edges (2U, false, true);
    CHECK (ir_barcode_recent ());
}

static void line_capture (uint32_t gpio, bool b_left, bool b_right, unsigned repeats)
{
    g_gpio_in = gpio;
    for (unsigned i = 0U; i < repeats; i++)
    {
        g_now_us += SAMPLE_US;
        ((void (*)(UINT))g_shim_inthdr) (LINE_TIMER_ALARM_IRQ);
        line_sample_t const sample = ir_get_line_sample ();

        CHECK (g_now_us == sample.timestamp_us);
        CHECK (b_left == sample.state.left);
        CHECK (b_right == sample.state.right);
    }
}

static void test_route_capture (void)
{
    g_gpio_in = BARCODE_PIN_BIT | LINE_LEFT_BIT;
    for (unsigned i = 0U; i < 3U; i++)
    {
        fire_line_alarm ();
    }
    line_sample_t const sample = ir_get_line_sample ();
    CHECK (sample.state.left && sample.state.barcode);
    /* An IRQ after the sample changes the live barcode channel. Route
     * observation must still see the level sampled at the line timestamp. */
    barcode_edges (2U, false, true);
    CHECK (!ir_get_state ().barcode);
    CHECK (ir_get_line_sample ().timestamp_us == sample.timestamp_us);
    CHECK (ir_get_line_sample ().state.barcode);
    g_gpio_in = LINE_LEFT_BIT;
    fire_line_alarm ();
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
/* A burst of ambiguous barcode edges must leave ordinary tape steering active. */
static void test_pilot_barcode_noise (void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd;
    pilot_meas_t m = {.b_line_left = true, .ping_echo = LOOK_ECHO_OPEN};
    pilot_init (&pilot);
    for (unsigned tick = 1; tick <= 60; tick++)
    {
        m.stamp_ms = m.now_ms = tick * 10U;
        g_now_us = m.now_ms * 1000U;
        barcode_edges (0U, true, true);
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
    CHECK (ir_barcode_drops () != 0U);
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
        (void)fprintf (stderr, "Usage: %s second-semaphore|pin-setup|timer-handler|barcode-handler|"
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
    else if (0 == strcmp (argv[1], "barcode-handler"))
    {
        g_b_fail_barcode_handler = true;
        anticipated = E_LIMIT;
    }
    else
    {
        return 2;
    }

    CHECK (anticipated == ir_capture_init ());
    CHECK (!g_semaphores[1]);
    CHECK (!g_semaphores[2]);
    CHECK (!g_b_barcode_handler);
    CHECK (NULL == g_shim_inthdr);
    CHECK (0 == g_shim_int_enabled);
    CHECK (0U == g_timer_enable);
    CHECK (!g_b_timer_armed);
    return (0U == g_failures) ? 0 : 1;
}
