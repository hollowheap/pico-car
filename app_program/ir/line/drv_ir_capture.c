#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "ir/barcode/action.h"
#include "platform/alarm.h"
#include "platform/log.h"
#include "platform/static_assert.h"
#include "ir/line/track.h"
#include "ir_edge.h"
#include "ir_pins.h"
#include "line_filter.h"

#define BARCODE_EDGE_RING 64U
#define BITS_PER_GAP_WORD 32U
#define BARCODE_LEVEL_MASK 1U
#define BARCODE_IDLE 0U
#define BARCODE_CLEAN 1U
#define BARCODE_NOISY 2U
#define BARCODE_SAMPLE_PERIOD_US 1000U
/* Two periods since the last sample means at least one was missed. */
#define BARCODE_MISSED_SAMPLE_US (2U * BARCODE_SAMPLE_PERIOD_US)
#define LINE_SAMPLE_PERIOD_US 5000U

/**
 * Barcode edge ring, written by the TIMER alarm ISR. The decoder may run on
 * the other core. Each entry holds the edge
 * timestamp with the new level in bit 0, for a resolution of 2 us.
 */
static volatile uint32_t g_barcode_edges[BARCODE_EDGE_RING];
/* Only the ISR writes these bits, before publishing the corresponding slot. */
static volatile uint32_t g_barcode_gaps[BARCODE_EDGE_RING / BITS_PER_GAP_WORD];
STATIC_ASSERT((BARCODE_EDGE_RING % BITS_PER_GAP_WORD) == 0U, barcode_gap_words_cover_ring);
static bool g_barcode_gap_pending;
static volatile uint32_t g_barcode_head;
static volatile uint32_t g_barcode_tail;
static volatile uint32_t g_barcode_drops;
static volatile uint32_t g_barcode_last_us;
static volatile uint8_t  g_barcode_quality;
static volatile uint32_t g_barcode_sample_us;
static bool g_b_barcode_candidate;

/**
 * Line sensor TIMER alarm state.
 */
static volatile uint32_t g_line_alarm_target;
static uint32_t g_line_next_us;
static volatile uint32_t g_line_timestamp_us;
static volatile ir_sensors_state_t g_sample_state;
static volatile uint32_t g_line_overruns;

/* The TIMER alarm is the sole writer after startup. */
LOCAL volatile ir_sensors_state_t g_state = {0};

/**
 * Per-sensor 3-sample debounce. Only `line_alarm_isr` touches these
 * after `ir_capture_init` has set them up, so they need no lock.
 */
LOCAL line_debounce_t g_left_filter;
LOCAL line_debounce_t g_right_filter;

/**
 * Sensor state changes since init: filtered (what the follower sees) and raw
 * (what the comparators put out). The telemetry task turns the difference
 * between two reads into flips per second. A big raw count with a small
 * filtered count means chatter. A big filtered count on a straight line
 * means the car is hunting. Only `line_alarm_isr` writes these.
 */
static volatile uint32_t g_line_edges;
static volatile uint32_t g_line_raw_edges;
LOCAL bool               g_prev_raw_left;
LOCAL bool               g_prev_raw_right;
LOCAL ID                          g_line_sem;
LOCAL ID                          g_barcode_sem;

/** Atomically loads `p_state`. */
LOCAL ir_sensors_state_t load_state (volatile ir_sensors_state_t * p_state)
{
    /* The state is only 1 byte, so a normal load is already atomic.
     * There are no memory barriers as we are not concerned about ordering. */
    return *p_state;
}

/** Atomically stores `p_state`. */
LOCAL void store_state (volatile ir_sensors_state_t * p_state,
                        ir_sensors_state_t            new_state)
{
    /* The state is only 1 byte, so a normal store is already atomic.
     * There are no memory barriers as we are not concerned about ordering. */
    *p_state = new_state;
}

/** Reads the current sensor state from GPIO. */
LOCAL ir_sensors_state_t current_state (void)
{
    uint32_t           gpio  = in_w(GPIO_IN);
    ir_sensors_state_t state = {0};
    state.barcode            = (gpio >> IR_BARCODE_PIN) & 1U;
    state.left               = ((gpio >> IR_LEFT_PIN) & 1U) == IR_LEVEL_LINE;
    state.right              = ((gpio >> IR_RIGHT_PIN) & 1U) == IR_LEVEL_LINE;
    return state;
}

/* A past comparison does not fire until the low timer word wraps.
 * Retry a missed write unless hardware already latched the interrupt. */
LOCAL void arm_line_alarm (uint32_t target)
{
    for (;;)
    {
        out_w(TIMER_ALARM0, target);
        uint32_t const now = in_w(TIMER_TIMERAWL);
        if (((int32_t)(target - now) > 0) || (0U != (in_w(TIMER_INTR) & (1U << LINE_TIMER_ALARM_NUM))))
        {
            g_line_alarm_target = target;
            return;
        }
        uint32_t missed = 0U;
        target = ir_alarm_next(target, now, BARCODE_SAMPLE_PERIOD_US, &missed);
    }
}

static void lose_barcode_edge (uint32_t timestamp_us)
{
    g_barcode_gap_pending = true;
    g_barcode_quality = BARCODE_NOISY;
    g_barcode_last_us = timestamp_us;
    g_barcode_drops++;
}

static void publish_barcode_edge (uint32_t timestamp_us, bool b_level)
{
    bool const b_qualified = (BARCODE_NOISY != g_barcode_quality) ||
        !barcode_suppress_active(true, timestamp_us, g_barcode_last_us);
    uint32_t const next = (g_barcode_head + 1U) % BARCODE_EDGE_RING;

    g_barcode_last_us = timestamp_us;
    if (next != g_barcode_tail)
    {
        uint32_t const word = g_barcode_head / BITS_PER_GAP_WORD;
        uint32_t const bit = 1UL << (g_barcode_head % BITS_PER_GAP_WORD);
        g_barcode_gaps[word] = (g_barcode_gaps[word] & ~bit) | (g_barcode_gap_pending ? bit : 0U);
        g_barcode_gap_pending = false;
        g_barcode_edges[g_barcode_head] =
            (timestamp_us & ~BARCODE_LEVEL_MASK) | (b_level ? BARCODE_LEVEL_MASK : 0U);
        mp_memory_barrir();
        g_barcode_head = next;
        if (b_qualified)
        {
            /* Continuous traffic after a loss cannot keep steering disabled. */
            g_barcode_quality = BARCODE_CLEAN;
        }
    }
    else
    {
        lose_barcode_edge(timestamp_us);
    }
    (void)tk_sig_sem(g_barcode_sem, 1);
}

/**
 * Sample barcode at 1 kHz and retain the 5 ms line follower cadence.
 */
LOCAL void line_alarm_isr (UINT intno)
{
    (void)intno;

    /* Acknowledge raw interrupt in hardware timer. */
    out_w(TIMER_INTR, (1U << LINE_TIMER_ALARM_NUM));

    /* Re-arm from target without phase drift. */
    uint32_t const now = in_w(TIMER_TIMERAWL);
    uint32_t       missed = 0U;
    uint32_t const target =
        ir_alarm_next(g_line_alarm_target, now, BARCODE_SAMPLE_PERIOD_US, &missed);
    arm_line_alarm(target);

    uint32_t const gpio = in_w(GPIO_IN);
    bool const b_barcode = (0U != ((gpio >> IR_BARCODE_PIN) & 1U));
    ir_sensors_state_t state = load_state(&g_state);
    if ((now - g_barcode_sample_us) >= BARCODE_MISSED_SAMPLE_US)
    {
        /* One missing sample can hide a legal short bar. */
        g_b_barcode_candidate = false;
        lose_barcode_edge(now);
    }
    if (b_barcode == state.barcode)
    {
        g_b_barcode_candidate = false;
    }
    else if (g_b_barcode_candidate)
    {
        /* Date a stable transition at its first sample. */
        state.barcode = b_barcode;
        store_state(&g_state, state);
        publish_barcode_edge(g_barcode_sample_us, b_barcode);
        g_b_barcode_candidate = false;
    }
    else
    {
        g_b_barcode_candidate = true;
    }
    mp_memory_barrir();
    g_barcode_sample_us = now;

    /* Expire the cue before the low timer word can make it recent again. */
    if ((BARCODE_IDLE != g_barcode_quality) &&
        !barcode_suppress_active(true, now, g_barcode_last_us))
    {
        g_barcode_quality = BARCODE_IDLE;
    }

    if ((int32_t)(now - g_line_next_us) < 0)
    {
        return;
    }
    g_line_next_us = ir_alarm_next(g_line_next_us, now, LINE_SAMPLE_PERIOD_US, &missed);
    g_line_overruns += missed;

    /* Sample left and right sensors in one GPIO_IN read. The raw bits pass
     * through the 3-sample debounce (15 ms), so one bad sample never reaches
     * the line task. */
    bool const         raw_left  = (((gpio >> IR_LEFT_PIN) & 1U) == IR_LEVEL_LINE);
    bool const         raw_right = (((gpio >> IR_RIGHT_PIN) & 1U) == IR_LEVEL_LINE);
    bool const         old_left  = state.left;
    bool const         old_right = state.right;

    state.left  = line_debounce_update(&g_left_filter, raw_left);
    state.right = line_debounce_update(&g_right_filter, raw_right);

    g_line_raw_edges += (uint32_t)(raw_left != g_prev_raw_left) +
                        (uint32_t)(raw_right != g_prev_raw_right);
    g_line_edges += (uint32_t)(state.left != old_left) +
                    (uint32_t)(state.right != old_right);
    g_prev_raw_left  = raw_left;
    g_prev_raw_right = raw_right;

    store_state(&g_state, state);
    store_state(&g_sample_state, state);
    mp_memory_barrir();
    g_line_timestamp_us = now;

    /* Signal line task every 5 ms sample. */
    (void)tk_sig_sem(g_line_sem, 1);
}

LOCAL void ir_line_pull (UINT pin)
{
    clr_w(GPIO(pin), IR_PAD_PULL_LINE);
    set_w(GPIO(pin), IR_PAD_PULL_NO_LINE);
}

EXPORT ER ir_capture_init (void)
{
    const T_CSEM sem_def = {
        .exinf   = NULL,
        .sematr  = TA_TPRI | TA_FIRST,
        .isemcnt = 0,
        .maxsem  = 1,
    };

    ER result = gpio_set_pin(IR_BARCODE_PIN, GPIO_MODE_IN);
    if (E_OK == result)
    {
        result = gpio_set_pin(IR_LEFT_PIN, GPIO_MODE_IN);
    }
    if (E_OK == result)
    {
        result = gpio_set_pin(IR_RIGHT_PIN, GPIO_MODE_IN);
    }
    if (E_OK != result)
    {
        return result;
    }

    result = tk_cre_sem(&sem_def);
    if (result < 0)
    {
        return result;
    }
    g_line_sem = (ID)result;

    result = tk_cre_sem(&sem_def);
    if (result < 0)
    {
        (void)tk_del_sem(g_line_sem);
        g_line_sem = 0;
        return result;
    }
    g_barcode_sem = (ID)result;

    /* A line sensor that is unplugged must read as no line, so the car stops
     * instead of following a floating pin. */
    ir_line_pull(IR_LEFT_PIN);
    ir_line_pull(IR_RIGHT_PIN);
    clr_w(GPIO(IR_BARCODE_PIN), IR_PAD_PULL_BAR);
    set_w(GPIO(IR_BARCODE_PIN), IR_PAD_PULL_SPACE);
    store_state(&g_state, current_state());
    store_state(&g_sample_state, load_state(&g_state));
    /* Start the filters at the first raw level so the output is right from
     * the first sample. */
    line_debounce_init(&g_left_filter, load_state(&g_state).left);
    line_debounce_init(&g_right_filter, load_state(&g_state).right);
    g_prev_raw_left  = load_state(&g_state).left;
    g_prev_raw_right = load_state(&g_state).right;
    g_line_edges     = 0U;
    g_line_raw_edges = 0U;

    /* The alarm drives both sampling rates on core 0. */
    uint32_t const start_now = in_w(TIMER_TIMERAWL);
    g_line_alarm_target      = start_now + BARCODE_SAMPLE_PERIOD_US;
    g_line_next_us           = start_now + LINE_SAMPLE_PERIOD_US;
    g_line_timestamp_us      = start_now;
    g_barcode_sample_us      = start_now;
    g_b_barcode_candidate    = false;
    g_line_overruns          = 0U;

    DisableInt(LINE_TIMER_ALARM_IRQ);
    result = tk_def_int(LINE_TIMER_ALARM_IRQ, &(T_DINT){
                                                   .intatr = TA_HLNG,
                                                   .inthdr = (FP)line_alarm_isr,
                                               });
    if (E_OK == result)
    {
        /* The other core may share TIMER_INTE. */
        out_w(TIMER_INTR, (1U << LINE_TIMER_ALARM_NUM));
        arm_line_alarm(g_line_alarm_target);
        set_w(TIMER_INTE, (1U << LINE_TIMER_ALARM_NUM));
        ClearInt(LINE_TIMER_ALARM_IRQ);
        EnableInt(LINE_TIMER_ALARM_IRQ, LINE_TIMER_ALARM_PRI);

    }

    if (E_OK != result)
    {
        (void)tk_del_sem(g_barcode_sem);
        (void)tk_del_sem(g_line_sem);
        g_barcode_sem = 0;
        g_line_sem    = 0;
    }
    return result;
}

EXPORT ir_sensors_state_t ir_get_state (void)
{
    return load_state(&g_state);
}

EXPORT line_sample_t ir_get_line_sample (void)
{
    line_sample_t sample;
    uint32_t      stamp = 0U;

    /* The alarm may run between the two reads: retry until the timestamp holds,
     * so the state belongs to it. */
    do
    {
        stamp = g_line_timestamp_us;
        mp_memory_barrir();
        sample.state = load_state(&g_sample_state);
        mp_memory_barrir();
    } while (stamp != g_line_timestamp_us);
    sample.timestamp_us = stamp;
    return sample;
}

EXPORT uint32_t ir_line_edges (void)
{
    return g_line_edges;
}

EXPORT uint32_t ir_line_raw_edges (void)
{
    return g_line_raw_edges;
}

EXPORT uint32_t ir_line_overruns (void)
{
    return g_line_overruns;
}

EXPORT ir_barcode_read_t ir_barcode_pop (uint32_t * p_timestamp_us, bool * p_level)
{
    ir_barcode_read_t result = IR_BARCODE_EMPTY;

    if ((NULL != p_timestamp_us) && (NULL != p_level))
    {
        uint32_t const tail = g_barcode_tail;
        uint32_t const head = g_barcode_head;

        /* Read the head before the entry it publishes. */
        mp_memory_barrir();
        if (tail != head)
        {
            uint32_t const entry = g_barcode_edges[tail];
            uint32_t const bit = 1UL << (tail % BITS_PER_GAP_WORD);
            result = ((g_barcode_gaps[tail / BITS_PER_GAP_WORD] & bit) != 0U)
                         ? IR_BARCODE_EDGE_AFTER_LOSS : IR_BARCODE_EDGE;

            *p_timestamp_us = entry & ~BARCODE_LEVEL_MASK;
            *p_level        = (0U != (entry & BARCODE_LEVEL_MASK));
            /* Finish reading the entry before the ISR may reuse its slot. */
            mp_memory_barrir();
            g_barcode_tail = (tail + 1U) % BARCODE_EDGE_RING;
        }
    }

    return result;
}

EXPORT uint32_t ir_barcode_drops (void)
{
    return (g_barcode_drops);
}

EXPORT uint32_t ir_barcode_drain_limit_us (void)
{
    return g_barcode_sample_us;
}

EXPORT bool ir_barcode_recent (void)
{
    uint32_t const edge_us = g_barcode_last_us;
    uint32_t const now = in_w(TIMER_TIMERAWL);

    return (BARCODE_CLEAN == g_barcode_quality) && barcode_suppress_active(true, now, edge_us);
}

EXPORT ER ir_wait_change (uint8_t type, TMO tmout)
{
    switch (type)
    {
        case IR_TYPE_BARCODE:
            return tk_wai_sem(g_barcode_sem, 1, tmout);
        case IR_TYPE_LINE:
            return tk_wai_sem(g_line_sem, 1, tmout);
        default:
            LOG_ERROR(LOG_TASK_LINE, LOG_MOD_NONE, "bad wait type %d", (int)type);
            return E_PAR;
    }
}

/*** end of file ***/
