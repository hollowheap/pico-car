#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "barcode/action.h"
#include "platform/alarm.h"
#include "platform/isr.h"
#include "platform/log.h"
#include "track/track.h"
#include "ir_pins.h"
#include "line_filter.h"

#define IO_IRQ_BANK0 13U
#define BARCODE_EDGE_RING 64U
#define BARCODE_LEVEL_MASK 1U
#define LINE_SAMPLE_PERIOD_US 5000U

/**
 * Barcode edge ring, written by `ir_isr` and read by `ir_barcode_pop`, which
 * may run on the other core. Each entry holds the edge timestamp with the new
 * level in bit 0, so the timestamp resolution is 2 us.
 */
static volatile uint32_t g_barcode_edges[BARCODE_EDGE_RING];
static volatile uint32_t g_barcode_head;
static volatile uint32_t g_barcode_tail;
static volatile uint32_t g_barcode_drops;
static volatile uint32_t g_barcode_last_us;
static volatile bool     g_barcode_seen;

/**
 * Line sensor TIMER alarm state (D88, S31).
 */
static volatile uint32_t g_line_alarm_target;
static volatile uint32_t g_line_timestamp_us;
static volatile uint32_t g_line_overruns;

/**
 * After the isr has been initialised, this can only be written to by `ir_isr`
 * (for barcode) and `line_alarm_isr` (for left/right line sensors).
 */
LOCAL volatile ir_sensors_state_t g_state = {0};

/**
 * Per-sensor 3-sample debounce (S09). Only `line_alarm_isr` touches these
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
    state.left               = (gpio >> IR_LEFT_PIN) & 1U;
    state.right              = (gpio >> IR_RIGHT_PIN) & 1U;
    return state;
}

/**
 * 5 ms RP2040 TIMER alarm ISR on Core 0 (D88, S31).
 * Re-arms from previous target without drift, samples left and right pins,
 * records the timer timestamp, and signals the line task.
 */
LOCAL void line_alarm_isr (UINT intno)
{
    (void)intno;

    /* Acknowledge raw interrupt in hardware timer. */
    out_w(TIMER_INTR, (1U << LINE_TIMER_ALARM_NUM));

    /* Re-arm from target without phase drift. */
    uint32_t       target = g_line_alarm_target + LINE_SAMPLE_PERIOD_US;
    uint32_t const now    = in_w(TIMER_TIMERAWL);

    while ((int32_t)(target - now) <= 0)
    {
        target += LINE_SAMPLE_PERIOD_US;
        g_line_overruns++;
    }
    g_line_alarm_target = target;
    out_w(TIMER_ALARM0, target);

    /* Sample left and right sensors in one GPIO_IN read. The raw bits pass
     * through the 3-sample debounce (15 ms), so one bad sample never reaches
     * the line task. */
    uint32_t const     gpio      = in_w(GPIO_IN);
    ir_sensors_state_t state     = load_state(&g_state);
    bool const         raw_left  = (0U != ((gpio >> IR_LEFT_PIN) & 1U));
    bool const         raw_right = (0U != ((gpio >> IR_RIGHT_PIN) & 1U));
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

    g_line_timestamp_us = now;
    mp_memory_barrir();
    store_state(&g_state, state);

    /* Signal line task every 5 ms sample. */
    (void)tk_sig_sem(g_line_sem, 1);
}

/**
 * Barcode GPIO edge ISR. Line sensors are sampled via TIMER alarm, so this
 * handler only processes barcode transitions.
 */
LOCAL void ir_isr (const gpio_edge_snapshot_t * p_snapshot)
{
    uint32_t const           gpio      = in_w(GPIO_IN);
    bool const               barcode   = (0U != ((gpio >> IR_BARCODE_PIN) & 1U));
    ir_sensors_state_t const old_state = load_state(&g_state);

    if (old_state.barcode != barcode)
    {
        ir_sensors_state_t new_state = old_state;
        new_state.barcode            = barcode;
        store_state(&g_state, new_state);

        uint32_t const now  = p_snapshot->timestamp_us;
        uint32_t const next = (g_barcode_head + 1U) % BARCODE_EDGE_RING;

        /* Every edge counts for the line-lost suppression, also a dropped one.
         * The time is the ISR sample timestamp. */
        g_barcode_last_us = now;
        g_barcode_seen    = true;

        if (next != g_barcode_tail)
        {
            g_barcode_edges[g_barcode_head] =
                (now & ~BARCODE_LEVEL_MASK) |
                (barcode ? BARCODE_LEVEL_MASK : 0U);
            mp_memory_barrir();
            g_barcode_head = next;
        }
        else
        {
            g_barcode_drops++;
        }

        (void)tk_sig_sem(g_barcode_sem, 1);
    }
}

LOCAL ER ir_sensor_init (UINT pin)
{
    return gpio_set_pin(pin, GPIO_MODE_IN);
}

EXPORT ER ir_capture_init (void)
{
    const T_CSEM sem_def = {
        .exinf   = NULL,
        .sematr  = TA_TPRI | TA_FIRST,
        .isemcnt = 0,
        .maxsem  = 1,
    };

    ER result = tk_cre_sem(&sem_def);
    if (result < 0)
    {
        return result;
    }
    g_line_sem = (ID)result;

    result = tk_cre_sem(&sem_def);
    if (result < 0)
    {
        return result;
    }
    g_barcode_sem = (ID)result;

    (void)ir_sensor_init(IR_BARCODE_PIN);
    (void)ir_sensor_init(IR_LEFT_PIN);
    (void)ir_sensor_init(IR_RIGHT_PIN);
    store_state(&g_state, current_state());
    /* Start the filters at the first raw level so the output is right from
     * the first sample. */
    line_debounce_init(&g_left_filter, load_state(&g_state).left);
    line_debounce_init(&g_right_filter, load_state(&g_state).right);
    g_prev_raw_left  = load_state(&g_state).left;
    g_prev_raw_right = load_state(&g_state).right;
    g_line_edges     = 0U;
    g_line_raw_edges = 0U;

    /* Set up 5 ms line sampling TIMER alarm on Core 0 (D88, S31). */
    uint32_t const start_now = in_w(TIMER_TIMERAWL);
    g_line_alarm_target      = start_now + LINE_SAMPLE_PERIOD_US;
    g_line_timestamp_us      = start_now;
    g_line_overruns          = 0U;

    out_w(TIMER_INTR, (1U << LINE_TIMER_ALARM_NUM));
    out_w(TIMER_ALARM0, g_line_alarm_target);
    out_w(TIMER_INTE, in_w(TIMER_INTE) | (1U << LINE_TIMER_ALARM_NUM));

    DisableInt(LINE_TIMER_ALARM_IRQ);
    result = tk_def_int(LINE_TIMER_ALARM_IRQ, &(T_DINT){
                                                   .intatr = TA_HLNG,
                                                   .inthdr = (FP)line_alarm_isr,
                                               });
    if (result != E_OK)
    {
        return result;
    }
    ClearInt(LINE_TIMER_ALARM_IRQ);
    EnableInt(LINE_TIMER_ALARM_IRQ, LINE_TIMER_ALARM_PRI);

    /* Barcode edges stay on GPIO edges; line sensors removed from edge mask. */
    return gpio_irq_add_handler(tk_get_prc(),
                                (1U << IR_BARCODE_PIN),
                                ir_isr);
}

EXPORT ir_sensors_state_t ir_get_state (void)
{
    return load_state(&g_state);
}

EXPORT line_sample_t ir_get_line_sample (void)
{
    line_sample_t sample;
    sample.timestamp_us = g_line_timestamp_us;
    sample.state        = load_state(&g_state);
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

EXPORT bool ir_barcode_pop (uint32_t * p_timestamp_us, bool * p_level)
{
    bool b_popped = false;

    if ((NULL != p_timestamp_us) && (NULL != p_level))
    {
        uint32_t const tail = g_barcode_tail;
        uint32_t const head = g_barcode_head;

        /* Read the head before the entry it publishes. */
        mp_memory_barrir();
        if (tail != head)
        {
            uint32_t const entry = g_barcode_edges[tail];

            *p_timestamp_us = entry & ~BARCODE_LEVEL_MASK;
            *p_level        = (0U != (entry & BARCODE_LEVEL_MASK));
            /* Finish reading the entry before the ISR may reuse its slot. */
            mp_memory_barrir();
            g_barcode_tail = (tail + 1U) % BARCODE_EDGE_RING;
            b_popped       = true;
        }
    }

    return (b_popped);
}

EXPORT uint32_t ir_barcode_drops (void)
{
    return (g_barcode_drops);
}

EXPORT bool ir_barcode_recent (void)
{
    return barcode_suppress_active(g_barcode_seen, in_w(TIMER_TIMERAWL), g_barcode_last_us);
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
