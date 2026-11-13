/** @file test_isr_dispatch.c
 * @brief Host test for the GPIO dispatcher in src/platform/isr.c.
 *
 * isr.c is compiled unchanged against the kernel shim in tests/shim/tk. The
 * test models the IO_BANK0 interrupt registers and the 1 MHz timer so that it
 * can check what the dispatcher reads, acknowledges and passes to callbacks.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform/isr.h"
#include "quadrature.h"

#define INT_WORDS      4U
#define PINS_PER_WORD  8U
#define NIBBLE_BITS    4U
#define EDGE_LOW_BIT   0x4U
#define EDGE_HIGH_BIT  0x8U
#define LEVEL_HIGH_BIT 0x2U
#define EDGE_WORD_MASK 0xCCCCCCCCU
#define GPIO_INTNO     13U

#define INTR_OFFSET       0xF0U
#define PROC1_INTE_OFFSET 0x100U
#define PROC1_INTS_OFFSET 0x120U
#define PROC2_INTE_OFFSET 0x130U
#define PROC2_INTS_OFFSET 0x150U
#define TIMER_RAWL_ADDR   (TIMER_BASE + 0x28U)
#define WORD_BYTES        4U

#define TIMER_START 1000U
#define TIMER_STEP  10U

#define PIN_A0 2U
#define PIN_A1 3U
#define PIN_B0 9U
#define PIN_B1 29U
#define PIN_C0 20U
#define PIN_NONE 15U

#define MASK_A ((1U << PIN_A0) | (1U << PIN_A1))
#define MASK_B ((1U << PIN_B0) | (1U << PIN_B1))
#define MASK_C (1U << PIN_C0)

#define HANDLER_COUNT 4U
#define PIN_COUNT     30U
#define MASK_ALL      ((1U << PIN_COUNT) - 1U)

typedef void (*hook_t)(void);

/* ---- Fake hardware ------------------------------------------------------ */

static uint32_t g_ints[2][INT_WORDS];
static uint32_t g_inte[2][INT_WORDS];
static uint32_t g_ack_log[INT_WORDS];
static uint32_t g_ints_reads[INT_WORDS];
static uint32_t g_timer_value;
static uint32_t g_timer_reads;
static uint32_t g_callbacks_started;
static int      g_ack_after_callback;

FP  g_shim_inthdr = NULL;
INT g_shim_prc    = 1;

static void die (const char * what, uint32_t addr)
{
    (void)fprintf(stderr, "unexpected register %s 0x%08x\n", what, addr);
    exit(2);
}

uint32_t shim_reg_read (uint32_t addr)
{
    uint32_t off = addr - IO_BANK0_BASE;
    if (addr == TIMER_RAWL_ADDR)
    {
        uint32_t value = g_timer_value;
        g_timer_reads++;
        g_timer_value += TIMER_STEP;
        return value;
    }
    if ((off >= PROC1_INTS_OFFSET) &&
        (off < (PROC1_INTS_OFFSET + (INT_WORDS * WORD_BYTES))))
    {
        uint32_t idx = (off - PROC1_INTS_OFFSET) / WORD_BYTES;
        g_ints_reads[idx]++;
        return g_ints[0][idx];
    }
    if ((off >= PROC2_INTS_OFFSET) &&
        (off < (PROC2_INTS_OFFSET + (INT_WORDS * WORD_BYTES))))
    {
        return g_ints[1][(off - PROC2_INTS_OFFSET) / WORD_BYTES];
    }
    if ((off >= PROC1_INTE_OFFSET) &&
        (off < (PROC1_INTE_OFFSET + (INT_WORDS * WORD_BYTES))))
    {
        return g_inte[0][(off - PROC1_INTE_OFFSET) / WORD_BYTES];
    }
    if ((off >= PROC2_INTE_OFFSET) &&
        (off < (PROC2_INTE_OFFSET + (INT_WORDS * WORD_BYTES))))
    {
        return g_inte[1][(off - PROC2_INTE_OFFSET) / WORD_BYTES];
    }
    die("read", addr);
    return 0U;
}

void shim_reg_write (uint32_t addr, uint32_t value)
{
    uint32_t off = addr - IO_BANK0_BASE;
    if ((off >= INTR_OFFSET) && (off < (INTR_OFFSET + (INT_WORDS * WORD_BYTES))))
    {
        /* INTR is write-1-to-clear for the edge bits only. */
        uint32_t idx = (off - INTR_OFFSET) / WORD_BYTES;
        if (g_callbacks_started != 0U)
        {
            g_ack_after_callback = 1;
        }
        g_ack_log[idx] |= value;
        g_ints[0][idx] &= ~(value & EDGE_WORD_MASK);
        g_ints[1][idx] &= ~(value & EDGE_WORD_MASK);
        return;
    }
    if ((off >= PROC1_INTE_OFFSET) &&
        (off < (PROC1_INTE_OFFSET + (INT_WORDS * WORD_BYTES))))
    {
        g_inte[0][(off - PROC1_INTE_OFFSET) / WORD_BYTES] = value;
        return;
    }
    if ((off >= PROC2_INTE_OFFSET) &&
        (off < (PROC2_INTE_OFFSET + (INT_WORDS * WORD_BYTES))))
    {
        g_inte[1][(off - PROC2_INTE_OFFSET) / WORD_BYTES] = value;
        return;
    }
    die("write", addr);
}

/** Raises the edge bit of `pin` in the INTS of core 0. */
static void raise_edge (uint32_t pin, uint32_t edge_bit)
{
    g_ints[0][pin / PINS_PER_WORD] |=
        edge_bit << ((pin % PINS_PER_WORD) * NIBBLE_BITS);
}

static void raise_rise (uint32_t pin)
{
    raise_edge(pin, EDGE_HIGH_BIT);
}

static void raise_fall (uint32_t pin)
{
    raise_edge(pin, EDGE_LOW_BIT);
}

/* ---- Handlers ----------------------------------------------------------- */

typedef struct
{
    uint32_t             calls;
    uint32_t             first_seq;
    gpio_edge_snapshot_t snap;
} record_t;

static record_t g_rec[HANDLER_COUNT];
static hook_t   g_hook_a;
static uint32_t g_seq;

static void record (uint32_t which, const gpio_edge_snapshot_t * p_snap)
{
    g_callbacks_started++;
    if (g_rec[which].calls == 0U)
    {
        g_rec[which].first_seq = g_seq;
    }
    g_seq++;
    g_rec[which].calls++;
    g_rec[which].snap = *p_snap;
}

static void handler_a (const gpio_edge_snapshot_t * p_snap)
{
    record(0U, p_snap);
    if (g_hook_a != NULL)
    {
        g_hook_a();
    }
}

static void handler_b (const gpio_edge_snapshot_t * p_snap)
{
    record(1U, p_snap);
}

static void handler_c (const gpio_edge_snapshot_t * p_snap)
{
    record(2U, p_snap);
}

static void handler_d (const gpio_edge_snapshot_t * p_snap)
{
    record(3U, p_snap);
}

/* ---- Harness ------------------------------------------------------------ */

static int g_failures;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            (void)fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,     \
                          __LINE__, #cond);                                  \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

static void reset_hw (void)
{
    memset(g_ints, 0, sizeof(g_ints));
    memset(g_ack_log, 0, sizeof(g_ack_log));
    memset(g_ints_reads, 0, sizeof(g_ints_reads));
    memset(g_rec, 0, sizeof(g_rec));
    g_timer_value        = TIMER_START;
    g_timer_reads        = 0U;
    g_callbacks_started  = 0U;
    g_ack_after_callback = 0;
    g_hook_a             = NULL;
    g_seq                = 0U;
    g_shim_prc           = 1;
}

static void run_isr (void)
{
    g_shim_inthdr();
}

static void test_rise_and_fall_are_separate (void)
{
    reset_hw();
    raise_rise(PIN_A0);
    raise_fall(PIN_A1);
    run_isr();
    CHECK(g_rec[0].calls == 1U);
    CHECK(g_rec[0].snap.rise_mask == (1U << PIN_A0));
    CHECK(g_rec[0].snap.fall_mask == (1U << PIN_A1));
    CHECK(g_rec[1].calls == 0U);
    CHECK(g_rec[2].calls == 0U);
}

static void test_both_edges_on_one_pin (void)
{
    reset_hw();
    raise_rise(PIN_B1);
    raise_fall(PIN_B1);
    run_isr();
    CHECK(g_rec[1].calls == 1U);
    CHECK(g_rec[1].snap.rise_mask == (1U << PIN_B1));
    CHECK(g_rec[1].snap.fall_mask == (1U << PIN_B1));
}

static void test_one_timestamp_per_entry (void)
{
    reset_hw();
    raise_rise(PIN_A0);
    raise_fall(PIN_B0);
    raise_rise(PIN_C0);
    run_isr();
    CHECK(g_timer_reads == 1U);
    CHECK(g_rec[0].calls == 1U);
    CHECK(g_rec[1].calls == 1U);
    CHECK(g_rec[2].calls == 1U);
    CHECK(g_rec[0].snap.timestamp_us == TIMER_START);
    CHECK(g_rec[1].snap.timestamp_us == TIMER_START);
    CHECK(g_rec[2].snap.timestamp_us == TIMER_START);
    for (uint32_t idx = 0U; idx < INT_WORDS; idx++)
    {
        CHECK(g_ints_reads[idx] == 1U);
    }
}

static void test_ack_equals_observed_before_callbacks (void)
{
    uint32_t observed[INT_WORDS];

    reset_hw();
    raise_rise(PIN_A0);
    raise_fall(PIN_A0);
    raise_rise(PIN_B1);
    raise_fall(PIN_C0);
    raise_rise(PIN_NONE);
    /* Level bits are not edges and must not be acknowledged. */
    g_ints[0][0] |= LEVEL_HIGH_BIT;
    memcpy(observed, g_ints[0], sizeof(observed));
    for (uint32_t idx = 0U; idx < INT_WORDS; idx++)
    {
        observed[idx] &= EDGE_WORD_MASK;
    }
    run_isr();
    for (uint32_t idx = 0U; idx < INT_WORDS; idx++)
    {
        CHECK(g_ack_log[idx] == observed[idx]);
    }
    CHECK(g_callbacks_started == 3U);
    CHECK(g_ack_after_callback == 0);
}

static void edge_in_callback (void)
{
    /* One edge on a pin of a later handler, one on this handler's own pin. */
    raise_rise(PIN_B0);
    raise_fall(PIN_A0);
}

static void test_edge_during_callback_stays_pending (void)
{
    uint32_t expect_b = 8U << ((PIN_B0 % PINS_PER_WORD) * NIBBLE_BITS);
    uint32_t expect_a = 4U << ((PIN_A0 % PINS_PER_WORD) * NIBBLE_BITS);

    reset_hw();
    raise_rise(PIN_A0);
    g_hook_a = edge_in_callback;
    run_isr();
    /* B runs after A, but its edge arrived after the snapshot. */
    CHECK(g_rec[1].calls == 0U);
    CHECK(g_rec[0].snap.fall_mask == 0U);
    CHECK((g_ack_log[PIN_B0 / PINS_PER_WORD] & expect_b) == 0U);
    CHECK((g_ack_log[PIN_A0 / PINS_PER_WORD] & expect_a) == 0U);
    CHECK((g_ints[0][PIN_B0 / PINS_PER_WORD] & expect_b) == expect_b);
    CHECK((g_ints[0][PIN_A0 / PINS_PER_WORD] & expect_a) == expect_a);

    /* The pending edges re-enter the ISR and are delivered then. */
    g_hook_a = NULL;
    run_isr();
    CHECK(g_rec[1].calls == 1U);
    CHECK(g_rec[1].snap.rise_mask == (1U << PIN_B0));
    CHECK(g_rec[0].snap.fall_mask == (1U << PIN_A0));
    CHECK(g_ints[0][PIN_B0 / PINS_PER_WORD] == 0U);
    CHECK(g_ints[0][PIN_A0 / PINS_PER_WORD] == 0U);
}

static void test_handler_sees_only_its_pins (void)
{
    reset_hw();
    raise_rise(PIN_A0);
    raise_fall(PIN_B0);
    raise_rise(PIN_B1);
    raise_fall(PIN_C0);
    run_isr();
    CHECK(g_rec[0].snap.rise_mask == (1U << PIN_A0));
    CHECK(g_rec[0].snap.fall_mask == 0U);
    CHECK(g_rec[1].snap.rise_mask == (1U << PIN_B1));
    CHECK(g_rec[1].snap.fall_mask == (1U << PIN_B0));
    CHECK(g_rec[2].snap.rise_mask == 0U);
    CHECK(g_rec[2].snap.fall_mask == (1U << PIN_C0));
}

static void test_unhandled_pin_never_reaches_callback (void)
{
    reset_hw();
    raise_rise(PIN_NONE);
    raise_fall(PIN_NONE);
    run_isr();
    CHECK(g_callbacks_started == 0U);
    CHECK(g_ints[0][PIN_NONE / PINS_PER_WORD] == 0U);
}

static void test_no_edges_no_callbacks (void)
{
    reset_hw();
    run_isr();
    CHECK(g_callbacks_started == 0U);
}

static void test_level_bits_alone_do_nothing (void)
{
    reset_hw();
    for (uint32_t idx = 0U; idx < INT_WORDS; idx++)
    {
        g_ints[0][idx] = ~EDGE_WORD_MASK;
    }
    run_isr();
    CHECK(g_callbacks_started == 0U);
    for (uint32_t idx = 0U; idx < INT_WORDS; idx++)
    {
        CHECK(g_ack_log[idx] == 0U);
    }
}

/* Needs the all-pins handler (index 3). Covers every GPIO and both polarities. */
static void test_every_pin_maps_to_its_bit (void)
{
    for (uint32_t pin = 0U; pin < PIN_COUNT; pin++)
    {
        uint32_t slot = (pin % PINS_PER_WORD) * NIBBLE_BITS;

        reset_hw();
        raise_rise(pin);
        run_isr();
        CHECK(g_rec[3].calls == 1U);
        CHECK(g_rec[3].snap.rise_mask == (1U << pin));
        CHECK(g_rec[3].snap.fall_mask == 0U);
        for (uint32_t idx = 0U; idx < INT_WORDS; idx++)
        {
            uint32_t want = (idx == (pin / PINS_PER_WORD))
                                ? (EDGE_HIGH_BIT << slot)
                                : 0U;
            CHECK(g_ack_log[idx] == want);
        }

        reset_hw();
        raise_fall(pin);
        run_isr();
        CHECK(g_rec[3].calls == 1U);
        CHECK(g_rec[3].snap.rise_mask == 0U);
        CHECK(g_rec[3].snap.fall_mask == (1U << pin));
        for (uint32_t idx = 0U; idx < INT_WORDS; idx++)
        {
            uint32_t want = (idx == (pin / PINS_PER_WORD))
                                ? (EDGE_LOW_BIT << slot)
                                : 0U;
            CHECK(g_ack_log[idx] == want);
        }
    }
}

static void test_core1_status_is_used_on_core1 (void)
{
    reset_hw();
    g_shim_prc = 2;
    g_ints[1][PIN_C0 / PINS_PER_WORD] |=
        EDGE_HIGH_BIT << ((PIN_C0 % PINS_PER_WORD) * NIBBLE_BITS);
    /* An edge in core 0's status must not be consumed by core 1. */
    raise_fall(PIN_A0);
    run_isr();
    CHECK(g_rec[3].calls == 1U);
    CHECK(g_rec[3].snap.rise_mask == (1U << PIN_C0));
    CHECK(g_rec[3].snap.fall_mask == 0U);
    for (uint32_t idx = 0U; idx < INT_WORDS; idx++)
    {
        CHECK(g_ints_reads[idx] == 0U);
    }
}

static void test_quadrature_preserves_a_later_pending_edge(void)
{
    quadrature_t quadrature;

    quadrature_init(&quadrature, 0U);
    /* GPIO already reads 11, but only B rising belongs to this interrupt. */
    CHECK(quadrature_update_edges(&quadrature, 0U, 1U, 0U, 3U));
    CHECK(quadrature.states == 1U);
    CHECK(quadrature_update_edges(&quadrature, 0U, 2U, 0U, 3U));
    CHECK(quadrature.wheel[0].positive == 2U);
    CHECK(quadrature.wheel[0].invalid == 0U);

    /* If both falling edges were pending together, resynchronise and flag loss. */
    CHECK(!quadrature_update_edges(&quadrature, 0U, 0U, 3U, 0U));
    CHECK(quadrature.states == 0U);
    CHECK(quadrature.wheel[0].invalid == 1U);
}

int main (void)
{
    CHECK(gpio_irq_init() == E_OK);
    CHECK(g_shim_inthdr != NULL);
    CHECK(gpio_irq_add_handler(TP_PRC1, MASK_A, handler_a) == E_OK);
    CHECK(gpio_irq_add_handler(TP_PRC1, MASK_B, handler_b) == E_OK);
    CHECK(gpio_irq_add_handler(TP_PRC1, MASK_C, handler_c) == E_OK);
    CHECK(sizeof(gpio_edge_snapshot_t) == 12U);
    if (g_failures != 0)
    {
        return 1;
    }

    test_rise_and_fall_are_separate();
    test_both_edges_on_one_pin();
    test_one_timestamp_per_entry();
    test_ack_equals_observed_before_callbacks();
    test_edge_during_callback_stays_pending();
    test_quadrature_preserves_a_later_pending_edge();
    test_handler_sees_only_its_pins();
    test_unhandled_pin_never_reaches_callback();
    test_no_edges_no_callbacks();
    test_level_bits_alone_do_nothing();

    /* From here on an all-pins handler observes every snapshot. */
    CHECK(gpio_irq_add_handler(TP_PRC1, MASK_ALL, handler_d) == E_OK);
    test_every_pin_maps_to_its_bit();
    test_core1_status_is_used_on_core1();

    if (g_failures != 0)
    {
        (void)fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    (void)printf("isr_dispatch: all tests passed\n");
    return 0;
}
