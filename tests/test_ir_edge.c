/** @file test_ir_edge.c
 * @brief Host tests for the barcode edge decision and the alarm catch-up in
 * src/track/ir_edge.h.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "ir_edge.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define PERIOD   5000U
#define PIN_BIT  (1U << IR_BARCODE_PIN)
#define LEFT_BIT (1U << IR_LEFT_PIN)

static void test_event (void)
{
    printf("barcode_event\n");
    assert(IR_BARCODE_RISE == ir_barcode_event(PIN_BIT, 0U));
    assert(IR_BARCODE_FALL == ir_barcode_event(0U, PIN_BIT));
    assert(IR_BARCODE_PULSE == ir_barcode_event(PIN_BIT, PIN_BIT));
    assert(IR_BARCODE_NONE == ir_barcode_event(0U, 0U));
    /* Other pins in the snapshot do not matter. */
    assert(IR_BARCODE_NONE == ir_barcode_event(LEFT_BIT, LEFT_BIT));
    assert(IR_BARCODE_RISE == ir_barcode_event(PIN_BIT | LEFT_BIT, LEFT_BIT));
}

static void test_alarm (void)
{
    uint32_t missed = 99U;

    printf("alarm_next\n");
    /* On time: one period ahead, nothing missed. */
    assert(15000U == ir_alarm_next(10000U, 10100U, PERIOD, &missed));
    assert(0U == missed);
    /* Exactly at the next target: one period is missed. */
    assert(20000U == ir_alarm_next(10000U, 15000U, PERIOD, &missed));
    assert(1U == missed);
    /* Late by 2.5 periods. */
    assert(25000U == ir_alarm_next(10000U, 22500U, PERIOD, &missed));
    assert(2U == missed);
    /* A clock jump of 1000 s is one division, not 199999 loops. */
    assert(1000000000U == ir_alarm_next(0U, 999999999U, PERIOD, &missed));
    assert(199999U == missed);
    /* Wrap of the 32 bit timer: the next target is still ahead of now. */
    assert(2000U == ir_alarm_next(0xFFFFFFFFU - 2999U, 500U, PERIOD, &missed));
    assert(0U == missed);
    assert(7000U == ir_alarm_next(0xFFFFFFFFU - 2999U, 2500U, PERIOD, &missed));
    assert(1U == missed);
}

int main (void)
{
    test_event();
    test_alarm();
    printf("ir_edge: all tests passed\n");
    return 0;
}
