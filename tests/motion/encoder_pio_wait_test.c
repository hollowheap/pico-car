/** @file encoder_pio_wait_test.c
 * @brief Host tests for the bounded wait on the encoder RX FIFO.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drv_encoder.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

typedef struct
{
    uint32_t polls;
    uint32_t word_at; /* poll number that sees a word; 0 means never */
} fake_fifo_t;

static bool fake_is_empty (void * p_ctx)
{
    fake_fifo_t * p_fifo = (fake_fifo_t *)p_ctx;

    p_fifo->polls++;
    return (p_fifo->word_at == 0U) || (p_fifo->polls < p_fifo->word_at);
}

static void test_stalled_fifo_expires (void)
{
    fake_fifo_t fifo = {0U, 0U};

    assert(!encoder_pio_wait(fake_is_empty, &fifo, ENCODER_PIO_WAIT_SPINS));
    assert(ENCODER_PIO_WAIT_SPINS == fifo.polls);
}

static void test_word_in_time_is_returned (void)
{
    fake_fifo_t fifo = {0U, ENCODER_PIO_WAIT_SPINS};

    assert(encoder_pio_wait(fake_is_empty, &fifo, ENCODER_PIO_WAIT_SPINS));
    assert(ENCODER_PIO_WAIT_SPINS == fifo.polls);
}

static void test_word_too_late_expires (void)
{
    fake_fifo_t fifo = {0U, ENCODER_PIO_WAIT_SPINS + 1U};

    assert(!encoder_pio_wait(fake_is_empty, &fifo, ENCODER_PIO_WAIT_SPINS));
}

int main (void)
{
    test_stalled_fifo_expires();
    test_word_in_time_is_returned();
    test_word_too_late_expires();
    puts("test_encoder_pio_wait: 3 tests passed");
    return 0;
}
