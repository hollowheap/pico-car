/** @file drv_encoder.c
 * @brief Start and read the quadrature state machines. See encoder_pio.h.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/pio.h"
#include "quadrature.pio.h"

#include "drv_encoder.h"

/* 125 MHz / 16: one sample about every 1.3 us. An edge at full speed is more
 * than 100 us from the next, and a short spike on one pin counts one step
 * each way, so it cancels. */
#define ENCODER_PIO_CLKDIV (16U)
#define ENCODER_PIO_PINS   (2U)
#define ENCODER_PIO_SHIFT  (32U)

static PIO  gp_pio = NULL;
static uint g_offset = 0U;
static uint g_sm[ENCODER_WHEELS];

static void start_sm (uint sm, uint offset, uint pin_a)
{
    pio_sm_config config = quadrature_program_get_default_config(offset);

    sm_config_set_in_pins(&config, pin_a);
    sm_config_set_in_shift(&config, false, false, ENCODER_PIO_SHIFT);
    sm_config_set_out_shift(&config, true, false, ENCODER_PIO_SHIFT);
    sm_config_set_clkdiv_int_frac(&config, (uint16_t)ENCODER_PIO_CLKDIV, 0U);
    (void)pio_sm_init(gp_pio, sm, offset + (uint)quadrature_wrap_target, &config);
    /* The last state is the present state, so the start counts no step. */
    pio_sm_exec(gp_pio, sm, pio_encode_in(pio_pins, ENCODER_PIO_PINS));
    pio_sm_exec(gp_pio, sm, pio_encode_mov(pio_osr, pio_isr));
    pio_sm_exec(gp_pio, sm, pio_encode_set(pio_x, 0U));
    pio_sm_exec(gp_pio, sm, pio_encode_set(pio_y, 0U));
}

bool encoder_pio_start (const unsigned int pins_a[ENCODER_WHEELS])
{
    PIO  pio    = NULL;
    uint offset = 0U;
    uint first  = 0U;
    int  second = -1;
    bool b_ok   = false;

    if (gp_pio != NULL)
    {
        return false;
    }

    b_ok = pio_claim_free_sm_and_add_program(&quadrature_program, &pio,
                                             &first, &offset);
    if (b_ok)
    {
        second = pio_claim_unused_sm(pio, false);
        if (second < 0)
        {
            pio_remove_program_and_unclaim_sm(&quadrature_program, pio, first,
                                              offset);
            b_ok = false;
        }
    }
    if (b_ok)
    {
        gp_pio              = pio;
        g_offset            = offset;
        g_sm[ENCODER_LEFT]  = first;
        g_sm[ENCODER_RIGHT] = (uint)second;
        start_sm(g_sm[ENCODER_LEFT], g_offset, pins_a[ENCODER_LEFT]);
        start_sm(g_sm[ENCODER_RIGHT], g_offset, pins_a[ENCODER_RIGHT]);
        pio_set_sm_mask_enabled(gp_pio,
                                (1U << g_sm[ENCODER_LEFT]) |
                                    (1U << g_sm[ENCODER_RIGHT]),
                                true);
    }

    return b_ok;
}

void encoder_pio_stop (void)
{
    if (gp_pio != NULL)
    {
        pio_set_sm_mask_enabled(gp_pio,
                                (1U << g_sm[ENCODER_LEFT]) |
                                    (1U << g_sm[ENCODER_RIGHT]),
                                false);
        pio_sm_unclaim(gp_pio, g_sm[ENCODER_RIGHT]);
        pio_remove_program_and_unclaim_sm(&quadrature_program, gp_pio,
                                          g_sm[ENCODER_LEFT], g_offset);
        gp_pio   = NULL;
        g_offset = 0U;
    }
}

static bool rx_fifo_empty (void * p_ctx)
{
    return pio_sm_is_rx_fifo_empty(gp_pio, *(const uint *)p_ctx);
}

bool encoder_pio_read (uint32_t words[ENCODER_WHEELS])
{
    uint     wheel                 = 0U;
    uint32_t fresh[ENCODER_WHEELS] = {0U};

    if ((words == NULL) || (gp_pio == NULL))
    {
        return false;
    }

    /* A full FIFO drops new words and keeps old ones. Drain both FIFOs first,
     * so the two waits for a fresh word overlap. */
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        while (!pio_sm_is_rx_fifo_empty(gp_pio, g_sm[wheel]))
        {
            (void)pio_sm_get(gp_pio, g_sm[wheel]);
        }
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if (!encoder_pio_wait(rx_fifo_empty, &g_sm[wheel], ENCODER_PIO_WAIT_SPINS))
        {
            return false;
        }
        fresh[wheel] = pio_sm_get(gp_pio, g_sm[wheel]);
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        words[wheel] = fresh[wheel];
    }
    return true;
}

/*** end of file ***/
