#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <bsp/libbsp.h>

static bool     g_irq_masked;
static bool     g_delay_right_word;
static bool     g_unavailable_right_word;
static bool     g_pio_start_ok = true;
static bool     g_pio_claimed;
static bool     g_mtx_used;
static bool     g_mtx_locked;
static ATR      g_mtx_atr;
static ER       g_mtx_cre_error;
static ER       g_mtx_loc_error;
static uint32_t g_timer_us = 100U;
static uint32_t g_next_left_word;
static uint32_t g_next_right_word;
static unsigned g_masked_waits;
static unsigned g_unserialized_reads;
static unsigned g_pio_read_calls;
static unsigned g_pio_read_depth;
static unsigned g_pio_read_max_depth;
static void (*gp_on_pio_read)(void);

static ER capture_lock(T_SPLOCK *lock, UINT *saved)
{
    assert(!g_irq_masked);
    g_irq_masked = true;
    *saved = 0U;
    return SpinLock(lock);
}

static ER capture_unlock(T_SPLOCK *lock, UINT saved)
{
    (void)saved;
    assert(g_irq_masked);
    g_irq_masked = false;
    return SpinUnlock(lock);
}

#define ISpinLock capture_lock
#define ISpinUnlock capture_unlock
#include "encoder.c"
#undef ISpinUnlock
#undef ISpinLock

ID tk_cre_mtx(const T_CMTX *pk_cmtx)
{
    assert(!g_irq_masked);
    assert(pk_cmtx != NULL);
    if (g_mtx_cre_error != 0)
    {
        ER injected = g_mtx_cre_error;
        g_mtx_cre_error = 0;
        return injected;
    }
    if (g_mtx_used)
    {
        return E_LIMIT;
    }
    g_mtx_used = true;
    g_mtx_locked = false;
    g_mtx_atr = pk_cmtx->mtxatr;
    return 1;
}

ER tk_del_mtx(ID mtxid)
{
    assert(!g_irq_masked);
    if ((mtxid != 1) || !g_mtx_used)
    {
        return E_NOEXS;
    }
    assert(!g_mtx_locked);
    g_mtx_used = false;
    return E_OK;
}

ER tk_loc_mtx(ID mtxid, TMO tmout)
{
    assert(!g_irq_masked);
    assert(TMO_FEVR == tmout);
    if (g_mtx_loc_error != 0)
    {
        ER injected = g_mtx_loc_error;
        g_mtx_loc_error = 0;
        return injected;
    }
    if ((mtxid != 1) || !g_mtx_used)
    {
        return E_NOEXS;
    }
    if (g_mtx_locked)
    {
        return E_ILUSE;
    }
    g_mtx_locked = true;
    return E_OK;
}

ER tk_unl_mtx(ID mtxid)
{
    assert(!g_irq_masked);
    if ((mtxid != 1) || !g_mtx_used)
    {
        return E_NOEXS;
    }
    if (!g_mtx_locked)
    {
        return E_ILUSE;
    }
    g_mtx_locked = false;
    return E_OK;
}

ER gpio_set_pin(UINT pin, UINT mode)
{
    (void)pin;
    (void)mode;
    return E_OK;
}

UW GPIO(UINT pin)
{
    return pin;
}

void clr_w(UW address, UW bits)
{
    (void)address;
    (void)bits;
}

uint32_t shim_reg_read(uint32_t address)
{
    return address == TIMER_TIMERAWL ? g_timer_us : 0U;
}

void shim_reg_write(uint32_t address, uint32_t value)
{
    (void)address;
    (void)value;
}

bool encoder_pio_start(unsigned int const pins[ENCODER_WHEELS])
{
    (void)pins;
    if (!g_pio_start_ok || g_pio_claimed)
    {
        return false;
    }
    g_pio_claimed = true;
    return true;
}

void encoder_pio_stop(void)
{
    g_pio_claimed = false;
}

bool encoder_pio_read(uint32_t words[ENCODER_WHEELS])
{
    assert(g_pio_claimed);
    ++g_pio_read_calls;
    ++g_pio_read_depth;
    if (g_pio_read_depth > g_pio_read_max_depth)
    {
        g_pio_read_max_depth = g_pio_read_depth;
    }
    if (!g_mtx_locked)
    {
        ++g_unserialized_reads;
    }
    if (g_irq_masked)
    {
        ++g_masked_waits;
    }
    if (gp_on_pio_read != NULL)
    {
        void (*hook)(void) = gp_on_pio_read;
        gp_on_pio_read = NULL;
        hook();
    }
    if (!g_delay_right_word)
    {
        words[ENCODER_LEFT] = g_next_left_word;
        words[ENCODER_RIGHT] = g_next_right_word;
        --g_pio_read_depth;
        return true;
    }
    words[ENCODER_LEFT] = g_next_left_word;
    if (g_unavailable_right_word)
    {
        --g_pio_read_depth;
        return false;
    }
    words[ENCODER_RIGHT] = g_next_right_word;
    --g_pio_read_depth;
    return true;
}

static void contend_during_pio_read(void)
{
    encoder_sample_t nested = {.timestamp_us = 55U};

    assert(g_mtx_locked);
    assert(!g_irq_masked);
    assert(E_ILUSE == encoder_read(&nested));
    assert(nested.timestamp_us == 55U);
    assert(nested.wheel[ENCODER_LEFT].positive == 0U);
    assert(E_ILUSE == encoder_reset());
    assert(g_mtx_locked);
}

static void test_init_failures_clean_up(void)
{
    g_mtx_cre_error = E_LIMIT;
    assert(E_LIMIT == encoder_init());
    assert(!g_mtx_used);
    assert(!g_pio_claimed);
    assert(!g_initialised);

    g_pio_start_ok = false;
    assert(E_LIMIT == encoder_init());
    assert(!g_mtx_used);
    assert(!g_pio_claimed);
    assert(!g_initialised);
    g_pio_start_ok = true;

    g_mtx_loc_error = E_CTX;
    assert(E_CTX == encoder_init());
    assert(!g_mtx_used);
    assert(!g_pio_claimed);
    assert(!g_initialised);

    g_delay_right_word = true;
    g_unavailable_right_word = true;
    g_next_left_word = 9U;
    assert(E_IO == encoder_init());
    assert(!g_mtx_used);
    assert(!g_pio_claimed);
    assert(!g_initialised);
    assert(g_masked_waits == 0U);
    assert(g_unserialized_reads == 0U);

    g_delay_right_word = false;
    g_unavailable_right_word = false;
    g_next_left_word = 0U;
    g_next_right_word = 0U;
}

static void test_coherent_pair_and_unmasked_wait(void)
{
    encoder_sample_t sample = {.timestamp_us = 77U};

    assert(E_OK == encoder_init());
    assert(g_mtx_used);
    assert(g_pio_claimed);
    assert(TA_INHERIT == g_mtx_atr);
    assert(!g_mtx_locked);
    assert(!g_irq_masked);

    g_delay_right_word = true;
    g_unavailable_right_word = true;
    g_next_left_word = 2U;
    g_next_right_word = 3U;

    assert(E_IO == encoder_read(&sample));
    assert(sample.timestamp_us == 77U && sample.wheel[ENCODER_LEFT].positive == 0U);
    assert(!g_mtx_locked && !g_irq_masked);

    g_unavailable_right_word = false;
    assert(E_OK == encoder_read(&sample));
    assert(sample.timestamp_us == 100U);
    assert(sample.wheel[ENCODER_LEFT].positive == 2U);
    assert(sample.wheel[ENCODER_RIGHT].positive == 3U);
    assert(g_masked_waits == 0U && "a delayed PIO word must not hold GPIO interrupts masked");
    assert(g_unserialized_reads == 0U);

    /* A subsequent partial failure must preserve the previous valid pair. */
    g_unavailable_right_word = true;
    g_next_left_word = 5U;
    g_timer_us = 200U;
    assert(E_IO == encoder_read(&sample));
    assert(sample.timestamp_us == 100U);
    assert(sample.wheel[ENCODER_LEFT].positive == 2U);
    assert(sample.wheel[ENCODER_RIGHT].positive == 3U);

    g_unavailable_right_word = false;
    g_next_right_word = 7U;
    g_timer_us = 250U;
    assert(E_OK == encoder_read(&sample));
    assert(sample.timestamp_us == 250U);
    assert(sample.wheel[ENCODER_LEFT].positive == 5U);
    assert(sample.wheel[ENCODER_RIGHT].positive == 7U);
}

static void test_serialization_and_rebase_coherence(void)
{
    encoder_sample_t sample = {.timestamp_us = 0U};
    unsigned reads_before = 0U;

    /* Contending read/reset during an active PIO read must not interleave. */
    g_next_left_word = 8U;
    g_next_right_word = 11U;
    g_timer_us = 300U;
    gp_on_pio_read = contend_during_pio_read;
    assert(E_OK == encoder_read(&sample));
    assert(sample.timestamp_us == 300U);
    assert(sample.wheel[ENCODER_LEFT].positive == 8U);
    assert(sample.wheel[ENCODER_RIGHT].positive == 11U);
    assert(g_pio_read_max_depth == 1U);
    assert(!g_mtx_locked && !g_irq_masked);

    /* Non-task context failure propagates without touching PIO or unlocking. */
    reads_before = g_pio_read_calls;
    g_mtx_loc_error = E_CTX;
    assert(E_CTX == encoder_read(&sample));
    assert(g_pio_read_calls == reads_before);
    assert(sample.timestamp_us == 300U);
    assert(!g_mtx_locked && !g_irq_masked);

    /* Failed rebase preserves previous baseline and counts cleanly. */
    g_unavailable_right_word = true;
    g_next_left_word = 12U;
    assert(E_IO == encoder_reset());
    assert(!g_mtx_locked && !g_irq_masked);

    g_unavailable_right_word = false;
    g_next_right_word = 15U;
    g_timer_us = 350U;
    assert(E_OK == encoder_read(&sample));
    assert(sample.wheel[ENCODER_LEFT].positive == 12U);
    assert(sample.wheel[ENCODER_RIGHT].positive == 15U);

    /* Valid rebase resets counts from fresh words outside IRQ mask. */
    g_next_left_word = 20U;
    g_next_right_word = 30U;
    assert(E_OK == encoder_reset());
    assert(!g_mtx_locked && !g_irq_masked);

    g_next_left_word = 23U;
    g_next_right_word = 35U;
    g_timer_us = 400U;
    assert(E_OK == encoder_read(&sample));
    assert(sample.timestamp_us == 400U);
    assert(sample.wheel[ENCODER_LEFT].positive == 3U);
    assert(sample.wheel[ENCODER_RIGHT].positive == 5U);
    assert(g_masked_waits == 0U);
    assert(g_unserialized_reads == 0U);
}

int main(void)
{
    test_init_failures_clean_up();
    test_coherent_pair_and_unmasked_wait();
    test_serialization_and_rebase_coherence();
    puts("test_encoder_capture: coherent pair, wait outside IRQ mask");
    return 0;
}
