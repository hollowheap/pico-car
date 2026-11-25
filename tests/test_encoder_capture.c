#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <bsp/libbsp.h>

static bool g_irq_masked;
static bool g_delay_right_word;
static bool g_unavailable_right_word;
static unsigned g_masked_waits;

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
    return address == TIMER_TIMERAWL ? 100U : 0U;
}

void shim_reg_write(uint32_t address, uint32_t value)
{
    (void)address;
    (void)value;
}

bool encoder_pio_start(unsigned int const pins[ENCODER_WHEELS])
{
    (void)pins;
    return true;
}

bool encoder_pio_read(uint32_t words[ENCODER_WHEELS])
{
    if (!g_delay_right_word)
    {
        words[ENCODER_LEFT] = words[ENCODER_RIGHT] = 0U;
        return true;
    }
    words[ENCODER_LEFT] = 2U;
    if (g_irq_masked)
    {
        ++g_masked_waits;
    }
    if (g_unavailable_right_word)
    {
        return false;
    }
    words[ENCODER_RIGHT] = 3U;
    return true;
}

int main(void)
{
    assert(E_OK == encoder_init());
    g_delay_right_word = true;
    g_unavailable_right_word = true;
    encoder_sample_t sample = {.timestamp_us = 77U};

    assert(E_IO == encoder_read(&sample));
    assert(sample.timestamp_us == 77U && sample.wheel[ENCODER_LEFT].positive == 0U);
    g_unavailable_right_word = false;
    assert(E_OK == encoder_read(&sample));
    assert(sample.timestamp_us == 100U);
    assert(sample.wheel[ENCODER_LEFT].positive == 2U);
    assert(sample.wheel[ENCODER_RIGHT].positive == 3U);
    assert(g_masked_waits == 0U && "a delayed PIO word must not hold GPIO interrupts masked");
    puts("test_encoder_capture: coherent pair, wait outside IRQ mask");
    return 0;
}
