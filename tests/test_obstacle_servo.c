/** Exercise the actual servo initialization and PWM compare conversion. */
#include <assert.h>
#include <bsp/libbsp.h>

#include "platform/isr.h"
#include "scan_hw.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

INT g_shim_prc = TP_PRC1;
static UW g_now;
static UW g_compare;
static UW g_compare_at_enable;

uint32_t shim_reg_read(uint32_t address)
{
    if (TIMER_TIMERAWL == address)
    {
        return g_now;
    }
    return (RESETS_RESET_DONE == address) ? RESETS_RESET_PWM : 0U;
}

void shim_reg_write(uint32_t address, uint32_t value)
{
    (void)address;
    (void)value;
}

void WaitUsec(UW usec)
{
    g_now += usec;
}

void clr_w(UW address, UW bits)
{
    shim_reg_write(address, shim_reg_read(address) & ~bits);
}

ER gpio_set_val(UINT pin, UINT value)
{
    assert(0U == pin && value <= 1U);
    return E_OK;
}

ER gpio_set_pin(UINT pin, UINT mode)
{
    assert((0U == pin && GPIO_MODE_OUT == mode) || (1U == pin && GPIO_MODE_IN == mode));
    return E_OK;
}

UINT gpio_get_val(UINT pin)
{
    assert(0U == pin);
    return 1U;
}

ER pwm_set_cc(UINT pin, UW compare)
{
    assert(12U == pin);
    g_compare = compare;
    return E_OK;
}

ER pwm_set_wrap(UINT pin, UW wrap)
{
    assert(12U == pin && 19999U == wrap);
    return E_OK;
}

ER pwm_set_pin(UINT pin)
{
    assert(12U == pin);
    return E_OK;
}

ER pwm_set_enabled(UINT pin, BOOL enabled)
{
    assert(12U == pin && TRUE == enabled);
    g_compare_at_enable = g_compare;
    return E_OK;
}

ER gpio_irq_add_handler(UW proc, uint32_t mask, gpio_irq_handler_t handler)
{
    assert(TP_PRC1 == proc && (1U << 1U) == mask && handler != 0);
    return E_OK;
}

ER tk_set_flg(ID flag, UINT mask)
{
    (void)flag;
    (void)mask;
    return E_OK;
}

int main(void)
{
    assert(E_OK == scan_hw_init(1));
    assert(1494U == g_compare_at_enable);

    scan_hw_servo_set(0);
    assert(g_compare_at_enable == g_compare);
    scan_hw_servo_set(201);
    assert(1382U == g_compare);
    scan_hw_servo_set(-201);
    assert(1606U == g_compare);
    return 0;
}
