/** Exercise the actual servo initialization and PWM compare conversion. */
#include <assert.h>
#include <bsp/libbsp.h>
#include <string.h>

#include "platform/isr.h"
#include "obstacle_pack.h"
#include "scan_hw.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

INT g_shim_prc = TP_PRC1;
static UW g_now;
static UW g_compare;
static UW g_compare_at_enable;
static UW g_trigger_started_us;
static UINT g_trigger_output;
static UINT g_trigger_pad = 1U;
static gpio_irq_handler_t g_irq;

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
    g_trigger_output = value;
    if (value == 1U)
    {
        g_trigger_started_us = g_now;
    }
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
    assert(g_trigger_output == 1U);
    assert(g_now - g_trigger_started_us == 12U);
    return g_trigger_pad;
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
    g_irq = handler;
    return E_OK;
}

ER tk_set_flg(ID flag, UINT mask)
{
    (void)flag;
    (void)mask;
    return E_OK;
}

static uint32_t captured_echo(uint32_t trigger_us, uint32_t rise_us, uint32_t fall_us)
{
    assert(!scan_hw_rearm());
    g_irq(&(gpio_edge_snapshot_t){.timestamp_us = rise_us, .rise_mask = 1U << 1U});
    g_irq(&(gpio_edge_snapshot_t){.timestamp_us = fall_us, .fall_mask = 1U << 1U});
    return scan_hw_echo_result(trigger_us);
}

int main(int argc, char * argv[])
{
    if (argc == 2 && strcmp(argv[1], "trigger-readback") == 0)
    {
        assert(scan_hw_trigger());
        assert(g_trigger_output == 0U);
        g_trigger_pad = 0U;
        assert(!scan_hw_trigger());
        assert(g_trigger_output == 0U);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "qualified-pwm-endpoints") == 0)
    {
        assert(E_OK == scan_hw_init(1));
        scan_hw_servo_set(310);
        assert(1326U == g_compare && "positive overtravel must clamp at 30 degrees");
        scan_hw_servo_set(0);
        assert(1494U == g_compare);
        scan_hw_servo_set(-310);
        assert(1662U == g_compare && "negative overtravel must clamp at 30 degrees");
        scan_hw_servo_set(0);
        assert(1494U == g_compare);
        return 0;
    }
    assert(argc == 1);
    assert(E_OK == scan_hw_init(1));
    assert(1494U == g_compare_at_enable);

    scan_hw_servo_set(0);
    assert(g_compare_at_enable == g_compare);
    scan_hw_servo_set(201);
    assert(1382U == g_compare);
    scan_hw_servo_set(0);
    assert(1494U == g_compare);
    scan_hw_servo_set(-201);
    assert(1606U == g_compare);
    scan_hw_servo_set(0);
    assert(1494U == g_compare);
    /* The echo slot spans timer wrap. A delayed task must still use the
     * actual edge times to decide which shot the pulse belongs to. */
    uint32_t const trigger = UINT32_MAX - 20000U;
    assert(captured_echo(trigger, trigger + 39000U, trigger + 43000U) ==
           (OBSTACLE_ECHO_RISE | OBSTACLE_ECHO_DONE | 4000U));
    assert(captured_echo(trigger, trigger - 1U, trigger + 1000U) == OBSTACLE_ECHO_DONE);
    assert(captured_echo(trigger, trigger + 41000U, trigger + 43000U) == OBSTACLE_ECHO_DONE);
    assert(captured_echo(trigger, trigger + 1000U, trigger + 60001U) == OBSTACLE_ECHO_DONE);
    return 0;
}
