/** @file pwm.c
 * @brief Break before drive; force low GPIOs before stopping PWM counters.
 */
#include "pwm.h"
#include <bsp/libbsp.h>
#include "motion_pins.h"
#include "bridge.h"

#if TMCLK_MHz != 125
#error "Revalidate motor PWM frequency after changing the system clock"
#endif
#if (MOTOR_LEFT_A != 8U) || (MOTOR_LEFT_B != 9U) || (MOTOR_RIGHT_A != 10U) ||  \
    (MOTOR_RIGHT_B != 11U)
#error "Revalidate reserved PWM slices after changing the motor pins"
#endif

#define MOTOR_PWM_PERIOD        BRIDGE_PERIOD
#define MOTOR_PWM_MASK          ((1U << 4U) | (1U << 5U))
#define MOTOR_SLICE_BASE(slice) (PWM_BASE + ((slice) * 0x14U))

static const UINT motor_pins[2][2] = {{MOTOR_LEFT_A, MOTOR_LEFT_B},
                                      {MOTOR_RIGHT_A, MOTOR_RIGHT_B}};
static ID         owner;
static bool       active;
static volatile bool stopped;
static int16_t    previous_duty[2];

void pwm_boot_brake (void)
{
    unsigned int wheel;
    unsigned int channel;

    for (wheel = 0U; wheel < 2U; wheel++)
    {
        for (channel = 0U; channel < 2U; channel++)
        {
            (void)gpio_set_val(motor_pins[wheel][channel], 0U);
            (void)gpio_set_pin(motor_pins[wheel][channel], GPIO_MODE_OUT);
        }
    }
}

static ER owner_context (void)
{
    if (tk_get_prc() != TP_PRC1)
    {
        return E_CTX;
    }
    if (owner <= 0)
    {
        return E_OBJ;
    }
    return tk_get_tid() == owner ? E_OK : E_CTX;
}

ER pwm_init (void)
{
    unsigned int slice;
    unsigned int wait;

    if ((tk_get_prc() != TP_PRC1) || (tk_get_tid() <= 0))
    {
        return E_CTX;
    }
    if (owner > 0)
    {
        return E_OBJ;
    }
    /* If the block was kept in reset, release it without resetting other
     * slices or changing the shared PWM wrap IRQ handler. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    for (wait = 0U; wait < 10000U; wait++)
    {
        if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) != 0U)
        {
            break;
        }
    }
    if (wait == 10000U)
    {
        return E_TMOUT;
    }
    if (((in_w(PWM_EN) | in_w(PWM_INTE) | in_w(PWM_INTF)) & MOTOR_PWM_MASK) !=
        0U)
    {
        return E_BUSY;
    }
    pwm_boot_brake();
    for (slice = 4U; slice <= 5U; slice++)
    {
        uint32_t base = MOTOR_SLICE_BASE(slice);
        out_w(base + PWM_CHx_CSR, 0U);
        out_w(base + PWM_CHx_DIV, 0x10U); /* Free-running clk_sys / 1. */
        out_w(base + PWM_CHx_CTR, 0U);
        out_w(base + PWM_CHx_CC, 0U);
        out_w(base + PWM_CHx_TOP, MOTOR_PWM_PERIOD - 1U);
    }
    owner            = tk_get_tid();
    active           = false;
    stopped          = false;
    previous_duty[0] = 0;
    previous_duty[1] = 0;
    return E_OK;
}

ER pwm_brake (void)
{
    unsigned int slice;
    ER           result = owner_context();

    if (result != E_OK)
    {
        return result;
    }
    /* Compare writes are buffered and disabling PWM can freeze a HIGH
     * output. First switch the pins to known-low SIO outputs instead. */
    pwm_boot_brake();
    for (slice = 4U; slice <= 5U; slice++)
    {
        uint32_t base = MOTOR_SLICE_BASE(slice);
        out_w(base + PWM_CHx_CSR, 0U);
        out_w(base + PWM_CHx_CC, 0U);
        out_w(base + PWM_CHx_CTR, 0U);
    }
    active           = false;
    previous_duty[0] = 0;
    previous_duty[1] = 0;
    return E_OK;
}

void pwm_emergency_stop (void)
{
    unsigned int slice;

    stopped = true;
    pwm_boot_brake();
    out_w(PWM_EN + 0x3000U, MOTOR_PWM_MASK);
    for (slice = 4U; slice <= 5U; slice++)
    {
        uint32_t base = MOTOR_SLICE_BASE(slice);
        out_w(base + PWM_CHx_CSR, 0U);
        out_w(base + PWM_CHx_CC, 0U);
        out_w(base + PWM_CHx_CTR, 0U);
    }
}

ER pwm_set_pair (int16_t left, int16_t right)
{
    bridge_plan_t plan;
    unsigned int      wheel;
    unsigned int      channel;
    ER                result = owner_context();

    if (result != E_OK)
    {
        return result;
    }
    if (stopped)
    {
        return E_OBJ;
    }
    if (!bridge_plan(previous_duty, left, right, &plan))
    {
        return E_PAR;
    }
    if ((left == 0) && (right == 0))
    {
        return pwm_brake();
    }
    if (plan.reversal)
    {
        (void)pwm_brake();
        return E_BUSY;
    }
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        uint32_t base = MOTOR_SLICE_BASE(wheel + 4U);

        if (plan.duty[wheel] == 0)
        {
            /* Force a zero-command wheel low before its buffered CC update. */
            for (channel = 0U; channel < 2U; channel++)
            {
                (void)gpio_set_val(motor_pins[wheel][channel], 0U);
                (void)gpio_set_pin(motor_pins[wheel][channel], GPIO_MODE_OUT);
            }
        }
        if ((plan.duty[wheel] == 0) || (previous_duty[wheel] == 0))
        {
            /* A newly started slice is configured while disabled; a stopped
             * wheel cannot retain a buffered compare from its last drive. */
            out_w(base + PWM_CHx_CSR, 0U);
            out_w(base + PWM_CHx_CTR, 0U);
        }
        out_w(base + PWM_CHx_CC, plan.compare[wheel]);
    }
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        if (plan.duty[wheel] != 0)
        {
            for (channel = 0U; channel < 2U; channel++)
            {
                (void)pwm_set_pin(motor_pins[wheel][channel]);
            }
        }
        previous_duty[wheel] = plan.duty[wheel];
    }
    /* RP2040 atomic SET alias: preserve other buddies' PWM slice enables. */
    if (stopped)
    {
        return E_OBJ;
    }
    out_w(PWM_EN + 0x2000U,
          (left != 0 ? 1U << 4U : 0U) | (right != 0 ? 1U << 5U : 0U));
    active = true;
    return E_OK;
}

ER pwm_latch_stop (void)
{
    ER result = pwm_brake();

    if (result == E_OK)
    {
        stopped = true;
    }
    return result;
}
