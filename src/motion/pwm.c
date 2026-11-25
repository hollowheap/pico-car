/** @file pwm.c
 * @brief Break before drive; force low GPIOs before stopping PWM counters.
 */
#include "pwm.h"
#include <bsp/libbsp.h>
#include "motion_pins.h"
#include "motor.h"
#include "platform/static_assert.h"

#if TMCLK_MHz != 125
#error "Revalidate motor PWM frequency after changing the system clock"
#endif
/* The slice runs at clk_sys with divider 1 and TOP = period - 1. */
STATIC_ASSERT (((uint32_t)TMCLK_MHz * 1000000U) / MOTOR_PAIR_PERIOD == 10000U,
               "motor PWM is 10 kHz");
#if (MOTOR_LEFT_A != 8U) || (MOTOR_LEFT_B != 9U) || (MOTOR_RIGHT_A != 10U) ||  \
    (MOTOR_RIGHT_B != 11U)
#error "Revalidate reserved PWM slices after changing the motor pins"
#endif

/* PWM slices driven by the two wheels (GP8/9 on slice 4, GP10/11 on 5). */
#define MOTOR_SLICE_FIRST 4U
#define MOTOR_SLICE_LAST  5U

#define MOTOR_PWM_PERIOD        MOTOR_PAIR_PERIOD
#define MOTOR_PWM_MASK \
    ((1U << MOTOR_SLICE_FIRST) | (1U << MOTOR_SLICE_LAST))
#define MOTOR_SLICE_BASE(slice) (PWM_BASE + ((slice) * 0x14U))

/* Bridge inputs per wheel (A side and B side). */
#define MOTOR_CHANNELS 2U

/* Spins allowed for the PWM block to leave reset. */
#define RESET_WAIT_SPINS 10000U

/* Unsigned copies of the kernel's signed RESETS_RESET_PWM and PWM_CH_CSR_EN
 * constants, which BARR-C 5.3.b forbids in bitwise expressions. */
#define RESET_PWM_BIT  ((UW)RESETS_RESET_PWM)
#define PWM_CSR_ENABLE 1U

/* CH_DIV value for an integer divider of 1 (free-running clk_sys). */
#define PWM_DIV_INT_ONE 0x10U

/* Offset of the RP2040 atomic SET alias from a register. */
#define PWM_ATOMIC_SET 0x2000U

/* GPIO number to PWM slice: two GPIOs per slice, eight slices. */
#define PWM_PIN_SLICE_SHIFT 1U
#define PWM_SLICE_MASK      7U

static const UINT g_motor_pins[MOTOR_PAIR_WHEELS][MOTOR_CHANNELS] = {
    {MOTOR_LEFT_A, MOTOR_LEFT_B},
    {MOTOR_RIGHT_A, MOTOR_RIGHT_B}};
static ID         g_owner;
static bool       g_active;
static bool       g_stopped;
static bool       g_pair_active;
static int16_t    g_previous_duty[MOTOR_PAIR_WHEELS];

/* SIO pin mask of both bridge inputs of one wheel. */
#define WHEEL_PIN_MASK(wheel)                                      \
    ((1U << g_motor_pins[(wheel)][0U]) | (1U << g_motor_pins[(wheel)][1U]))

/* Drive the pins in mask low as SIO outputs. The level is cleared and the
 * output enabled with one write each, before the pins leave the PWM function,
 * so no pin of the mask is ever high or released on its own. */
static void force_low (uint32_t mask)
{
    unsigned int wheel   = 0U;
    unsigned int channel = 0U;

    out_w(GPIO_OUT_CLR, mask);
    out_w(GPIO_OE_SET, mask);
    for (wheel = 0U; wheel < MOTOR_PAIR_WHEELS; wheel++)
    {
        for (channel = 0U; channel < MOTOR_CHANNELS; channel++)
        {
            UINT const pin = g_motor_pins[wheel][channel];

            if ((mask & (1U << pin)) != 0U)
            {
                out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
            }
        }
    }
}

void drv_pwm_boot_brake (void)
{
    force_low(WHEEL_PIN_MASK(0U) | WHEEL_PIN_MASK(1U));
}

static ER owner_context (void)
{
    if (tk_get_prc() != TP_PRC1)
    {
        return E_CTX;
    }
    if (g_owner <= 0)
    {
        return E_OBJ;
    }
    return tk_get_tid() == g_owner ? E_OK : E_CTX;
}

ER drv_pwm_init (void)
{
    unsigned int slice = 0U;
    unsigned int wait  = 0U;

    if ((tk_get_prc() != TP_PRC1) || (tk_get_tid() <= 0))
    {
        return E_CTX;
    }
    if (g_owner > 0)
    {
        return E_OBJ;
    }
    /* If the block was kept in reset, release it without resetting other
     * slices or changing the shared PWM wrap IRQ handler. */
    clr_w(RESETS_RESET, RESET_PWM_BIT);
    for (wait = 0U; wait < RESET_WAIT_SPINS; wait++)
    {
        if ((in_w(RESETS_RESET_DONE) & RESET_PWM_BIT) != 0U)
        {
            break;
        }
    }
    if (wait == RESET_WAIT_SPINS)
    {
        return E_TMOUT;
    }
    if (((in_w(PWM_EN) | in_w(PWM_INTE) | in_w(PWM_INTF)) & MOTOR_PWM_MASK) !=
        0U)
    {
        return E_BUSY;
    }
    drv_pwm_boot_brake();
    for (slice = MOTOR_SLICE_FIRST; slice <= MOTOR_SLICE_LAST; slice++)
    {
        uint32_t base = MOTOR_SLICE_BASE(slice);
        out_w(base + PWM_CHx_CSR, 0U);
        /* Free-running clk_sys / 1. */
        out_w(base + PWM_CHx_DIV, PWM_DIV_INT_ONE);
        out_w(base + PWM_CHx_CTR, 0U);
        out_w(base + PWM_CHx_CC, 0U);
        out_w(base + PWM_CHx_TOP, MOTOR_PWM_PERIOD - 1U);
    }
    g_owner            = tk_get_tid();
    g_active           = false;
    g_stopped          = false;
    g_pair_active      = false;
    g_previous_duty[0] = 0;
    g_previous_duty[1] = 0;
    return E_OK;
}

ER drv_pwm_brake (void)
{
    unsigned int slice  = 0U;
    ER           result = owner_context();

    if (result != E_OK)
    {
        return result;
    }
    /* Compare writes are buffered and disabling PWM can freeze a HIGH
     * output. First switch the pins to known-low SIO outputs instead. */
    drv_pwm_boot_brake();
    for (slice = MOTOR_SLICE_FIRST; slice <= MOTOR_SLICE_LAST; slice++)
    {
        uint32_t base = MOTOR_SLICE_BASE(slice);
        out_w(base + PWM_CHx_CSR, 0U);
        out_w(base + PWM_CHx_CC, 0U);
        out_w(base + PWM_CHx_CTR, 0U);
    }
    g_active           = false;
    g_pair_active      = false;
    g_previous_duty[0] = 0;
    g_previous_duty[1] = 0;
    return E_OK;
}

ER drv_pwm_set_pair (int16_t left, int16_t right)
{
    motor_plan_t plan;
    unsigned int wheel   = 0U;
    unsigned int channel = 0U;
    ER           result  = owner_context();

    if (result != E_OK)
    {
        return result;
    }
    if (g_stopped)
    {
        return E_OBJ;
    }
    if (!motor_plan(g_previous_duty, left, right, &plan))
    {
        return E_PAR;
    }
    if ((left == 0) && (right == 0))
    {
        return drv_pwm_brake();
    }
    if (g_active && !g_pair_active)
    {
        return E_BUSY;
    }
    if (plan.reversal)
    {
        (void)drv_pwm_brake();
        return E_BUSY;
    }
    for (wheel = 0U; wheel < MOTOR_PAIR_WHEELS; wheel++)
    {
        uint32_t base = MOTOR_SLICE_BASE(wheel + MOTOR_SLICE_FIRST);

        if (plan.duty[wheel] == 0)
        {
            /* Force a zero-command wheel low before its buffered CC update. */
            force_low(WHEEL_PIN_MASK(wheel));
        }
        if ((plan.duty[wheel] == 0) || (g_previous_duty[wheel] == 0))
        {
            /* A newly started slice is configured while disabled; a stopped
             * wheel cannot retain a buffered compare from its last drive. */
            out_w(base + PWM_CHx_CSR, 0U);
            out_w(base + PWM_CHx_CTR, 0U);
        }
        out_w(base + PWM_CHx_CC, plan.compare[wheel]);
    }
    for (wheel = 0U; wheel < MOTOR_PAIR_WHEELS; wheel++)
    {
        if (plan.duty[wheel] != 0)
        {
            for (channel = 0U; channel < MOTOR_CHANNELS; channel++)
            {
                (void)pwm_set_pin(g_motor_pins[wheel][channel]);
            }
        }
        g_previous_duty[wheel] = plan.duty[wheel];
    }
    /* RP2040 atomic SET alias: preserve other buddies' PWM slice enables. */
    out_w(PWM_EN + PWM_ATOMIC_SET,
          (plan.duty[0] != 0 ? 1U << MOTOR_SLICE_FIRST : 0U) |
              (plan.duty[1] != 0 ? 1U << MOTOR_SLICE_LAST : 0U));
    g_pair_active = true;
    g_active      = true;
    return E_OK;
}

ER drv_pwm_drive (unsigned int wheel, bool channel_a, uint16_t duty_permille)
{
    UINT     pin     = 0U;
    uint32_t compare = 0U;
    uint32_t base    = 0U;
    ER       result  = owner_context();

    if (result != E_OK)
    {
        return result;
    }
    if ((wheel >= MOTOR_PAIR_WHEELS) ||
        (duty_permille > MOTOR_DUTY_MAX_PERMILLE))
    {
        return E_PAR;
    }
    if (g_stopped)
    {
        return E_OBJ;
    }
    if (duty_permille == 0U)
    {
        return drv_pwm_brake();
    }
    if (g_active)
    {
        return E_BUSY;
    }
    pin     = g_motor_pins[wheel][channel_a ? 0U : 1U];
    base    = MOTOR_SLICE_BASE((pin >> PWM_PIN_SLICE_SHIFT) & PWM_SLICE_MASK);
    compare = (MOTOR_PWM_PERIOD * (uint32_t)duty_permille) /
              MOTOR_DUTY_MAX_PERMILLE;
    out_w(base + PWM_CHx_CC,
          (pin & 1U) != 0U ? compare << MOTOR_COMPARE_B_SHIFT : compare);
    (void)pwm_set_pin(pin);
    out_w(base + PWM_CHx_CSR, PWM_CSR_ENABLE);
    g_active = true;
    return E_OK;
}

ER drv_pwm_latch_stop (void)
{
    ER result = drv_pwm_brake();

    if (result == E_OK)
    {
        g_stopped = true;
    }
    return result;
}
