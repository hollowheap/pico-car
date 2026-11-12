/** @file scan_hw.c
 *
 * @brief Buddy 5 hardware layer: sensor pins, servo PWM, TIMER and the
 *        ECHO edge ISR.
 *
 * Port facts this file respects:
 *   - hw_setting.c puts GP0/GP1 on UART0 at boot; gpio_set_pin() moves
 *     them back to SIO, so hw_setting.c is not edited.
 *   - USE_PTMR = 0 can leave PWM in reset. This file releases the reset if
 *     it is held and never asserts one, because another buddy may already
 *     be using PWM.
 *   - ECHO edges come through the shared IO_IRQ_BANK0 dispatcher
 *     (src/sys/gpio_irq) registered for processor 1 (core 0). The ISR
 *     reads the pin level, so it works whatever the dispatcher acks.
 *   - set_w() and clr_w() are used on peripheral registers only.
 *   - The kernel (hw_setting.c, cpu_clock.c) neither releases the TIMER
 *     from reset nor starts the watchdog tick that makes it count in
 *     microseconds. After drag-and-drop flashing the boot ROM has done both,
 *     but after a cold power-on the TIMER is frozen. This file releases the
 *     TIMER reset if it is held and starts the tick if it is not set up,
 *     never asserting a reset or changing a tick someone else set up.
 *
 * NOTE: The kernel and BSP types ID, ER, UINT, UW and BOOL are used where
 * their API requires them (deviation from BARR-C 5.2.a, see
 * docs/obstacle.md).
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "platform/isr.h"
#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "obstacle_internal.h"
#include "scan_hw.h"

/* GPIO pad control bits. NOTE: These are unsigned copies of the port's
 * GPIO_PUE and GPIO_PDE, which are signed ints (BARR-C 5.3.b). */
#define PAD_PULL_UP   (1UL << 3U)
#define PAD_PULL_DOWN (1UL << 2U)
#define PAD_INPUT_ENABLE (1UL << 6U)

/* Cast is safe: RESETS_RESET_PWM is a positive single-bit constant. */
#define RESET_PWM_BIT ((UW)RESETS_RESET_PWM)

/* Enable bit of a PWM slice's CSR register (PWM_CH_CSR_EN). */
#define PWM_CSR_ENABLE (1UL)

/* ECHO pin bit in GPIO_IN and in the dispatcher mask. */
#define ECHO_BIT (1UL << OBSTACLE_ECHO_PIN)

/* Processor that owns the ISR and the NVIC entry (RP2040 core 0). */
#define PROCESSOR_1 (1)

/* Upper bound on the wait for a reset to finish. */
#define RESET_SPINS (10000U)

/* Cast is safe: RESETS_RESET_TIMER is a positive single-bit constant. */
#define RESET_TIMER_BIT ((UW)RESETS_RESET_TIMER)

/* Watchdog tick generator (RP2040 datasheet 4.7): CYCLES in bits 0-8 and
 * ENABLE in bit 9. clk_ref is the 12 MHz crystal (cpu_clock.c), so 12
 * cycles give the TIMER its 1 us tick. */
#define WATCHDOG_TICK    (0x40058000UL + 0x2CUL)
#define TICK_CYCLES_MASK (0x1FFUL)
#define TICK_ENABLE      (1UL << 9U)
#define TICK_CYCLES_1US  ((UW)XOSC_MHz)

/* deci-degrees * us-per-10-degrees / 100 gives microseconds. */
#define SERVO_SCALE_DIV (100)

/* PWM register block layout: one slice every 0x14 bytes, 8 slices. */
#define PWM_SLICE_STRIDE (0x14U)
#define PWM_SLICE_MASK   (0x7U)
#define SERVO_SLICE_BASE (PWM_BASE + (OBSTACLE_PWM_SLICE * PWM_SLICE_STRIDE))

/* Signed copies of the servo numbers. Casts are safe: all are below
 * 2401. */
#define SERVO_CENTRE_US    ((int32_t)OBSTACLE_SERVO_CENTRE_US)
#define SERVO_US_PER_10DEG ((int32_t)OBSTACLE_SERVO_US_PER_10DEG)
#define SERVO_MIN_US       ((int32_t)OBSTACLE_SERVO_MIN_US)
#define SERVO_MAX_US       ((int32_t)OBSTACLE_SERVO_MAX_US)

/* The slice must match the pin: GP12 is slice 6, channel A (even pin). */
OBSTACLE_CT_ASSERT(servo_slice_matches_pin,
             OBSTACLE_PWM_SLICE == ((OBSTACLE_SERVO_PIN >> 1U) & PWM_SLICE_MASK));
OBSTACLE_CT_ASSERT(servo_on_channel_a, 0U == (OBSTACLE_SERVO_PIN & 1U));

/* Echo state shared with the ISR, so every member is volatile: the
 * echo result (width 16 | RISE | DONE), the rise time and the flag to set.
 * NOTE: The handle is an h_ member of a g_ struct, which satisfies both
 * BARR-C 7.1.n/7.1.o and the team .clang-tidy. */
typedef struct
{
    uint32_t volatile result;
    uint32_t volatile rise_us;
    ID volatile h_flag;
} echo_state_t;

static echo_state_t g_echo = {0U, 0U, 0};

static void start_timer (void);
static ER   check_timer (void);
static bool is_pwm_out_of_reset (void);
static ER   servo_pwm_init (void);
static void echo_rise (uint32_t echo_result, uint32_t now_us);
static void echo_fall (uint32_t echo_result, uint32_t now_us);
static void echo_isr (const gpio_edge_snapshot_t * p_snapshot);

/*!
 * @brief Set up the sensor pins, the servo PWM and the ECHO ISR.
 *
 * Run on processor 1.
 *
 * @param[in] h_echo_flag Flag that the ISR sets with OBSTACLE_FLG_ECHO.
 *
 * @return E_OK; E_CTX on the wrong core; E_SYS if the TIMER is not
 *         ticking or PWM stays in reset; E_OBJ if slice 6 is in use.
 */
ER scan_hw_init (ID h_echo_flag)
{
    ER result = E_OK;

#if TK_SUPPORT_SMP
    if (PROCESSOR_1 != tk_get_prc())
    {
        result = E_CTX;
    }
#endif

    g_echo.h_flag = h_echo_flag;
    if (E_OK == result)
    {
        result = check_timer();
    }

    if (E_OK == result)
    {
        /* TRIG is driven low before it becomes an output. Both pins leave
         * UART0. ECHO is pulled down so that an unplugged sensor reads
         * "no rise" rather than noise. */
        (void)gpio_set_val(OBSTACLE_TRIG_PIN, 0U);
        (void)gpio_set_pin(OBSTACLE_TRIG_PIN, GPIO_MODE_OUT);
        /* The BSP turns the input buffer off for an output. P4 reads TRIG back, so
         * turn it on again. */
        set_w(GPIO(OBSTACLE_TRIG_PIN), PAD_INPUT_ENABLE);
        (void)gpio_set_pin(OBSTACLE_ECHO_PIN, GPIO_MODE_IN);
        clr_w(GPIO(OBSTACLE_ECHO_PIN), PAD_PULL_UP);
        set_w(GPIO(OBSTACLE_ECHO_PIN), PAD_PULL_DOWN);
        result = servo_pwm_init();
    }

    if (E_OK == result)
    {
        g_echo.result = 0U;
        result     = gpio_irq_add_handler(TP_PRC1, ECHO_BIT, echo_isr);
    }

    return (result);
}

/*!
 * @brief Point the servo, clamped to +-30 degrees.
 *
 * @param[in] deci Angle in deci-degrees, positive = left.
 */
void scan_hw_servo_set (int16_t deci)
{
    int32_t angle    = deci;
    int32_t pulse_us = 0;

    if (angle > OBSTACLE_SERVO_LIMIT_DECI)
    {
        angle = OBSTACLE_SERVO_LIMIT_DECI;
    }
    else if (angle < -OBSTACLE_SERVO_LIMIT_DECI)
    {
        angle = -OBSTACLE_SERVO_LIMIT_DECI;
    }
    else
    {
        /* The angle is already inside the limits. */
    }

    pulse_us =
        SERVO_CENTRE_US +
        (((OBSTACLE_SERVO_DIR * angle) * SERVO_US_PER_10DEG) / SERVO_SCALE_DIV);

    if (pulse_us < SERVO_MIN_US)
    {
        pulse_us = SERVO_MIN_US;
    }
    else if (pulse_us > SERVO_MAX_US)
    {
        pulse_us = SERVO_MAX_US;
    }
    else
    {
        /* The pulse is already inside the servo's range. */
    }

    /* Cast is safe: pulse_us is 500..2400 here. One count is 1 us. */
    (void)pwm_set_cc(OBSTACLE_SERVO_PIN, (UW)pulse_us);
}

/*!
 * @brief Read the 1 MHz TIMER.
 *
 * @return Low 32 bits of the microsecond counter.
 */
uint32_t scan_hw_now_us (void)
{
    return (in_w(TIMER_TIMERAWL));
}

/*!
 * @brief Empty the echo result for the next ping.
 *
 * Interrupts are paused on this core for two statements (about 0.2 us)
 * so the ISR cannot write while the box is being emptied.
 * WARNING: Keep this block tiny. Do not add code between DI and EI.
 *
 * @return True if ECHO is already high (STUCK: do not trigger).
 */
bool scan_hw_rearm (void)
{
    UINT intsts = 0U;
    bool b_high = false;

    DI(intsts);
    g_echo.result = 0U;
    b_high     = scan_hw_is_echo_high();
    EI(intsts);

    return (b_high);
}

/*!
 * @brief Send the 12 us trigger pulse on GP0 and read the pad back (P4).
 *
 * @return True if the pad read high at the end of the pulse. Always true when
 *         OBSTACLE_TRIG_READBACK is 0.
 */
bool scan_hw_trigger (void)
{
    bool b_high = true;

    (void)gpio_set_val(OBSTACLE_TRIG_PIN, 1U);
    WaitUsec(OBSTACLE_TRIG_PULSE_US);
#if (0U != OBSTACLE_TRIG_READBACK)
    b_high = (0U != gpio_get_val(OBSTACLE_TRIG_PIN));
#endif
    (void)gpio_set_val(OBSTACLE_TRIG_PIN, 0U);

    return (b_high);
}

/*!
 * @brief Read the echo result the ISR writes.
 *
 * @return The echo result word (width 16 | RISE | DONE).
 */
uint32_t scan_hw_echo_result (void)
{
    return (g_echo.result);
}

/*!
 * @brief Read the ECHO pin.
 *
 * @return True if ECHO is high.
 */
bool scan_hw_is_echo_high (void)
{
    return (0U != (in_w(GPIO_IN) & ECHO_BIT));
}

/*!
 * @brief Make the 1 MHz TIMER count: release its reset if held, and start
 *        the watchdog tick if it is not already set for 1 us.
 *
 * NOTE: This only ever releases or starts; it never asserts a reset, so a
 * TIMER that another buddy already uses keeps running undisturbed.
 */
static void start_timer (void)
{
    uint32_t       spins    = 0U;
    uint32_t const tick     = in_w(WATCHDOG_TICK);
    uint32_t const tick_1us = TICK_ENABLE | TICK_CYCLES_1US;

    if (0U != (in_w(RESETS_RESET) & RESET_TIMER_BIT))
    {
        clr_w(RESETS_RESET, RESET_TIMER_BIT);
        while ((spins < RESET_SPINS) &&
               (0U == (in_w(RESETS_RESET_DONE) & RESET_TIMER_BIT)))
        {
            spins++;
        }
    }

    if (tick_1us != (tick & (TICK_ENABLE | TICK_CYCLES_MASK)))
    {
        out_w(WATCHDOG_TICK, tick_1us);
    }
}

/*!
 * @brief Check that the 1 MHz TIMER is ticking; echo widths depend on it.
 *
 * @return E_OK, or E_SYS if it moved too little.
 */
static ER check_timer (void)
{
    ER       result   = E_SYS;
    uint32_t start_us = 0U;
    uint32_t end_us   = 0U;

    start_timer();
    start_us = in_w(TIMER_TIMERAWL);
    WaitUsec(OBSTACLE_TIMER_CHECK_US);
    end_us = in_w(TIMER_TIMERAWL);

    /* Unsigned subtraction is wrap-safe. */
    if ((end_us - start_us) >= OBSTACLE_TIMER_CHECK_MIN_TICKS)
    {
        result = E_OK;
    }

    return (result);
}

/*!
 * @brief Release PWM from reset if it is held. Never asserts a reset.
 *
 * @return True if PWM is out of reset.
 */
static bool is_pwm_out_of_reset (void)
{
    uint32_t spins = 0U;

    if (0U != (in_w(RESETS_RESET) & RESET_PWM_BIT))
    {
        clr_w(RESETS_RESET, RESET_PWM_BIT);
        while ((spins < RESET_SPINS) &&
               (0U == (in_w(RESETS_RESET_DONE) & RESET_PWM_BIT)))
        {
            spins++;
        }
    }

    return (0U != (in_w(RESETS_RESET_DONE) & RESET_PWM_BIT));
}

/*!
 * @brief Servo PWM: 125 MHz / 125 = 1 MHz, TOP 19999, so 50 Hz.
 *
 * @return E_OK; E_SYS if PWM stays in reset; E_OBJ if slice 6 is in use.
 */
static ER servo_pwm_init (void)
{
    ER result = E_OK;

    if (!is_pwm_out_of_reset())
    {
        result = E_SYS;
    }
    else if (0U != (in_w(SERVO_SLICE_BASE + PWM_CHx_CSR) & PWM_CSR_ENABLE))
    {
        result = E_OBJ;
    }
    else
    {
        out_w(SERVO_SLICE_BASE + PWM_CHx_DIV,
              OBSTACLE_PWM_DIV_INT << OBSTACLE_PWM_DIV_SHIFT);
        (void)pwm_set_wrap(OBSTACLE_SERVO_PIN, OBSTACLE_PWM_TOP);
        scan_hw_servo_set(0);
        (void)pwm_set_pin(OBSTACLE_SERVO_PIN);
        (void)pwm_set_enabled(OBSTACLE_SERVO_PIN, TRUE);
    }

    return (result);
}

/*!
 * @brief ISR helper for a rising level: remember the time, set RISE.
 *
 * @param[in] echo_result    Echo result the ISR read.
 * @param[in] now_us TIMER value at the edge.
 */
static void echo_rise (uint32_t echo_result, uint32_t now_us)
{
    if (0U == (echo_result & OBSTACLE_ECHO_RISE))
    {
        g_echo.rise_us = now_us;
        g_echo.result     = OBSTACLE_ECHO_RISE;
    }
}

/*!
 * @brief ISR helper for a falling level: store width | RISE | DONE in one
 *        write and wake the task. A fall with no rise stores DONE alone,
 *        which is classed GLITCH.
 *
 * @param[in] echo_result    Echo result the ISR read.
 * @param[in] now_us TIMER value at the edge.
 */
static void echo_fall (uint32_t echo_result, uint32_t now_us)
{
    uint32_t width_us = 0U;

    if (0U != (echo_result & OBSTACLE_ECHO_RISE))
    {
        /* Unsigned subtraction is wrap-safe. */
        width_us = now_us - g_echo.rise_us;
        if (width_us > OBSTACLE_ECHO_WIDTH_MASK)
        {
            width_us = OBSTACLE_ECHO_WIDTH_MASK;
        }
    }

    g_echo.result = (echo_result & OBSTACLE_ECHO_RISE) | OBSTACLE_ECHO_DONE | width_us;
    (void)tk_set_flg(g_echo.h_flag, OBSTACLE_FLG_ECHO);
}

/*!
 * @brief ECHO edge ISR, called by the shared dispatcher on core 0.
 *
 * NOTE: There is no ISR #pragma (BARR-C 6.5.a). The kernel's interrupt
 * entry runs the shared dispatcher, which calls this as an ordinary C
 * function, so it must not be marked as an interrupt.
 * Short on purpose: a few register reads and one kernel call.
 *
 * @param[in] p_snapshot Edge snapshot from the dispatcher.
 */
static void echo_isr (const gpio_edge_snapshot_t * p_snapshot)
{
    uint32_t const now_us = p_snapshot->timestamp_us;
    uint32_t const echo_result    = g_echo.result;
    bool const     b_high = scan_hw_is_echo_high();

    if (0U != (echo_result & OBSTACLE_ECHO_DONE))
    {
        /* This ping is finished; later edges wait for the next re-arm. */
    }
    else if (b_high)
    {
        echo_rise(echo_result, now_us);
    }
    else
    {
        echo_fall(echo_result, now_us);
    }
}

/*** end of file ***/

