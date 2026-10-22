/** @file buddy2_route_check.c
 * @brief GP20-started approximate50cm / left-wheel3revs right /10cm demo.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tm/tmonitor.h>
#include "encoder_capture.h"
#include "motor_pwm.h"
#include "route_demo.h"
#include "speed_probe.h"
#include "usb_console_compat.h"

#define START_PIN 20U
#define STOP_PIN 21U
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)
#define TRACE_ROWS 96U

typedef struct
{
    uint32_t time_us;
    unsigned int phase;
    int32_t edges[2];
    int32_t speed[2];
    int32_t target[2];
    int16_t duty[2];
    uint32_t flags;
} route_row_t;

static motion_control_t controller;
static motion_sampler_t segment_sampler;
static route_demo_t route;
static route_row_t trace[TRACE_ROWS];
static unsigned int rows;
static unsigned int dropped;
static uint32_t max_interval_us;
static uint32_t last_trace_us;
static uint32_t idle_invalid[2];

static motion_apply_result_t apply_output (void * context, int16_t left, int16_t right)
{
    ER result;

    (void)context;
    result = motor_pwm_set_pair(left, right);
    if (result == E_OK)
    {
        return MOTION_APPLIED;
    }
    if (result == E_BUSY && motor_pwm_brake() == E_OK)
    {
        return MOTION_BRAKED_RETRY;
    }
    return MOTION_OUTPUT_FAILED;
}

static bool brake_output (void * context, bool latch)
{
    (void)context;
    return (latch ? motor_pwm_latch_stop() : motor_pwm_brake()) == E_OK;
}

static bool capture (motion_input_t * p_input)
{
    encoder_capture_sample_t raw;

    if (encoder_capture_read(&raw) != E_OK)
    {
        return false;
    }
    p_input->timestamp_us = raw.timestamp_us;
    p_input->wheel[0] = raw.wheel[0];
    p_input->wheel[1] = raw.wheel[1];
    return true;
}

static void record (const motion_sample_t * p_sample, bool phase_changed)
{
    unsigned int wheel;
    route_row_t row = {0};

    if (rows != 0U && !phase_changed &&
        (uint32_t)(p_sample->timestamp_us - last_trace_us) < 1000000U)
    {
        return;
    }
    row.time_us = p_sample->timestamp_us;
    row.phase = (unsigned int)route.phase;
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        row.edges[wheel] = (int32_t)p_sample->wheel[wheel].net_edges;
        row.speed[wheel] = p_sample->wheel[wheel].velocity_um_s;
        row.target[wheel] = controller.status.target_um_s[wheel];
        row.duty[wheel] = controller.status.duty[wheel];
        row.flags |= (p_sample->wheel[wheel].speed_valid ? 1U : 0U) << wheel;
        row.flags |= (p_sample->wheel[wheel].odometry_valid ? 1U : 0U) << (wheel + 2U);
        row.flags |= (p_sample->wheel[wheel].scale_verified ? 1U : 0U) << (wheel + 4U);
    }
    last_trace_us = p_sample->timestamp_us;
    if (rows < TRACE_ROWS)
    {
        trace[rows++] = row;
    }
    else
    {
        dropped++;
    }
}

static void report (ER error, uint32_t elapsed_us)
{
    unsigned int index;

    tm_printf((UB *)"ROUTE TRACE BEGIN rows=%u dropped=%u\n", rows, dropped);
    tm_printf((UB *)"END phase=%u route_error=%u state=%u fault=%u api=%d brake_failed=%u\n",
        (unsigned int)route.phase, (unsigned int)route.error,
        (unsigned int)controller.status.state, (unsigned int)controller.status.fault,
        error, controller.status.brake_failed ? 1U : 0U);
    tm_printf((UB *)"SET start_us=%u elapsed_us=%u speed_um_s=50000 cap_permille=200 max_interval_us=%u\n",
        route.started_us, elapsed_us, max_interval_us);
    tm_printf((UB *)"GOALS first=%d,%d turn=%d,0 last=%d,%d; scales UNVERIFIED\n",
        (int)route.goals[0][0], (int)route.goals[0][1], (int)route.goals[1][0],
        (int)route.goals[2][0], (int)route.goals[2][1]);
    tm_printf((UB *)"BASELINE idle_invalid_left=%u idle_invalid_right=%u; explicit braked run baseline\n",
        idle_invalid[0], idle_invalid[1]);
    tm_printf((UB *)"PHASE first=0 pause=1 right=2 pause=3 last=4 done=5 aborted=6\n");
    tm_printf((UB *)"ROW sample_us phase left_edges right_edges left_um_s right_um_s target_left target_right duty_left duty_right flags\n");
    for (index = 0U; index < rows; index++)
    {
        const route_row_t * row = &trace[index];

        tm_printf((UB *)"DATA %u %u %d %d %d %d %d %d %d %d %u\n",
            row->time_us, row->phase, row->edges[0], row->edges[1],
            row->speed[0], row->speed[1], row->target[0], row->target[1],
            (int)row->duty[0], (int)row->duty[1], row->flags);
        (void)tk_dly_tsk(10);
    }
    tm_printf((UB *)"ROUTE TRACE COMPLETE rows=%u; stopped until reboot\n", rows);
}

static void route_task (INT start_code, void * p_extra)
{
    motion_control_config_t config = speed_probe_configuration();
    motion_output_t output = {NULL, apply_output, brake_output};
    motion_input_t baseline;
    motor_pulse_t gate = {0};
    ER error;
    uint32_t ready_us = 0U;
    uint32_t countdown_us;
    uint32_t elapsed;
    unsigned int wait;

    (void)start_code;
    (void)p_extra;
    error = motor_pwm_init();
    if (error == E_OK)
    {
        error = encoder_capture_init();
    }
    (void)gpio_set_pin(START_PIN, GPIO_MODE_IN);
    (void)gpio_set_pin(STOP_PIN, GPIO_MODE_IN);
    clr_w(GPIO(START_PIN), GPIO_PDE);
    set_w(GPIO(START_PIN), GPIO_PUE | GPIO_SHEMITT);
    clr_w(GPIO(STOP_PIN), GPIO_PDE);
    set_w(GPIO(STOP_PIN), GPIO_PUE | GPIO_SHEMITT);
    if (error != E_OK)
    {
        motor_pwm_boot_brake();
        (void)motor_pwm_latch_stop();
        tm_printf((UB *)"ROUTE startup failed=%d; no movement\n", error);
        (void)tk_slp_tsk(TMO_FEVR);
        return;
    }
    for (wait = 0U; wait < 200U && tm_usb_state() < 2U; wait++)
    {
        (void)tk_dly_tsk(100);
    }
    tm_printf((UB *)"Buddy2 ROUTE: approx50cm, left3revs/rightbraked, approx10cm\n");
    tm_printf((UB *)"GP20 once starts; GP21 latched STOP. Clear floor/cable; NO obstacle sensors\n");
    for (;;)
    {
        uint32_t now = in_w(TIMER_RAW_LOW);

        if (gpio_get_val(STOP_PIN) == 0U)
        {
            (void)motor_pwm_latch_stop();
            tm_printf((UB *)"ROUTE stopped before start; reboot required\n");
            (void)tk_slp_tsk(TMO_FEVR);
            return;
        }
        if (motor_pulse_update(&gate, now, gpio_get_val(START_PIN) == 0U, false) != MOTOR_PULSE_OFF)
        {
            break;
        }
        if ((uint32_t)(now - ready_us) >= 1000000U)
        {
            tm_printf((UB *)"READY route GP20=%u GP21=%u\n",
                gpio_get_val(START_PIN), gpio_get_val(STOP_PIN));
            ready_us = now;
        }
        (void)tk_dly_tsk(10);
    }
    tm_printf((UB *)"START accepted; route starts in2seconds; hands clear\n");
    /* Remain braked while the operator releases the button. */
    countdown_us = in_w(TIMER_RAW_LOW);
    while ((uint32_t)(in_w(TIMER_RAW_LOW) - countdown_us) < 2000000U)
    {
        if (gpio_get_val(STOP_PIN) == 0U)
        {
            (void)motor_pwm_latch_stop();
            tm_printf((UB *)"ROUTE stopped during countdown; reboot required\n");
            (void)tk_slp_tsk(TMO_FEVR);
            return;
        }
        (void)tk_dly_tsk(10);
    }
    if (!capture(&baseline))
    {
        error = E_IO;
    }
    else
    {
        idle_invalid[0] = baseline.wheel[0].invalid;
        idle_invalid[1] = baseline.wheel[1].invalid;
        if (!motion_control_init(&controller, &config, &baseline, &output) ||
            !motion_sampler_init(&segment_sampler, &config.sampling, &baseline) ||
            !route_demo_start(&route, config.sampling.wheel, baseline.timestamp_us))
        {
            error = E_SYS;
        }
    }
    while (error == E_OK && route.phase != ROUTE_DEMO_DONE && route.phase != ROUTE_DEMO_ABORTED)
    {
        motion_input_t input;
        motion_sample_t sample = {0};
        motion_command_t command = {1U, 0U, SPEED_PROBE_TTL_US, {0, 0}};
        route_demo_phase_t previous = route.phase;
        uint32_t now;
        bool stop;

        (void)tk_dly_tsk(10);
        stop = gpio_get_val(STOP_PIN) == 0U;
        if (!stop && !capture(&input))
        {
            error = E_IO;
            stop = true;
        }
        now = in_w(TIMER_RAW_LOW);
        stop = stop || gpio_get_val(STOP_PIN) == 0U;
        if (stop)
        {
            route_demo_update(&route, NULL, now, true);
            motion_control_tick(&controller, NULL, now, NULL, true);
            break;
        }
        if (!motion_sampler_update(&segment_sampler, &input, &sample))
        {
            error = E_IO;
            route_demo_update(&route, NULL, now, false);
        }
        else
        {
            unsigned int wheel;

            for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
            {
                if (sample.wheel[wheel].net_edges < INT32_MIN ||
                    sample.wheel[wheel].net_edges > INT32_MAX)
                {
                    error = E_PAR;
                }
            }
            if (sample.elapsed_us > max_interval_us)
            {
                max_interval_us = sample.elapsed_us;
            }
            route_demo_update(&route, &sample, now, error != E_OK);
        }
        route_demo_targets(&route, &command.target_um_s[0], &command.target_um_s[1]);
        command.issued_us = in_w(TIMER_RAW_LOW);
        stop = route.phase == ROUTE_DEMO_DONE || route.phase == ROUTE_DEMO_ABORTED ||
               gpio_get_val(STOP_PIN) == 0U;
        motion_control_tick(&controller, &input, in_w(TIMER_RAW_LOW), &command, stop);
        if (controller.status.state == MOTION_STOPPED && route.phase != ROUTE_DEMO_DONE)
        {
            route_demo_update(&route, NULL, in_w(TIMER_RAW_LOW), true);
        }
        if (controller.status.state == MOTION_FAULT)
        {
            error = E_SYS;
            route_demo_update(&route, NULL, in_w(TIMER_RAW_LOW), true);
        }
        if (error == E_OK && sample.elapsed_us != 0U)
        {
            record(&sample, route.phase != previous);
        }
    }
    elapsed = route.initialised ? in_w(TIMER_RAW_LOW) - route.started_us : 0U;
    /* Terminal stop before all frozen reporting; no automatic repeat. */
    motion_control_tick(&controller, NULL, in_w(TIMER_RAW_LOW), NULL, true);
    (void)motor_pwm_latch_stop();
    for (;;)
    {
        report(error, elapsed);
        (void)tk_dly_tsk(2000);
    }
}

EXPORT INT usermain (void)
{
    T_CTSK definition = {
        .task = route_task, .itskpri = 5, .stksz = 4096,
        .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1
    };
    ID task;

    motor_pwm_boot_brake();
    task = tk_cre_tsk(&definition);
    if (task <= E_OK || tk_sta_tsk(task, 0) != E_OK)
    {
        return 1;
    }
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
