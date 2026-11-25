/** Host probe for applied PWM duty using the actual car setup. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion/motor_setup.h"
#include "motion_service.h"
#include "platform/log.h"
#include "ir/line/line_control.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define TICK_US 10000U
#define TARGET_UM_S 150000
#define NOMINAL_EDGE_COUNT 18U
#define LOADED_EDGE_COUNT 4U
#define PIVOT_EDGE_COUNT 22U
#define FORMER_FIXED_DUTY 400
#define TRIAL_DUTY 450
#define PIVOT_DUTY 500

typedef struct
{
    int16_t left;
    int16_t right;
    bool    b_latched;
} motor_duty_t;

static controller_config_t g_config;

ER motion_service_init(motion_service_config_t const * p_config, ID task_id)
{
    assert(1 == task_id);
    g_config = p_config->control;
    return E_OK;
}

void log_write(log_sev_t sev, log_task_t task, log_mod_t mod, char const * p_fmt, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_fmt;
}

static motion_apply_result_t record_pwm(void * p_context, int16_t left, int16_t right)
{
    motor_duty_t * p_duty = p_context;

    p_duty->left = left;
    p_duty->right = right;
    return MOTION_APPLIED;
}

static bool record_brake(void * p_context, bool b_latched)
{
    motor_duty_t * p_duty = p_context;

    p_duty->b_latched = b_latched;
    (void)record_pwm(p_context, 0, 0);
    return true;
}

static void step_wheels(controller_t * p_control, motion_input_t * p_sample,
                        int32_t left_target, int32_t right_target,
                        uint32_t left_edges, uint32_t right_edges, bool b_scan_pivot)
{
    p_sample->timestamp_us += TICK_US;
    motion_command_t const command = {p_sample->timestamp_us / TICK_US,
        p_sample->timestamp_us, 100000U, {left_target, right_target}, b_scan_pivot};

    if (left_target > 0)
    {
        p_sample->wheel[0].negative += left_edges;
    }
    else
    {
        p_sample->wheel[0].positive += left_edges;
    }
    if (right_target > 0)
    {
        p_sample->wheel[1].positive += right_edges;
    }
    else
    {
        p_sample->wheel[1].negative += right_edges;
    }
    controller_tick(p_control, p_sample, p_sample->timestamp_us, &command, false);
}

static motor_duty_t run_wheels(uint32_t left_edges)
{
    motion_input_t sample = {0};
    motor_duty_t duty = {0};
    motion_output_t const out = {&duty, record_pwm, record_brake};
    controller_t control;

    assert(E_OK == motion_setup(1));
    assert(controller_init(&control, &g_config, &sample, &out));
    for (uint32_t tick = 1U; tick <= 40U; tick++)
    {
        step_wheels(&control, &sample, TARGET_UM_S, TARGET_UM_S,
                    left_edges, NOMINAL_EDGE_COUNT, false);
    }
    assert(MOTION_RUNNING == control.status.state);
    assert(control.status.sample.wheel[0].speed_valid);
    assert(control.status.sample.wheel[1].speed_valid);
    return duty;
}

static void check_pivot(int32_t direction)
{
    int32_t const left_target = direction * LINE_TURN_FAST_UM_S;
    motion_input_t sample = {0};
    motor_duty_t duty = {0};
    motion_output_t const out = {&duty, record_pwm, record_brake};
    controller_t control;

    assert(controller_init(&control, &g_config, &sample, &out));
    for (uint32_t tick = 0U; tick < 40U; tick++)
    {
        step_wheels(&control, &sample, left_target, -left_target,
                    LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, false);
        assert(abs(duty.left) <= PIVOT_DUTY && abs(duty.right) < FORMER_FIXED_DUTY);
    }
    (void)printf("Pivot targets %d/%d um/s, loaded duty %d/%d permille\n",
                 left_target, -left_target, duty.left, duty.right);
    (void)fflush(stdout);
    assert(MOTION_RUNNING == control.status.state);
    assert(duty.left * direction > TRIAL_DUTY);
    assert(duty.right * direction < 0);

    step_wheels(&control, &sample, left_target, -left_target,
                LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, true);
    assert(duty.left * direction > FORMER_FIXED_DUTY && abs(duty.left) <= TRIAL_DUTY);

    step_wheels(&control, &sample, left_target, -left_target, 0U, PIVOT_EDGE_COUNT, false);
    assert(abs(duty.left) <= FORMER_FIXED_DUTY);
    assert(MOTION_RUNNING == control.status.state);

    step_wheels(&control, &sample, left_target, -left_target,
                LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, false);
    assert(duty.left * direction > FORMER_FIXED_DUTY);

    sample.wheel[0].invalid++;
    step_wheels(&control, &sample, left_target, -left_target,
                LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, false);
    assert(!control.status.sample.wheel[0].speed_valid);
    assert(abs(duty.left) <= FORMER_FIXED_DUTY);
    assert(MOTION_RUNNING == control.status.state);

    step_wheels(&control, &sample, left_target, -left_target,
                LOADED_EDGE_COUNT, PIVOT_EDGE_COUNT, false);
    assert(duty.left * direction > FORMER_FIXED_DUTY);
    controller_tick(&control, NULL, sample.timestamp_us + TICK_US, NULL, true);
    assert(MOTION_STOPPED == control.status.state);
    assert(duty.b_latched && duty.left == 0 && duty.right == 0);
}

int main(int argc, char * argv[])
{
    if (argc != 2)
    {
        (void)fprintf(stderr, "Usage: %s unloaded|loaded|pivot\n", argv[0]);
        return EXIT_FAILURE;
    }

    if (strcmp(argv[1], "unloaded") == 0)
    {
        motor_duty_t const duty = run_wheels(NOMINAL_EDGE_COUNT);

        assert(duty.left == duty.right);
        assert(duty.left > 0 && duty.left < FORMER_FIXED_DUTY);
        (void)printf("Unloaded wheel duty: %d/%d permille\n", duty.left, duty.right);
        return EXIT_SUCCESS;
    }
    if (strcmp(argv[1], "loaded") == 0)
    {
        motor_duty_t const duty = run_wheels(LOADED_EDGE_COUNT);

        assert(duty.right > 0 && duty.right < FORMER_FIXED_DUTY);
        if (duty.left <= FORMER_FIXED_DUTY)
        {
            (void)fprintf(stderr, "Loaded wheel duty remained at %d permille\n", duty.left);
            return EXIT_FAILURE;
        }
        assert(duty.left <= TRIAL_DUTY);
        (void)printf("Loaded wheel duty: %d/%d permille\n", duty.left, duty.right);
        return EXIT_SUCCESS;
    }
    if (strcmp(argv[1], "pivot") == 0)
    {
        assert(E_OK == motion_setup(1));
        check_pivot(1);
        check_pivot(-1);
        return EXIT_SUCCESS;
    }
    (void)fprintf(stderr, "Unknown motor duty case: %s\n", argv[1]);
    return EXIT_FAILURE;
}
